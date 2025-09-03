#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# CC13xx/CC26xx/CC2538 serial bootloader flasher - BeaglePlay fixes
# - --play usa /dev/ttyS1 (fallback a /dev/ttyS4 si existe)
# - gpiod robusto: v2 (top-level o gpiod.line.*) y v1
# - fallback de baudrate a 115200 si el driver devuelve EIO al abrir
#

from __future__ import annotations

from subprocess import Popen, PIPE
import sys, getopt, glob, math, time, os, struct, binascii, traceback

# ---------- opcionales ----------
have_magic = False
try:
    import magic
    have_magic = hasattr(magic, "from_file")
except Exception:
    have_magic = False

have_hex_support = False
try:
    from intelhex import IntelHex
    have_hex_support = True
except Exception:
    have_hex_support = False

# ---------- gpiod detección robusta ----------
have_gpiod = False
gpiod_variant = "none"   # "v2", "v1", "bad", "none"
LineDirection = None
LineValue = None
try:
    import gpiod
    have_gpiod = True
    # 1) intenta enums en submódulo v2
    try:
        from gpiod.line import Direction as _Dir, Value as _Val  # v2 típico
        LineDirection, LineValue = _Dir, _Val
    except Exception:
        # 2) intenta enums top-level (algunas builds aliasan)
        LineDirection = getattr(gpiod, "LineDirection", None)
        LineValue = getattr(gpiod, "LineValue", None)

    if hasattr(gpiod, "LineSettings") and LineDirection and LineValue:
        gpiod_variant = "v2"
    elif hasattr(gpiod, "Chip") and hasattr(gpiod, "LINE_REQ_DIR_OUT"):
        gpiod_variant = "v1"
    elif hasattr(gpiod, "chip") and hasattr(gpiod.chip, "Chip"):
        gpiod_variant = "v1"
    else:
        gpiod_variant = "bad"
except Exception:
    have_gpiod = False
    gpiod_variant = "none"

# ---------- pyserial ----------
try:
    import serial
except ImportError:
    print(f"{sys.argv[0]} requiere pyserial (pip3 install pyserial)")
    sys.exit(1)

__version__ = "3.3-playfix-ttyS2-gpiodline"
QUIET = 5  # 0=silencioso, 10=debug

def mdebug(level: int, msg: str, end: str="\n"):
    if QUIET >= level:
        print(msg, end=end, file=sys.stderr)

# ---------- bootloader const ----------
CHIP_ID_STRS = {0xB964: "CC2538", 0xB965: "CC2538"}
RETURN_CMD_STRS = {0x40:"Success",0x41:"Unknown command",0x42:"Invalid command",0x43:"Invalid address",0x44:"Flash fail"}
COMMAND_RET_SUCCESS=0x40

class CmdException(Exception): pass

# =========================================================
#                   Firmware file
# =========================================================
class FirmwareFile(object):
    HEX_FILE_EXTENSIONS=("hex","ihx","ihex")
    def __init__(self, path: str):
        self._crc32=None
        firmware_is_hex=False
        if have_magic:
            try:
                mt = magic.from_file(path, mime=True)
                if mt=="text/plain":
                    firmware_is_hex=True
                    mdebug(5,"Firmware: Intel Hex")
                elif mt in ("application/octet-stream","application/x-dosexec"):
                    mdebug(5,"Firmware: Binario")
                else:
                    raise CmdException(f"No se reconoce el tipo (magic={mt})")
            except Exception:
                mdebug(5,"python-magic falló; se usará la extensión")
        if not have_magic:
            ext=os.path.splitext(path)[1][1:].lower()
            firmware_is_hex = ext in self.HEX_FILE_EXTENSIONS
            mdebug(5,"Asumiendo IntelHex" if firmware_is_hex else "Asumiendo .bin")
        if firmware_is_hex:
            if not have_hex_support:
                raise CmdException("Falta IntelHex. Instala 'pip3 install intelhex' o usa .bin")
            self.bytes = bytearray(IntelHex(path).tobinarray())
        else:
            with open(path,"rb") as f:
                self.bytes=bytearray(f.read())
    def crc32(self)->int:
        if self._crc32 is None:
            self._crc32 = binascii.crc32(bytearray(self.bytes)) & 0xFFFFFFFF
        return self._crc32

# =========================================================
#                 Serial / bootloader
# =========================================================
class CommandInterface(object):
    ACK_BYTE=0xCC
    NACK_BYTE=0x33

    def open(self, aport:str=None, abaud:int=500000):
        try:
            self.sp=serial.serial_for_url(aport, do_not_open=True, timeout=10, write_timeout=10)
        except AttributeError:
            self.sp=serial.Serial(port=None, timeout=10, write_timeout=10)
            self.sp.port=aport
        # set attrs
        try:
            self.sp.baudrate=abaud
            self.sp.bytesize=8
            self.sp.parity=serial.PARITY_NONE
            self.sp.stopbits=1
            self.sp.xonxoff=0
            self.sp.rtscts=0
            self.sp.timeout=0.5
        except Exception:
            pass
        # open with baud fallback if needed
        try:
            self.sp.open()
        except Exception as e:
            s=str(e)
            if any(k in s for k in ("Input/output error","configure port","EIO")) and abaud>115200:
                mdebug(5,f"Fallo configurar {aport} @ {abaud}. Reintento @115200…")
                self.sp.baudrate=115200
                self.sp.open()
                mdebug(5,"Abierto @115200 (fallback)")
            else:
                raise

    def _flush_input(self):
        if hasattr(self.sp,"reset_input_buffer"):
            self.sp.reset_input_buffer()
        elif hasattr(self.sp,"flushInput"):
            self.sp.flushInput()

    def invoke_bootloader(self, dtr_active_high=False, inverted=False, sonoff_usb=False, send_break=False, gpio: str|bool=False):
        if send_break:
            mdebug(5,"Enviando break para invocar BSL")
            to=self.sp.timeout
            self.sp.timeout=.1
            self.sp.send_break()
            self.sp.read(100)
            self.sp.timeout=to
            return
        # DTR/RTS toggle
        set_boot = self.sp.setRTS if inverted else self.sp.setDTR
        set_rst  = self.sp.setDTR if inverted else self.sp.setRTS
        if sonoff_usb:
            mdebug(5,"Secuencia Sonoff")
            set_boot(0); set_rst(1); set_boot(1); set_rst(0)
        else:
            set_boot(0 if dtr_active_high else 1)
            set_rst(0)
            set_rst(1)
            set_rst(0)
            time.sleep(0.002)
            set_boot(1 if dtr_active_high else 0)
        if gpio:
            _toggle_boot_reset_via_gpiod(gpio)
        time.sleep(0.1)

    def close(self):
        try:
            self.sp.close()
        except Exception:
            pass

    # --- low level ---
    def _wait_for_ack(self, info="", timeout=1.0):
        stop=time.time()+timeout
        got=bytearray(2)
        while got[-2]!=0x00 or got[-1] not in (self.ACK_BYTE,self.NACK_BYTE):
            got+=self._read(1)
            if time.time()>stop:
                raise CmdException(f"Timeout esperando ACK/NACK tras '{info}'")
        return 1 if got[-1]==self.ACK_BYTE else 0

    def _encode_addr(self, addr:int)->bytes:
        return bytes([(addr>>24)&0xFF,(addr>>16)&0xFF,(addr>>8)&0xFF,(addr>>0)&0xFF])

    def _decode_addr(self,b0,b1,b2,b3)->int:
        return ((b3<<24)|(b2<<16)|(b1<<8)|(b0<<0))

    def _calc_checks(self,cmd,addr,size):
        return ((sum(bytearray(self._encode_addr(addr)))+sum(bytearray(self._encode_addr(size)))+cmd)&0xFF)

    def _write(self,data,is_retry=False):
        if isinstance(data,int):
            goal=1
            written=self.sp.write(bytes([data]))
        elif isinstance(data,(bytes,bytearray)):
            goal=len(data)
            written=self.sp.write(data)
        else:
            raise CmdException(f"Tipo inválido _write: {type(data)}")
        if written<goal:
            if is_retry and written==0:
                raise CmdException("Falló escritura serie")
            return self._write(data if isinstance(data,int) else data[written:], True)

    def _read(self,n:int)->bytearray:
        return bytearray(self.sp.read(n))

    def sendAck(self):
        self._write(0x00)
        self._write(0xCC)

    def sendNAck(self):
        self._write(0x00)
        self._write(0x33)

    def receivePacket(self):
        got=self._read(2)
        size=got[0]
        chks=got[1]
        data=bytearray(self._read(size-2))
        if chks==(sum(data)&0xFF):
            self.sendAck()
            return data
        self.sendNAck()
        raise CmdException("Checksum paquete inválido")

    def sendSynch(self):
        self._flush_input()
        self._write(0x55)
        self._write(0x55)
        return self._wait_for_ack("Synch (0x55 0x55)",2)

    def checkLastCmd(self):
        stat=self.cmdGetStatus()
        if not stat:
            raise CmdException("Sin respuesta en status (¿bootloader deshabilitado?)")
        return 1 if stat[0]==COMMAND_RET_SUCCESS else 0

    # --- commands ---
    def cmdPing(self):
        self._write(3)
        self._write(0x20)
        self._write(0x20)
        mdebug(10, "*** Ping (0x20)")
        if self._wait_for_ack("Ping"):
            return self.checkLastCmd()

    def cmdReset(self):
        self._write(3)
        self._write(0x25)
        self._write(0x25)
        mdebug(10, "*** Reset (0x25)")
        if self._wait_for_ack("Reset"):
            return 1

    def cmdGetChipId(self):
        self._write(3)
        self._write(0x28)
        self._write(0x28)
        mdebug(10, "*** GetChipId (0x28)")
        if self._wait_for_ack("GetChipId"):
            v=self.receivePacket()
            if self.checkLastCmd():
                return (v[2]<<8)|v[3]
            raise CmdException("GetChipId falló")

    def cmdGetStatus(self):
        self._write(3)
        self._write(0x23)
        self._write(0x23)
        mdebug(10, "*** GetStatus (0x23)")
        if self._wait_for_ack("GetStatus"):
            return self.receivePacket()

    def cmdSetXOsc(self):
        self._write(3)
        self._write(0x29)
        self._write(0x29)
        mdebug(10, "*** SetXOsc (0x29)")
        if self._wait_for_ack("SetXOsc"):
            return 1

    def cmdRun(self,addr):
        self._write(7)
        self._write(self._calc_checks(0x22,addr,0))
        self._write(0x22)
        self._write(self._encode_addr(addr))
        mdebug(10, "*** Run (0x22)")
        return 1

    def cmdEraseMemory(self,addr,size):
        self._write(11)
        self._write(self._calc_checks(0x26,addr,size))
        self._write(0x26)
        self._write(self._encode_addr(addr))
        self._write(self._encode_addr(size))
        mdebug(10, "*** Erase (0x26)")
        if self._wait_for_ack("Erase",10):
            return self.checkLastCmd()

    def cmdBankErase(self):
        self._write(3)
        self._write(0x2C)
        self._write(0x2C)
        mdebug(10, "*** Bank Erase (0x2C)")
        if self._wait_for_ack("BankErase",10):
            return self.checkLastCmd()

    def cmdCRC32(self,addr,size):
        self._write(11)
        self._write(self._calc_checks(0x27,addr,size))
        self._write(0x27)
        self._write(self._encode_addr(addr))
        self._write(self._encode_addr(size))
        mdebug(10, "*** CRC32 (0x27)")
        if self._wait_for_ack("CRC32"):
            crc=self.receivePacket()
            if self.checkLastCmd():
                return self._decode_addr(crc[3],crc[2],crc[1],crc[0])

    def cmdCRC32CC26xx(self,addr,size):
        self._write(15)
        self._write(self._calc_checks(0x27,addr,size))
        self._write(0x27)
        self._write(self._encode_addr(addr))
        self._write(self._encode_addr(size))
        self._write(self._encode_addr(0))
        mdebug(10, "*** CRC32 (0x27) CC26xx")
        if self._wait_for_ack("CRC32 CC26xx"):
            crc=self.receivePacket()
            if self.checkLastCmd():
                return self._decode_addr(crc[3],crc[2],crc[1],crc[0])

    def cmdDownload(self,addr,size):
        if size%4!=0:
            raise Exception(f"Tamaño inválido {size} (múltiplo de 4)")
        self._write(11)
        self._write(self._calc_checks(0x21,addr,size))
        self._write(0x21)
        self._write(self._encode_addr(addr))
        self._write(self._encode_addr(size))
        mdebug(10, "*** Download (0x21)")
        if self._wait_for_ack("Download",2):
            return self.checkLastCmd()

    def cmdSendData(self,data:bytes):
        self._write(len(data)+3)
        self._write((sum(bytearray(data))+0x24)&0xFF)
        self._write(0x24)
        self._write(bytearray(data))
        mdebug(10, "*** Send Data (0x24)")
        if self._wait_for_ack("SendData",10):
            return self.checkLastCmd()

    def cmdMemRead(self,addr):
        self._write(8)
        self._write(self._calc_checks(0x2A,addr,4))
        self._write(0x2A)
        self._write(self._encode_addr(addr))
        self._write(4)
        mdebug(10, "*** MemRead (0x2A)")
        if self._wait_for_ack("MemRead"):
            d=self.receivePacket()
            if self.checkLastCmd():
                return d

    def cmdMemReadCC26xx(self,addr):
        self._write(9)
        self._write(self._calc_checks(0x2A,addr,2))
        self._write(0x2A)
        self._write(self._encode_addr(addr))
        self._write(1)  # width
        self._write(1)  # reads
        mdebug(10, "*** MemRead (0x2A) CC26xx")
        if self._wait_for_ack("MemRead CC26xx"):
            d=self.receivePacket()
            if self.checkLastCmd():
                return d

    def writeMemory(self,addr,data:bytes):
        trsz=248
        empty=bytearray((0xFF,)*trsz)
        offs=0
        lng=len(data)
        addr_set=0
        mdebug(5,f"Escribiendo {lng} bytes desde 0x{addr:08X}")
        while lng>trsz:
            if data[offs:offs+trsz]!=empty:
                if addr_set!=1:
                    self.cmdDownload(addr,lng)
                    addr_set=1
                mdebug(5,f" Write {trsz} @ 0x{addr:08X}\r","")
                sys.stdout.flush()
                self.cmdSendData(data[offs:offs+trsz])
            else:
                addr_set=0
            offs+=trsz
            addr+=trsz
            lng-=trsz
        mdebug(5,f"Write {lng} @ 0x{addr:08X}")
        self.cmdDownload(addr,lng)
        return self.cmdSendData(data[offs:offs+lng])

# =========================================================
#                         Chips
# =========================================================
class Chip:
    def __init__(self, ci: CommandInterface):
        self.command_interface=ci
        self.flash_start_addr=0x00000000
        self.has_cmd_set_xosc=False
        self.page_size=2048
    def page_align_up(self,v):
        return int(math.ceil(v/self.page_size)*self.page_size)
    def page_to_addr(self,pages):
        return [int(self.flash_start_addr)+int(p)*self.page_size for p in pages]
    def crc(self,addr,size):
        return getattr(self.command_interface,self.crc_cmd)(addr,size)
    def disable_bootloader(self):
        if not (conf.get("force") or query_yes_no("¡Vas a deshabilitar el bootloader! ¿Continuar?","no")):
            raise Exception("Abortado por el usuario.")
        pattern = struct.pack("<L", self.bootloader_dis_val)
        if self.command_interface.writeMemory(self.bootloader_address, pattern):
            mdebug(5, "Bootloader deshabilitado")
        else:
            raise CmdException("Fallo al deshabilitar bootloader")

class CC2538(Chip):
    def __init__(self,ci):
        super().__init__(ci)
        self.flash_start_addr=0x00200000
        self.addr_ieee_address_secondary=0x0027FFCC
        self.has_cmd_set_xosc=True
        self.bootloader_dis_val=0xEFFFFFFF
        self.crc_cmd="cmdCRC32"
        FLASH_CTRL_DIECFG0=0x400D3014
        FLASH_CTRL_DIECFG2=0x400D301C
        addr_ieee_address_primary=0x00280028
        ccfg_len=44
        model=self.command_interface.cmdMemRead(FLASH_CTRL_DIECFG0)
        size=(model[3]&0x70)>>4
        self.size=size*0x20000 if 0<size<=4 else 0x10000
        self.bootloader_address=self.flash_start_addr+self.size-ccfg_len
        pg=self.command_interface.cmdMemRead(FLASH_CTRL_DIECFG2)
        pg_major=(pg[2]&0xF0)>>4 or 1
        pg_minor=pg[2]&0x0F
        ti_oui=bytearray([0x00,0x12,0x4B])
        ieee=self.command_interface.cmdMemRead(addr_ieee_address_primary)
        ieee_end=self.command_interface.cmdMemRead(addr_ieee_address_primary+4)
        ieee = ieee+ieee_end if ieee[:3]==ti_oui else ieee_end+ieee
        mdebug(5,f"CC2538 PG{pg_major}.{pg_minor}: {self.size>>10}KB Flash, CCFG @ 0x{self.bootloader_address:08X}")
        mdebug(5,"IEEE primario: "+":".join("%02X"%x for x in ieee))
    def erase(self):
        mdebug(5,f"Borrando {self.size} bytes desde 0x{self.flash_start_addr:08X}")
        return self.command_interface.cmdEraseMemory(self.flash_start_addr,self.size)
    def read_memory(self,addr):
        d=self.command_interface.cmdMemRead(addr)
        return bytearray([d[x] for x in range(3,-1,-1)])

class CC26xx(Chip):
    def __init__(self,ci):
        super().__init__(ci)
        self.bootloader_dis_val=0x00000000
        self.crc_cmd="cmdCRC32CC26xx"
        self.page_size=4096
        ICEPICK_DEVICE_ID=0x50001318
        FCFG_USER_ID=0x50001294
        PRCM_RAMHWOPT=0x40082250
        FLASH_SIZE=0x4003002C
        addr_ieee_address_primary=0x500012F0
        ccfg_len=88
        ieee_sec_off=0x20
        bl_dis_off=0x30
        _=self.command_interface.cmdMemReadCC26xx(ICEPICK_DEVICE_ID)
        _=self.command_interface.cmdMemReadCC26xx(FCFG_USER_ID)
        self.size=self.command_interface.cmdMemReadCC26xx(FLASH_SIZE)[0]*self.page_size
        self.bootloader_address=self.size-ccfg_len+bl_dis_off
        self.addr_ieee_address_secondary=self.size-ccfg_len+ieee_sec_off
        ieee=self.command_interface.cmdMemReadCC26xx(addr_ieee_address_primary+4)[::-1]
        ieee+=self.command_interface.cmdMemReadCC26xx(addr_ieee_address_primary)[::-1]
        mdebug(5,f"CC13xx/26xx: {self.size>>10}KB Flash, CCFG.BL_CONFIG @ 0x{self.bootloader_address:08X}")
        mdebug(5,"IEEE primario: "+":".join("%02X"%x for x in ieee))
    def erase(self):
        mdebug(5,"Borrando todos los sectores del banco principal")
        return self.command_interface.cmdBankErase()
    def read_memory(self,addr):
        return self.command_interface.cmdMemReadCC26xx(addr)

# =========================================================
#                 Helpers CLI / GPIO
# =========================================================
def query_yes_no(q, default="yes"):
    valid={"yes":True,"y":True,"ye":True,"no":False,"n":False}
    prompt=" [Y/n] " if default=="yes" else (" [y/N] " if default=="no" else " [y/n] ")
    while True:
        sys.stdout.write(q+prompt)
        ch=input().lower()
        if default is not None and ch=="":
            return valid[default]
        if ch in valid:
            return valid[ch]
        sys.stdout.write("Responde yes/no (y/n).\n")

def parse_ieee_address(inaddr:str)->int:
    try:
        return int(inaddr,16)
    except ValueError:
        parts = inaddr.split(":") if ":" in inaddr else (inaddr.split("-") if "-" in inaddr else None)
        if not parts or len(parts)!=8:
            raise ValueError("IEEE address inválida (8 bytes)")
        addr=0
        for i,b in zip(range(8),parts):
            addr += int(b,16) << (56-(i*8))
        return addr

def _parse_gpio(spec:str):
    parts=[p.strip() for p in str(spec).split(",")]
    if len(parts)!=4:
        raise CmdException(f"GPIO spec inválida: {spec}")
    return (parts[0], int(parts[1]), parts[2], int(parts[3]))

def _chip_path(name:str)->str:
    return name if name.startswith("/dev/") else f"/dev/{name}"

def _toggle_boot_reset_via_gpiod(spec:str):
    if not have_gpiod:
        raise CmdException("Se pidió GPIO pero no se pudo importar 'gpiod'. Instala python3-libgpiod o 'pip install gpiod>=2'.")
    if gpiod_variant=="bad":
        raise CmdException("El 'gpiod' importado no expone API válida (ni v1 ni v2). Reinstala bindings correctos.")

    chip_boot, line_boot, chip_rst, line_rst = _parse_gpio(spec)
    mdebug(5, f"Usando GPIO para BOOT ({chip_boot} {line_boot}) y RESET ({chip_rst} {line_rst})")

    if gpiod_variant=="v2":
        # v2: usar LineValue.{INACTIVE,ACTIVE} en set_values (no 0/1)
        def request_v2(chip_name, lines):
            cfg = {
                ln: gpiod.LineSettings(
                    direction=LineDirection.OUTPUT,
                    output_value=LineValue.INACTIVE
                ) for ln in lines
            }
            try:
                chip = gpiod.Chip(_chip_path(chip_name))
                if hasattr(chip, "request_lines"):
                    req = chip.request_lines(consumer="cc1352-flasher", config=cfg)
                else:
                    req = gpiod.request_lines(_chip_path(chip_name), consumer="cc1352-flasher", config=cfg)
                return chip, req
            except Exception:
                req = gpiod.request_lines(_chip_path(chip_name), consumer="cc1352-flasher", config=cfg)
                return None, req

        if chip_boot == chip_rst:
            chip, req = request_v2(chip_boot, [line_boot, line_rst])
            try:
                # BOOT=0, RESET=0
                req.set_values({line_boot: LineValue.INACTIVE, line_rst: LineValue.INACTIVE})
                time.sleep(0.2)
                # RESET=1
                req.set_values({line_rst: LineValue.ACTIVE})
                time.sleep(0.2)
                # BOOT=1
                req.set_values({line_boot: LineValue.ACTIVE})
                time.sleep(0.2)
            finally:
                try: req.release()
                except Exception: pass
                try: chip and chip.close()
                except Exception: pass
        else:
            cb, rq_b = request_v2(chip_boot, [line_boot])
            cr, rq_r = request_v2(chip_rst, [line_rst])
            try:
                # BOOT=0, RESET=0
                rq_b.set_values({line_boot: LineValue.INACTIVE})
                rq_r.set_values({line_rst: LineValue.INACTIVE})
                time.sleep(0.2)
                # RESET=1
                rq_r.set_values({line_rst: LineValue.ACTIVE})
                time.sleep(0.2)
                # BOOT=1
                rq_b.set_values({line_boot: LineValue.ACTIVE})
                time.sleep(0.2)
            finally:
                for req in (rq_b, rq_r):
                    try: req.release()
                    except Exception: pass
                for chip in (cb, cr):
                    try: chip and chip.close()
                    except Exception: pass

    else:
        # v1: enteros 0/1 (igual que antes)
        def open_chip(name):
            try:
                return gpiod.Chip(name, gpiod.Chip.OPEN_BY_NAME)
            except Exception:
                try:
                    return gpiod.Chip(_chip_path(name))
                except Exception:
                    return gpiod.Chip(name)

        bc = open_chip(chip_boot)
        rc = open_chip(chip_rst)
        try:
            bl = bc.get_line(line_boot)
            rl = rc.get_line(line_rst)
            bl.request(consumer="cc1352-flasher", type=gpiod.LINE_REQ_DIR_OUT)
            rl.request(consumer="cc1352-flasher", type=gpiod.LINE_REQ_DIR_OUT)

            bl.set_value(0); rl.set_value(0); time.sleep(0.2)
            rl.set_value(1); time.sleep(0.2)
            bl.set_value(1); time.sleep(0.2)
        finally:
            for x in (bl, rl):
                try: x.release()
                except Exception: pass
            for c in (bc, rc):
                try: c.close()
                except Exception: pass
                
def _parse_range_values(device, values):
    if len(values) and len(values)<3:
        out=[]
        for v in values:
            try:
                iv=int(v)
            except ValueError:
                iv=int(v,16)
            if iv%int(device.page_size)!=0:
                raise ValueError(f"No alineado a page_size={device.page_size}")
            out.append(iv)
        return out
    raise ValueError("Rango no es página/dirección válido")

def parse_page_address_range(device, pg_range):
    vals=pg_range.split(",")
    page_addr=[]
    if vals[0].isalpha():
        vals[0].lower()
        if vals[0] in ("p","page"):
            vals[1:]=device.page_to_addr(vals[1:])
        elif vals[0] not in ("a","address"):
            raise ValueError("Prefijo debe ser a/address o p/page")
        page_addr.extend(_parse_range_values(device, vals[1:]))
    else:
        page_addr.extend(_parse_range_values(device, vals))
    return [page_addr[0], device.page_size] if len(page_addr)==1 else [page_addr[0], (page_addr[1]-page_addr[0])]

def print_version():
    try:
        p=Popen(["git","describe","--tags","--match","[0-9]*"],stdout=PIPE,stderr=PIPE)
        p.stderr.close()
        version=p.stdout.readlines()[0].decode().strip()
    except Exception:
        version=__version__
    print(f"{sys.argv[0]} {version}")

# =========================================================
#                          MAIN
# =========================================================
if __name__=="__main__":
    conf={"port":"auto","baud":500000,"force_speed":0,"address":None,"force":0,"erase":0,"write":0,"erase_write":0,
          "erase_page":0,"verify":0,"read":0,"len":0x80000,"fname":"","append":"","ieee_address":0,
          "bootloader_active_high":False,"bootloader_invert_lines":False,"bootloader_sonoff_usb":False,
          "bootloader_send_break":False,"gpio":None,"disable-bootloader":0}

    try:
        opts,args=getopt.getopt(sys.argv[1:],"DhqVfeE:wWvrp:b:a:l:i:",[
            "help","ieee-address=","erase-write=","erase-page=","append=","gpio=","disable-bootloader",
            "bootloader-active-high","bootloader-invert-lines","bootloader-sonoff-usb","bootloader-send-break",
            "bcf","play","version"])
    except getopt.GetoptError as err:
        print(str(err))
        print_version()
        sys.exit(2)

    for o,a in opts:
        if o=="-V":
            QUIET=10
        elif o=="-q":
            QUIET=0
        elif o in ("-h","--help"):
            print_version()
            print("Usa -V para más verbosidad. -e/-w/-v para erase/write/verify.")
            sys.exit(0)
        elif o=="--append":
            conf["append"]=str(a)
        elif o=="-f":
            conf["force"]=1
        elif o=="-e":
            conf["erase"]=1
        elif o=="-w":
            conf["write"]=1
        elif o in ("-W","--erase-write"):
            conf["erase_write"]=1
        elif o in ("-E","--erase-page"):
            conf["erase_page"]=str(a)
        elif o=="-v":
            conf["verify"]=1
        elif o=="-r":
            conf["read"]=1
        elif o=="-p":
            conf["port"]=a
        elif o=="-b":
            conf["baud"]=int(a)
            conf["force_speed"]=1
        elif o=="-a":
            conf["address"]=int(a,0)
        elif o=="-l":
            conf["len"]=int(a,0)
        elif o in ("-i","--ieee-address"):
            conf["ieee_address"]=str(a)
        elif o=="--bootloader-active-high":
            conf["bootloader_active_high"]=True
        elif o=="--bootloader-invert-lines":
            conf["bootloader_invert_lines"]=True
        elif o=="--bootloader-sonoff-usb":
            conf["bootloader_sonoff_usb"]=True
        elif o=="--bootloader-send-break":
            conf["bootloader_send_break"]=True
        elif o in ("-D","--disable-bootloader"):
            conf["disable-bootloader"]=1
        elif o=="--bcf":
            conf["erase"]=1
            conf["write"]=1
            conf["verify"]=1
            conf["append"]="/zephyr/zephyr.bin"
            conf["bootloader_send_break"]=True
        elif o=="--play":
            conf["erase"]=1
            conf["write"]=1
            conf["verify"]=1
            conf["gpio"]="gpiochip2,13,gpiochip2,14"
            default_port="/dev/ttyS1"
            if not os.path.exists(default_port) and os.path.exists("/dev/ttyS4"):
                default_port="/dev/ttyS4"
            conf["port"]=default_port
            conf["append"]="/zephyr/zephyr.bin"
        elif o=="--gpio":
            conf["gpio"]=str(a)
        elif o=="--version":
            print_version()
            sys.exit(0)
        else:
            assert False,"Opción no manejada"

    try:
        if conf["write"] or conf["erase_write"] or conf["read"] or conf["verify"]:
            try:
                conf["fname"]=(args[0]+conf["append"]) if conf["append"] else args[0]
            except Exception:
                raise Exception("Falta archivo/carpeta base (.bin/.hex o build dir)")
            if (conf["write"] or conf["verify"]) and not os.path.exists(conf["fname"]):
                raise Exception(f"No existe: {conf['fname']}")

        if (conf["write"] and conf["read"]) or (conf["erase_write"] and conf["read"]):
            if not (conf["force"] or query_yes_no("Leerás y escribirás el mismo archivo. ¿Continuar?","no")):
                raise Exception("Abortado.")

        if conf["len"]<0:
            raise Exception(f"Length inválido: {conf['len']}")

        if conf["port"]=="auto":
            ports=[]
            for name in ["ttyACM","tty.usbserial","ttyUSB","tty.usbmodem","tty.SLAB_USBtoUART","ttyS2","ttyS4"]:
                ports+=glob.glob(f"/dev/{name}*")
            ports=sorted(ports)
            if ports:
                conf["port"]=ports[0]
            else:
                raise Exception("No se encontró puerto serie.")

        cmd=CommandInterface()
        cmd.open(conf["port"], conf["baud"])
        cmd.invoke_bootloader(
            dtr_active_high=conf["bootloader_active_high"],
            inverted=conf["bootloader_invert_lines"],
            sonoff_usb=conf["bootloader_sonoff_usb"],
            send_break=conf["bootloader_send_break"],
            gpio=conf["gpio"]
        )
        mdebug(5,f"Puerto {conf['port']} abierto @ {cmd.sp.baudrate}")

        if conf["write"] or conf["erase_write"] or conf["verify"]:
            mdebug(5,f"Leyendo firmware: {conf['fname']}")
            fw=FirmwareFile(conf["fname"])

        mdebug(5,"Conectando con el target…")
        if not cmd.sendSynch():
            raise CmdException("Sin respuesta a sync (0x55 0x55)")

        chip_id=cmd.cmdGetChipId()
        device = CC26xx(cmd) if CHIP_ID_STRS.get(chip_id,None) is None else CC2538(cmd)
        if conf["address"] is None:
            conf["address"]=device.flash_start_addr

        # (opcional) SetXOsc para CC2538
        if conf["force_speed"] != 1 and getattr(device,"has_cmd_set_xosc",False):
            if cmd.cmdSetXOsc():
                cmd.close()
                conf["baud"]=1000000
                cmd.open(conf["port"], conf["baud"])
                mdebug(6,f"Reabriendo {conf['port']} a {conf['baud']}")
                if cmd.sendSynch()!=1:
                    raise CmdException("No hay sync tras SetXOsc (revisa cristal externo)")
            else:
                raise CmdException("SetXOsc falló (prueba forzar velocidad)")

        if conf["erase"]:
            mdebug(5,"Borrado masivo…")
            if not device.erase():
                raise CmdException("Borrado falló")

        if conf["erase_page"]:
            er=parse_page_address_range(device, conf["erase_page"])
            mdebug(5,f"Borrando {er[1]} bytes en 0x{er[0]:x}")
            cmd.cmdEraseMemory(er[0],er[1])
            mdebug(5,"Borrado parcial OK")

        if conf["write"]:
            if not cmd.writeMemory(conf["address"], fw.bytes):
                raise CmdException("Escritura falló")
            mdebug(5,"Escritura OK")

        if conf["erase_write"]:
            erase_len = device.page_align_up(len(fw.bytes))
            erase_len=min(erase_len, getattr(device,"size",erase_len))
            if cmd.cmdEraseMemory(conf["address"], erase_len):
                mdebug(5,"Borrado previo OK")
            if not cmd.writeMemory(conf["address"], fw.bytes):
                raise CmdException("Escritura falló")
            mdebug(5,"Escritura OK (tras borrado)")

        if conf["verify"]:
            mdebug(5,"Verificando CRC32…")
            crc_local=fw.crc32()
            crc_target=device.crc(conf["address"], len(fw.bytes))
            if crc_local!=crc_target:
                cmd.cmdReset()
                raise Exception(f"CRC mismatch: local=0x{crc_local:08x} target=0x{crc_target:08x}")
            mdebug(5,f"Verificado (0x{crc_local:08x})")

        if conf["ieee_address"]:
            ieee=parse_ieee_address(conf["ieee_address"])
            mdebug(5,"Seteando IEEE "+":".join(f"{b:02x}" for b in struct.pack(">Q",ieee)))
            if not cmd.writeMemory(device.addr_ieee_address_secondary, struct.pack("<Q",ieee)):
                raise CmdException("Fallo al escribir IEEE")

        if conf["read"]:
            length=(conf["len"]+3)&~0x03
            mdebug(5,f"Leyendo {length} bytes desde 0x{conf['address']:x} → {conf['fname']}")
            with open(conf["fname"],"wb") as f:
                for i in range(0,length>>2):
                    d=device.read_memory(conf["address"]+(i*4))
                    f.write(d)
            mdebug(5,"Lectura OK")

        cmd.cmdReset()

    except Exception as err:
        if QUIET>=10:
            traceback.print_exc()
        sys.exit(f"ERROR: {err}")
