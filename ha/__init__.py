"""The SolaX Modbus Integration."""
import logging
import threading
from datetime import datetime, timedelta
from typing import Optional
import importlib.util, sys
import importlib

_LOGGER = logging.getLogger(__name__)

from pymodbus.client import ModbusSerialClient

UNIT_OR_SLAVE = 'slave'
_LOGGER.debug("using pymodbus library 3.x")

from pymodbus.constants import Endian
from pymodbus.exceptions import ConnectionException
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder, Endian

from .const import (
    DEFAULT_NAME,
    DEFAULT_SCAN_INTERVAL,
    DOMAIN,
    CONF_MODBUS_ADDR,
    CONF_INTERFACE,
    CONF_SERIAL_PORT,
    CONF_READ_EPS,
    CONF_READ_DCB,
    CONF_BAUDRATE,
    CONF_PLUGIN,
    DEFAULT_READ_EPS,
    DEFAULT_READ_DCB,
    DEFAULT_INTERFACE,
    DEFAULT_SERIAL_PORT,
    DEFAULT_MODBUS_ADDR,
    DEFAULT_PORT,
    DEFAULT_BAUDRATE,
    DEFAULT_PLUGIN,
    PLUGIN_PATH,
    SLEEPMODE_LASTAWAKE
)
from .const import REGISTER_S32, REGISTER_U32, REGISTER_U16, REGISTER_S16, REGISTER_ULSB16MSB16, REGISTER_STR, \
    REGISTER_WORDS, REGISTER_U8H, REGISTER_U8L
from .const import setPlugin, getPlugin, getPluginName

PLATFORMS = ["button", "number", "select", "sensor"]


# seriesnumber = 'unknown'

async def async_setup_entry():
    """Set up a SolaX mobus."""
    name = "test_name" # config[CONF_NAME]

    # ================== dynamically load desired plugin
    _LOGGER.debug(f"Ready to load plugin")
    plugin = importlib.import_module(f".plugin_solax", 'ha')
    if not plugin: _LOGGER.error(f"could not import plugin")
    setPlugin(name, plugin)
    # ====================== end of dynamic load

    hub = SolaXModbusHub(name)
    """Register the hub."""
    return True

def defaultIsAwake(datadict):
    return True

def Gen4Timestring(numb):
    h = numb % 256
    m = numb >> 8
    return f"{h:02d}:{m:02d}"


class SolaXModbusHub:
    """Thread safe wrapper class for pymodbus."""

    def __init__(
            self,
            name
    ):
        """Initialize the Modbus hub."""
        _LOGGER.info(f"solax modbushub creation with interface serial baudrate (only for serial): 9600")
#        self._client = ModbusSerialClient(method="rtu", port="COM7", baudrate=9600, parity='N', stopbits=1,
        self._client = ModbusSerialClient(method="rtu", port="/dev/cu.usbserial-14210", baudrate=9600, parity='N', stopbits=1,
                                              bytesize=8, timeout=3)
        self._lock = threading.Lock()
        self._name = name
        self._modbus_addr = 1
        self._seriesnumber = 'still unknown'
        self._scan_interval = timedelta(seconds=5)
        self._unsub_interval_method = None
        self._sensors = []
        self.data = {}
        self.cyclecount = 0  # temporary - remove later
        self.slowdown = 1  # slow down factor when modbus is not responding: 1 : no slowdown, 10: ignore 9 out of 10 cycles
        self.inputBlocks = {}
        self.holdingBlocks = {}
        self.computedRegs = {}
        self.sleepzero = []  # sensors that will be set to zero in sleepmode
        self.sleepnone = []  # sensors that will be cleared in sleepmode
        self.writequeue = {}  # queue requests when inverter is in sleep mode
        _LOGGER.debug(f"{self.name}: ready to call plugin to determine inverter type")
        self.plugin = getPlugin(name).plugin_instance
        self.awake_button = None
        self._invertertype = self.plugin.determineInverterType(self)
        _LOGGER.setLevel(logging.DEBUG)
        _LOGGER.info("solax modbushub done %s", self.__dict__)



    def async_refresh_modbus_data(self, _now: Optional[int] = None) -> None:
        """Time to update."""
        self.cyclecount = self.cyclecount + 1
        if not self._sensors:
            return
        if (self.cyclecount % self.slowdown) == 0:  # only execute once every slowdown count
            update_result = self.read_modbus_data()
            if update_result:
                self.slowdown = 1  # return to full polling after succesfull cycle
                for update_callback in self._sensors:
                    update_callback()
            else:
                _LOGGER.debug(f"assuming sleep mode - slowing down by factor 10")
                self.slowdown = 10
                for i in self.sleepnone: self.data.pop(i, None)
                for i in self.sleepzero: self.data[i] = 0
                # self.data = {} # invalidate data - do we want this ??

    @property
    def invertertype(self):
        return self._invertertype

    @invertertype.setter
    def invertertype(self, newtype):
        self._invertertype = newtype

    @property
    def seriesnumber(self):
        return self._seriesnumber

    @seriesnumber.setter
    def seriesnumber(self, nr):
        self._seriesnumber = nr

    @property
    def name(self):
        """Return the name of this hub."""
        return self._name

    def close(self):
        """Disconnect client."""
        with self._lock:
            self._client.close()

    def connect(self):
        """Connect client."""
        with self._lock:
            self._client.connect()

    def read_holding_registers(self, unit, address, count):
        """Read holding registers."""
        with self._lock:
            kwargs = {UNIT_OR_SLAVE: unit} if unit else {}
            return self._client.read_holding_registers(address, count, **kwargs)

    def read_input_registers(self, unit, address, count):
        """Read input registers."""
        # unit -> modbus address
        with self._lock:
            kwargs = {UNIT_OR_SLAVE: unit} if unit else {}
            _LOGGER.debug(f"read_input_register Unit: {unit}, Address: {address}, Count:{count}")
            return self._client.read_input_registers(address, count, **kwargs)

    def _lowlevel_write_register(self, unit, address, payload):
        with self._lock:
            kwargs = {UNIT_OR_SLAVE: unit} if unit else {}
            # builder = BinaryPayloadBuilder(byteorder=Endian.Big, wordorder=Endian.Big)
            builder = BinaryPayloadBuilder(byteorder=self.plugin.order16, wordorder=self.plugin.order32)
            builder.reset()
            builder.add_16bit_int(payload)
            payload = builder.to_registers()
            return self._client.write_register(address, payload[0], **kwargs)

    def write_register(self, unit, address, payload):
        """Write register."""
        # awake = self.awakeplugin(self.data)
        awake = self.plugin.isAwake(self.data)
        if awake:
            return self._lowlevel_write_register(unit, address, payload)
        else:
            # put request in queue
            self.writequeue[address] = payload
            # awaken inverter
            if self.awake_button:
                _LOGGER.info("waking up inverter: pressing awake button")
                return self._lowlevel_write_register(unit=self._modbus_addr, address=self.awake_button.register,
                                                     payload=self.awake_button.command)
            else:
                _LOGGER.warning("cannot wakeup inverter: no awake button found")

    def write_registers_single(self, unit, address, payload):  # Needs adapting for regiater que
        """Write registers."""
        with self._lock:
            kwargs = {"unit": unit} if unit else {}
            builder = BinaryPayloadBuilder(byteorder=self.plugin.order16, wordorder=self.plugin.order32)
            builder.reset()
            builder.add_16bit_int(payload)
            payload = builder.to_registers()
            return self._client.write_register(address, payload, **kwargs)

    def read_modbus_data(self):
        res = True
        try:
            res = self.read_modbus_registers_all()
        except ConnectionException as ex:
            _LOGGER.error("Reading data failed! Inverter is offline.")
            res = False
        except Exception as ex:
            _LOGGER.exception("Something went wrong reading from modbus")
            res = False
        return res

    def treat_address(self, decoder, descr, initval=0):
        return_value = None
        if self.cyclecount < 5: _LOGGER.debug(f"treating register 0x{descr.register:02x} : {descr.key}")
        try:
            if descr.unit == REGISTER_U16:
                val = decoder.decode_16bit_uint()
            elif descr.unit == REGISTER_S16:
                val = decoder.decode_16bit_int()
            elif descr.unit == REGISTER_U32:
                val = decoder.decode_32bit_uint()
            elif descr.unit == REGISTER_S32:
                val = decoder.decode_32bit_int()
            elif descr.unit == REGISTER_STR:
                val = str(decoder.decode_string(descr.wordcount * 2).decode("ascii"))
            elif descr.unit == REGISTER_WORDS:
                val = [decoder.decode_16bit_uint() for val in range(descr.wordcount)]
            elif descr.unit == REGISTER_ULSB16MSB16:
                val = decoder.decode_16bit_uint() + decoder.decode_16bit_uint() * 256 * 256
            elif descr.unit == REGISTER_U8L:
                val = initval % 256
            elif descr.unit == REGISTER_U8H:
                val = initval >> 8
            else:
                _LOGGER.warning(f"undefinded unit for entity {descr.key} - setting value to zero")
                val = 0
        except Exception as ex:
            if self.cyclecount < 5:
                _LOGGER.warning(f"{self.name}: read failed at 0x{descr.register:02x}: {descr.key}", exc_info=True)
            else:
                _LOGGER.warning(f"{self.name}: read failed at 0x{descr.register:02x}: {descr.key} ")
            val = 0
        if type(descr.scale) is dict:  # translate int to string
            return_value = descr.scale.get(val, "Unknown")
        elif callable(descr.scale):  # function to call ?
            return_value = descr.scale(val, descr, self.data)
        else:  # apply simple numeric scaling and rounding if not a list of words
            try:
                return_value = round(val * descr.scale, descr.rounding)
            except:
                return_value = val  # probably a REGISTER_WORDS instance
        # if (descr.sleepmode != SLEEPMODE_LASTAWAKE) or self.awakeplugin(self.data): self.data[descr.key] = return_value
        if (descr.sleepmode != SLEEPMODE_LASTAWAKE) or self.plugin.isAwake(self.data): self.data[
            descr.key] = return_value
        _LOGGER.debug(f"treating register 0x{descr.register:02x} : {descr.key} with result:{return_value}")

    def read_modbus_block(self, block, typ):
        if self.cyclecount < 5:
            _LOGGER.debug(
                f"{self.name} modbus {typ} block start: 0x{block.start:x} end: 0x{block.end:x}  len: {block.end - block.start} \nregs: {block.regs}")
        try:
            if typ == 'input':
                realtime_data = self.read_input_registers(unit=self._modbus_addr, address=block.start,
                                                          count=block.end - block.start)
            else:
                realtime_data = self.read_holding_registers(unit=self._modbus_addr, address=block.start,
                                                            count=block.end - block.start)
        except Exception as ex:
            if self.slowdown == 1: _LOGGER.error(
                f"{str(ex)}: {self.name} cannot read {typ} registers at device {self._modbus_addr} position 0x{block.start:x}",
                exc_info=True)
            return False
        if realtime_data.isError():
            if self.slowdown == 1: _LOGGER.error(
                f"{self.name} error reading {typ} registers at device {self._modbus_addr} position 0x{block.start:x}",
                exc_info=True)
            return False
        # decoder = BinaryPayloadDecoder.fromRegisters(realtime_data.registers, block.order16, wordorder=block.order32)
        decoder = BinaryPayloadDecoder.fromRegisters(realtime_data.registers, self.plugin.order16,
                                                     wordorder=self.plugin.order32)
        prevreg = block.start
        for reg in block.regs:
            if (reg - prevreg) > 0:
                decoder.skip_bytes((reg - prevreg) * 2)
                if self.cyclecount < 5: _LOGGER.debug(f"skipping bytes {(reg - prevreg) * 2}")
            descr = block.descriptions[reg]
            if type(descr) is dict:  # set of byte values
                val = decoder.decode_16bit_uint()
                for k in descr: self.treat_address(decoder, descr[k], val)
                prevreg = reg + 1
            else:  # single value
                self.treat_address(decoder, descr)
                if descr.unit in (REGISTER_S32, REGISTER_U32, REGISTER_ULSB16MSB16,):
                    prevreg = reg + 2
                elif descr.unit in (REGISTER_STR, REGISTER_WORDS,):
                    prevreg = reg + descr.wordcount
                else:
                    prevreg = reg + 1
        return True

    def read_modbus_registers_all(self):
        res = True
        for block in self.holdingBlocks:
            res = res and self.read_modbus_block(block, 'holding')
        for block in self.inputBlocks:
            res = res and self.read_modbus_block(block, 'input')
        for reg in self.computedRegs:
            descr = self.computedRegs[reg]
            self.data[descr.key] = descr.value_function(0, descr, self.data)
        if res and self.writequeue and self.plugin.isAwake(self.data):  # self.awakeplugin(self.data):
            # process outstanding write requests
            _LOGGER.info(f"inverter is now awake, processing outstanding write requests {self.writequeue}")
            for addr in self.writequeue:
                val = self.writequeue.pop(addr)
                self.write_register(self.modbus_addr, addr, val)
            self.writequeue = {}  # not really needed
        return res

