import importlib
import logging
from dataclasses import dataclass

from pymodbus import pymodbus_apply_logging_config
from pymodbus.payload import Endian

import serial
import os
# --------------------------------------------------------------------------- #
# import the various client implementations
# --------------------------------------------------------------------------- #
from pymodbus.client import (
    ModbusSerialClient
)
from pymodbus.exceptions import ModbusException
from pymodbus.payload import BinaryPayloadDecoder

from ha import SolaXModbusHub, setPlugin
from ha.const import BaseModbusSensorEntityDescription, REG_HOLDING, REGISTER_U8H, REGISTER_U8L, SLEEPMODE_NONE, \
    SLEEPMODE_ZERO, REG_INPUT, REGISTER_STR, REGISTER_WORDS, REGISTER_S32, REGISTER_U32, REGISTER_ULSB16MSB16

_logger = logging.getLogger()
_logger.setLevel(logging.DEBUG)

def unsigned16(result, addr):
    return result.getRegister(addr)

def join_msb_lsb(msb, lsb):
    return (msb << 16) | lsb

def setup_sync_client():
    """Run client setup."""
    _logger.info("### Create client object")
    client = ModbusSerialClient(method="rtu", port="COM7", baudrate=9600, bytesize=8, parity="N", stopbits=1, timeout=3)
    return client


def run_sync_client(client):
    _logger.info("### Client starting")
    pymodbus_apply_logging_config()
    _logger.setLevel(logging.DEBUG)

    client.connect()

    # result = client.read_input_registers(slave=1, address=0x03)
    # if isinstance(result, ModbusException):
    #     print("Exception from SolaxX3RS485: {}".format(result))
    #     return

    result = client.read_holding_registers(slave=1, address=0x300, count=7)
    if isinstance(result, ModbusException):
        print("Exception from SolaxX3RS485: {}".format(result))
        return
    decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.Big)
    seriesnumber = decoder.decode_string(14).decode("ascii")
    ba = bytearray(seriesnumber, "ascii")  # convert to bytearray for swapping
    ba[0::2], ba[1::2] = ba[1::2], ba[0::2]  # swap bytes ourselves - due to bug in Endian.Little ?
    res = str(ba, "ascii")  # convert back to string

    _logger.info(res)

    # self.vals['Pv1 input voltage'] = unsigned16(result, 0) / 10
    # self.vals['Pv2 input voltage'] = unsigned16(result, 1) / 10
    # self.vals['Pv1 input current'] = unsigned16(result, 2) / 10
    # self.vals['Pv2 input current'] = unsigned16(result, 3) / 10
    # self.vals['Grid Voltage Phase 1'] = unsigned16(result, 4) / 10
    # self.vals['Grid Voltage Phase 2'] = unsigned16(result, 5) / 10
    # self.vals['Grid Voltage Phase 3'] = unsigned16(result, 6) / 10
    # self.vals['Grid Frequency Phase 1'] = unsigned16(result, 7) / 100
    # self.vals['Grid Frequency Phase 2'] = unsigned16(result, 8) / 100
    # self.vals['Grid Frequency Phase 3'] = unsigned16(result, 9) / 100
    # self.vals['Output Current Phase 1'] = unsigned16(result, 10) / 10
    # self.vals['Output Current Phase 2'] = unsigned16(result, 11) / 10
    # self.vals['Output Current Phase 3'] = unsigned16(result, 12) / 10
    # self.vals['Temperature'] = unsigned16(result, 13)
    # self.vals['Inverter Power'] = unsigned16(result, 14)
    # self.vals['RunMode'] = unsigned16(result, 15)
    # self.vals['Output Power Phase 1'] = unsigned16(result, 16)
    # self.vals['Output Power Phase 2'] = unsigned16(result, 17)
    # self.vals['Output Power Phase 3'] = unsigned16(result, 18)
    # self.vals['Total DC Power'] = unsigned16(result, 19)
    # self.vals['PV1 DC Power'] = unsigned16(result, 20)
    # self.vals['PV2 DC Power'] = unsigned16(result, 21)
    # self.vals['Fault value of Phase 1 Voltage'] = unsigned16(result, 22) / 10
    # self.vals['Fault value of Phase 2 Voltage'] = unsigned16(result, 23) / 10
    # self.vals['Fault value of Phase 3 Voltage'] = unsigned16(result, 24) / 10
    # self.vals['Fault value of Phase 1 Frequency'] = unsigned16(result, 25) / 100
    # self.vals['Fault value of Phase 2 Frequency'] = unsigned16(result, 26) / 100
    # self.vals['Fault value of Phase 3 Frequency'] = unsigned16(result, 27) / 100
    # self.vals['Fault value of Phase 1 DCI'] = unsigned16(result, 28) / 1000
    # self.vals['Fault value of Phase 2 DCI'] = unsigned16(result, 29) / 1000
    # self.vals['Fault value of Phase 3 DCI'] = unsigned16(result, 30) / 1000
    # self.vals['Fault value of PV1 Voltage'] = unsigned16(result, 31) / 10
    # self.vals['Fault value of PV2 Voltage'] = unsigned16(result, 32) / 10
    # self.vals['Fault value of Temperature'] = unsigned16(result, 33)
    # self.vals['Fault value of GFCI'] = unsigned16(result, 34) / 1000
    # self.vals['Total Yield'] = join_msb_lsb(unsigned16(result, 36), unsigned16(result, 35)) / 1000
    # self.vals['Yield Today'] = join_msb_lsb(unsigned16(result, 38), unsigned16(result, 37)) / 1000

    client.close()
    _logger.info("### End of Program")

INVALID_START = 99999

@dataclass
class block():
    start: int = None  # start address of the block
    end: int = None  # end address of the block
    # order16: int = None # byte endian for 16bit registers
    # order32: int = None # word endian for 32bit registers
    descriptions: None = None
    regs: None = None  # sorted list of registers used in this block

def splitInBlocks(descriptions, block_size):
    start = INVALID_START
    end = 0
    blocks = []
    curblockregs = []
    for reg in descriptions:
        descr = descriptions[reg]
        if (not type(descr) is dict) and (descr.newblock or ((reg - start) > block_size)):
            if ((end - start) > 0):
                _logger.info(f"Starting new block at 0x{reg:x} ")
                # newblock = block(start = start, end = end, order16 = descriptions[start].order16, order32 = descriptions[start].order32, descriptions = descriptions, regs = curblockregs)
                newblock = block(start=start, end=end, descriptions=descriptions, regs=curblockregs)
                blocks.append(newblock)
                start = INVALID_START
                end = 0
                curblockregs = []
            else:
                _logger.info(f"newblock declaration found for empty block")

        if start == INVALID_START: start = reg
        if type(descr) is dict:
            end = reg + 1  # couple of byte values
        else:
            _logger.info(f"adding register 0x{reg:x} {descr.key} to block with start 0x{start:x}")
            if descr.unit in (REGISTER_STR, REGISTER_WORDS,):
                if (descr.wordcount):
                    end = reg + descr.wordcount
                else:
                    _logger.warning(f"invalid or missing missing wordcount for {descr.key}")
            elif descr.unit in (REGISTER_S32, REGISTER_U32, REGISTER_ULSB16MSB16,):
                end = reg + 2
            else:
                end = reg + 1
        curblockregs.append(reg)
    if ((end - start) > 0):  # close last block
        # newblock = block(start = start, end = end, order16 = descriptions[start].order16, order32 = descriptions[start].order32, descriptions = descriptions, regs = curblockregs)
        newblock = block(start=start, end=end, descriptions=descriptions, regs=curblockregs)
        blocks.append(newblock)
    return blocks

def setup_entry(hub): #, async_add_entities):
    # if entry.data:
    #     hub_name = entry.data[CONF_NAME]  # old style - remove soon
    # else:
    #     hub_name = entry.options[CONF_NAME]  # new format
    # hub = hass.data[DOMAIN][hub_name]["hub"]

    # device_info = {
    #     "identifiers": {(DOMAIN, hub_name)},
    #     "name": hub_name,
    #     "manufacturer": ATTR_MANUFACTURER,
    # }

    entities = []
    holdingRegs = {}
    inputRegs = {}
    computedRegs = {}
    # holdingOrder16 = {} # all entities should have the same order
    # inputOrder16   = {} # all entities should have the same order
    # holdingOrder32 = {} # all entities should have the same order
    # inputOrder32   = {} # all entities should have the same order

    plugin = hub.plugin  # getPlugin(hub_name)
    for sensor_description in plugin.SENSOR_TYPES:
        if plugin.matchInverterWithMask(hub._invertertype, sensor_description.allowedtypes, hub.seriesnumber,
                                        sensor_description.blacklist):
            # apply scale exceptions early
            readscale = None
            # normal_scale = not ((type(sensor_description.scale) is dict) or callable(sensor_description.scale))
            # if normal_scale and sensor_description.read_scale_exceptions:
            if sensor_description.read_scale_exceptions:
                for (prefix, value,) in sensor_description.read_scale_exceptions:
                    if hub.seriesnumber.startswith(prefix): readscale = value
            sensor = SolaXModbusSensor(
                None,
                hub,
                None, #device_info,
                sensor_description,
                read_scale=readscale,
            )
            entities.append(sensor)
            if sensor_description.sleepmode == SLEEPMODE_NONE: hub.sleepnone.append(sensor_description.key)
            if sensor_description.sleepmode == SLEEPMODE_ZERO: hub.sleepzero.append(sensor_description.key)
            if (sensor_description.register < 0):  # entity without modbus address
                if sensor_description.value_function:
                    computedRegs[sensor_description.key] = sensor_description
                else:
                    _logger.warning(
                        f"entity without modbus register address and without value_function found: {sensor_description.key}")
            else:
                if sensor_description.register_type == REG_HOLDING:
                    if sensor_description.register in holdingRegs:  # duplicate or 2 bytes in one register ?
                        if sensor_description.unit in (REGISTER_U8H, REGISTER_U8L,) and holdingRegs[
                            sensor_description.register].unit in (REGISTER_U8H, REGISTER_U8L,):
                            first = holdingRegs[sensor_description.register]
                            holdingRegs[sensor_description.register] = {first.unit: first,
                                                                        sensor_description.unit: sensor_description}
                        else:
                            _logger.warning(
                                f"holding register already used: 0x{sensor_description.register:x} {sensor_description.key}")
                    else:
                        holdingRegs[sensor_description.register] = sensor_description
                        # holdingOrder16[sensor_description.order16] = True
                        # holdingOrder32[sensor_description.order32] = True
                elif sensor_description.register_type == REG_INPUT:
                    if sensor_description.register in inputRegs:  # duplicate or 2 bytes in one register ?
                        first = inputRegs[sensor_description.register]
                        inputRegs[sensor_description.register] = {first.unit: first,
                                                                  sensor_description.unit: sensor_description}
                        _logger.warning(
                            f"input register already declared: 0x{sensor_description.register:x} {sensor_description.key}")
                    else:
                        inputRegs[sensor_description.register] = sensor_description
                        # inputOrder16[sensor_description.order16] = True
                        # inputOrder32[sensor_description.order32] = True
                else:
                    _logger.warning(f"entity declaration without register_type found: {sensor_description.key}")
    #async_add_entities(entities)
    # sort the registers for this type of inverter
    holdingRegs = dict(sorted(holdingRegs.items()))
    inputRegs = dict(sorted(inputRegs.items()))
    # check for consistency
    # if (len(inputOrder32)>1) or (len(holdingOrder32)>1): _logger.warning(f"inconsistent Big or Little Endian declaration for 32bit registers")
    # if (len(inputOrder16)>1) or (len(holdingOrder16)>1): _logger.warning(f"inconsistent Big or Little Endian declaration for 16bit registers")
    # split in blocks and store results
    hub.holdingBlocks = splitInBlocks(holdingRegs, hub.plugin.block_size)
    hub.inputBlocks = splitInBlocks(inputRegs, hub.plugin.block_size)
    hub.computedRegs = computedRegs

    for i in hub.holdingBlocks: _logger.info(f"returning holding block: 0x{i.start:x} 0x{i.end:x} {i.regs}")
    for i in hub.inputBlocks: _logger.info(f"returning input block: 0x{i.start:x} 0x{i.end:x} {i.regs}")
    _logger.debug(f"holdingBlocks: {hub.holdingBlocks}")
    _logger.debug(f"inputBlocks: {hub.inputBlocks}")
    _logger.info(f"computedRegs: {hub.computedRegs}")
    return True


class SolaXModbusSensor():# SensorEntity):
    """Representation of an SolaX Modbus sensor."""

    def __init__(
            self,
            platform_name,
            hub,
            device_info,
            description: BaseModbusSensorEntityDescription,
            read_scale=1
    ):
        """Initialize the sensor."""
        self._platform_name = platform_name
        self._attr_device_info = device_info
        self._hub = hub
        self.entity_description: BaseModbusSensorEntityDescription = description
        # self._attr_scale = scale
        self._read_scale = read_scale


if __name__ == "__main__":

    # ================== dynamically load desired plugin
    _logger.debug(f"Ready to load plugin")
    plugin = importlib.import_module(f".plugin_solax", 'ha')
    if not plugin: _logger.error(f"could not import plugin")
    setPlugin("SolaxMIC", plugin)
    hub = SolaXModbusHub("SolaxMIC")
    setup_entry(hub)
    hub.read_modbus_data()
#   testclient = setup_sync_client()
#   run_sync_client(testclient)
