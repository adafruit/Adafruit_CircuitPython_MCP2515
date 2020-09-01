# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_mcp2515`
================================================================================

A CircuitPython library for working with the MCP2515 CAN bus controller


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""


from struct import unpack_from, pack_into  # pylint:disable=unused-import
from time import sleep, monotonic, monotonic_ns  # pylint:disable=unused-import
from micropython import const
import adafruit_bus_device.spi_device as spi_device

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MCP2515.git"

# modes
_MODE_NORMAL = const(0x00)
# _MODE_SLEEP = const(0x20)
# _MODE_LOOPBACK = const(0x40)
# _MODE_LISTENONLY = const(0x60)
_MODE_CONFIG = const(0x80)


# commands
_MCP_RESET = const(0xC0)
_MCP_WRITE = const(0x02)
_MCP_READ = const(0x03)
_MCP_BITMOD = const(0x05)
# _MCP_LOAD_TX0 = const(0x40)
# _MCP_LOAD_TX1 = const(0x42)
# _MCP_LOAD_TX2 = const(0x44)

# _MCP_RTS_TX0 = const(0x81)
# _MCP_RTS_TX1 = const(0x82)
# _MCP_RTS_TX2 = const(0x84)
# _MCP_RTS_ALL = const(0x87)
# _MCP_READ_RX0 = const(0x90)
# _MCP_READ_RX1 = const(0x94)
_MCP_READ_STATUS = const(0xA0)
# _MCP_RX_STATUS = const(0xB0)
# _MCP_RESET = const(0xC0)

# Registers

_MCP_CANINTE = const(0x2B)
_MCP_CANINTF = const(0x2C)
_MCP_CANSTAT = const(0x0E)
_MCP_CANCTRL = const(0x0F)

_MCP_CNF3 = const(0x28)
_MCP_CNF2 = const(0x29)
_MCP_CNF1 = const(0x2A)


_MCP_TXB0CTRL = const(0x30)
_MCP_TXB1CTRL = const(0x40)
_MCP_TXB2CTRL = const(0x50)

_MCP_RXB0CTRL = const(0x60)
_MCP_RXB1CTRL = const(0x70)


# bitz
_MCP_RX0IF = const(0x01)
_MCP_RX1IF = const(0x02)
# _MCP_TX0IF = const(0x04)
# _MCP_TX1IF = const(0x08)
# _MCP_TX2IF = const(0x10)
# _MCP_ERRIF = const(0x20)
_MCP_WAKIF = const(0x40)
# _MCP_MERRF = const(0x80)


# CANINTF Register Bits


# masks
_MODE_MASK = const(0xE0)

_MCP_RXB_RX_MASK = const(0x60)
_MCP_RXB_BUKT_MASK = const((1 << 2))
_MCP_RXB_RX_STDEXT = const(0x00)


class MCP2515:
    """Library for the MCP2515 CANbus controller

    Args:
        spi_bus ([busio.SPI]): The SPI bus to use to communicate with the MCP2515
        cs_pin ([digitalio.DigitalInOut]): The pin object to use for the SPI Chip Select
        debug (bool, optional): Enables print statements used for debugging. Defaults to False.
    """

    def __init__(self, spi_bus, cs_pin, debug=False):
        self._debug = debug
        self.bus_device_obj = spi_device.SPIDevice(spi_bus, cs_pin)
        self._buffer = bytearray(255)

        self.initialize()

    def initialize(self):
        """Return the sensor to the default configuration"""

        self._reset()
        # our mode set skips checking for sleep
        res = self._set_mode(_MODE_CONFIG)
        # SET MODE (FAILABLE)

        self._set_baud_rate()

        # ************ init canbuffers ***********
        # void MCP_CAN::mcp2515_initCANBuffers(void) {
        #     byte i, a1, a2, a3;

        #     a1 = MCP_TXB0CTRL;
        #     a2 = MCP_TXB1CTRL;
        #     a3 = MCP_TXB2CTRL;
        for idx in range(14):
            self._set_register(_MCP_TXB0CTRL + idx, 0)
            self._set_register(_MCP_TXB1CTRL + idx, 0)
            self._set_register(_MCP_TXB2CTRL + idx, 0)

        # above seems to be zeroing from
        # 0x30-0x3E
        # 0x40-0x4E
        # 0x50-0x5E
        # Pretty sure we can do them all at once, but not just yet.

        self._set_register(_MCP_RXB0CTRL, 0)
        self._set_register(_MCP_RXB1CTRL, 0)

        # # # interrupt mode

        self._set_register(_MCP_CANINTE, _MCP_RX0IF | _MCP_RX1IF)
        sleep(0.010)
        self._mod_register(
            _MCP_RXB0CTRL,
            _MCP_RXB_RX_MASK | _MCP_RXB_BUKT_MASK,
            _MCP_RXB_RX_STDEXT | _MCP_RXB_BUKT_MASK,
        )

        self._mod_register(_MCP_RXB1CTRL, _MCP_RXB_RX_MASK, _MCP_RXB_RX_STDEXT)

        self._set_mode(_MODE_NORMAL)
        return res

    def _set_baud_rate(self):

        # *******8 set baud rate ***********
        # if (mcp2515_configRate(canSpeed, clock)) {
        # mcp2515_setRegister(MCP_CNF1, 0x00)
        # mcp2515_setRegister(MCP_CNF2, 0xF0)
        # mcp2515_setRegister(MCP_CNF3, 0x86)

        self._set_register(_MCP_CNF1, 0x00)
        self._set_register(_MCP_CNF2, 0xF0)
        self._set_register(_MCP_CNF3, 0x86)
        sleep(0.010)

    def _reset(self):
        self._buffer[0] = _MCP_RESET
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=1)
        sleep(0.010)

    def _set_mode(self, mode):
        stat_reg = self._read_register(_MCP_CANSTAT)
        current_mode = stat_reg & _MODE_MASK

        if current_mode == mode:
            print("mode already set")
            return True
        return self._request_new_mode(mode)

    def _request_new_mode(self, mode):
        start_time_ns = monotonic_ns()

        while True:
            # Request new mode
            # This is inside the loop as sometimes requesting the new mode once doesn't work
            # (usually when attempting to sleep)
            self._mod_register(_MCP_CANCTRL, _MODE_MASK, mode)

            status = self._read_register(_MCP_CANSTAT)
            if (status & _MODE_MASK) == mode:
                return True

            # timeout
            if ((monotonic_ns() - start_time_ns) / 1000000) > 200:
                raise RuntimeError("Timeout setting Mode")

    def _mod_register(self, register_addr, mask, new_value):
        """There appears to be an interface on the MCP2515 that allows for
        setting a register using a mask"""
        self._buffer[0] = _MCP_BITMOD
        self._buffer[1] = register_addr
        self._buffer[2] = mask
        self._buffer[3] = new_value
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=4)

    # def _disable_wake(self):
    #     self._mod_register(_MCP_CANINTF, _MCP_WAKIF, 0)

    def _read_register(self, regsiter_addr):
        self._buffer[0] = _MCP_READ
        self._buffer[1] = regsiter_addr

        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=2)
            self._buffer[0] = 0
            spi.write_readinto(
                self._buffer, self._buffer, out_start=0, out_end=1, in_start=0, in_end=1
            )

        return self._buffer[0]

    def _read_status(self):
        self._buffer[0] = _MCP_READ_STATUS
        with self.bus_device_obj as spi:
            spi.write_readinto(
                self._buffer, self._buffer, out_start=0, out_end=1, in_start=0, in_end=1
            )
        return self._buffer[0]

    def _set_register(self, regsiter_addr, register_value):
        self._buffer[0] = _MCP_WRITE
        self._buffer[1] = regsiter_addr
        self._buffer[2] = register_value
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=3)
