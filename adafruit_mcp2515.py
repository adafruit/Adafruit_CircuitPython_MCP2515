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
_RESET = const(0xC0)
_WRITE = const(0x02)
_READ = const(0x03)
_BITMOD = const(0x05)

_READ_STATUS = const(0xA0)

# Registers

_CANINTE = const(0x2B)
_CANINTF = const(0x2C)
_CANSTAT = const(0x0E)
_CANCTRL = const(0x0F)

_CNF3 = const(0x28)
_CNF2 = const(0x29)
_CNF1 = const(0x2A)


_TXB0CTRL = const(0x30)
_TXB0SIDH = const(0x31)
_TXB1CTRL = const(0x40)
_TXB1SIDH = const(0x41)
_TXB2CTRL = const(0x50)
_TXB2SIDH = const(0x51)
_RXB0CTRL = const(0x60)
_RXB0SIDH = const(0x61)
_RXB1CTRL = const(0x70)
_RXB1SIDH = const(0x71)

_RXB0CTRL = const(0x60)
_RXB1CTRL = const(0x70)

_TX0IF = const(0x04)
_TX1IF = const(0x08)
_TX2IF = const(0x10)

# bitz
_RX0IF = const(0x01)
_RX1IF = const(0x02)
# _TX0IF = const(0x04)
# _TX1IF = const(0x08)
# _TX2IF = const(0x10)
# _ERRIF = const(0x20)
_WAKIF = const(0x40)
# _MERRF = const(0x80)


# CANINTF Register Bits


# masks
_MODE_MASK = const(0xE0)

_RXB_RX_MASK = const(0x60)
_RXB_BUKT_MASK = const((1 << 2))
_RXB_RX_STDEXT = const(0x00)
_STAT_TX_PENDING_MASK = const(0x54)
_STAT_TX0_PENDING = const(0x04)
_STAT_TX1_PENDING = const(0x10)
_STAT_TX2_PENDING = const(0x40)

_SEND_TIMEOUT_MS = const(500)  # 500ms


def _has_elapsed(start, timeout_ms):
    return (monotonic_ns() - start) / 1000000 > timeout_ms


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
        self._tx_buffers = []

    def initialize(self):
        """Return the sensor to the default configuration"""

        self._reset()
        # our mode set skips checking for sleep
        res = self._set_mode(_MODE_CONFIG)
        # SET MODE (FAILABLE)

        self._set_baud_rate()

        # ************ init canbuffers ***********
        # void MCP_CAN::mcp2515_initCANBuffers(void):
        #     byte i, a1, a2, a3

        #     a1 = MCP_TXB0CTRL
        #     a2 = MCP_TXB1CTRL
        #     a3 = MCP_TXB2CTRL
        for idx in range(14):
            self._set_register(_TXB0CTRL + idx, 0)
            self._set_register(_TXB1CTRL + idx, 0)
            self._set_register(_TXB2CTRL + idx, 0)

        # above seems to be zeroing from
        # 0x30-0x3E
        # 0x40-0x4E
        # 0x50-0x5E
        # Pretty sure we can do them all at once, but not just yet.

        self._set_register(_RXB0CTRL, 0)
        self._set_register(_RXB1CTRL, 0)

        # # # interrupt mode

        self._set_register(_CANINTE, _RX0IF | _RX1IF)
        sleep(0.010)
        self._mod_register(
            _RXB0CTRL, _RXB_RX_MASK | _RXB_BUKT_MASK, _RXB_RX_STDEXT | _RXB_BUKT_MASK,
        )

        self._mod_register(_RXB1CTRL, _RXB_RX_MASK, _RXB_RX_STDEXT)

        self._set_mode(_MODE_NORMAL)
        return res

    # pylint:disable=too-many-arguments
    # pylint:disable=unused-argument
    # pylint:disable=unused-variable
    def send_buffer(self, tx_id, message_buffer, ext=None, rtr=False, wait_sent=False):
        """send a message buffer"""
        # send example call:
        # CAN.sendMsgBuf(tx_id=0x00, ext=0, rtrBit=0, len=8, buf=stmp, wait_sent=true)

        # get the next available tx buffer; raise exception if you cna't get one
        # before a timeout hits
        tx_buff = self._get_tx_buffer()  # info = addr.

        # write_canMsg(tx_buffer_addr, tx_id, ext, rtrBit, len, byte* buf)
        if not wait_sent:
            return True
        sleep(0.010)
        # uiTimeOut = 0
        send_confirmed = False
        timed_out = False
        start = monotonic_ns()
        while not send_confirmed:
            if _has_elapsed(start, _SEND_TIMEOUT_MS):
                raise RuntimeError("Timeout occoured waiting for transmit confirmation")
            # the status register address is whatever tx_buff_n is, minus one?

            # tx_buff_status = self._read_register(tx_buff.offset - 1)

            # tx_buffer_status = mcp2515_readRegister(txbuf_n - 1)  # read send buff ctrl reg
            # check for 0x08/0b00001000 being un-set

            # TODO: double check this polarity
            # send_confirmed = (tx_buffer_status & 0x08) == 0

        return True

    # def _write_message(self, tx_buffer, send_id, ext, rtr, messeage_buffer ):
    #     # mcp2515_write_canMsg(txbuf_n, id, ext, rtrBit, len, buf)
    #     # get setter command for the chosen buffer
    #     load_command = txSidhToTxLoad(tx_buffer);

    #     tbufdata[4];
    #     # add the RTR_MASK bits to len to get DLC, if rtr
    #     dlc = len(message_buffer)
    #     if rtr:
    #         dlc |= _RTR_MASK

    #     # get id buffer segment

    #     mcp2515_id_to_buf(ext, id, tbufdata);
    #     id_buffer = self._id_buffer(ext, id, self._id_buffer)

    #     # this splits up the id header, dlc (len, rtr status), and message buffer
    #     # TODO: check if we can send in one buffer, in which case `id_buffer` isn't needed
    #     spi_readwrite(load_command);
    #     for (i = 0; i < 4; i++)
    #         spi_write(tbufdata[i]);
    #     }
    #     spi_write(dlc);
    #     for (i = 0; i < len && i < CAN_MAX_CHAR_IN_MESSAGE; i++)
    #         spi_write(buf[i]);
    #     }

    #     mcp2515_start_transmit(buffer_sidh_addr);

    # def getNextFreeTXBuf(self, txbuf_n):
    #     pass
    # def start_transmit(self, buffer_sidh_addr):
    #     pass

    @property
    def _tx_buffers_available(self):
        # the ref code allows for reserving buffers, but didn't see any way
        # to use them. maybe un-reserve then use?
        # TODO: this should return a tuple of busy states
        # byte status = mcp2515_readStatus() & MCP_STAT_TX_PENDING_MASK;

        return (False, False, False)

    #
    def _get_tx_buffer(self):
        """Get the address of the next available tx buffer and unset
        its interrupt bit in _CANINTF"""
        # check all buffers by looking for match on
        txs_available = self._tx_buffers_available
        if not any(txs_available):
            return False
        available_buffer = txs_available.index(True)  # => self._tx_buffers

        # * txbuf_n = txCtrlReg(i) + 1
        # save the xxSIDH register addr for the available TXBn

        # self._set_tx_buffer_int(available_buffer, 0)
        #   mcp2515_modifyRegister(MCP_CANINTF, txIfFlag(i), 0)
        return True

    def _set_baud_rate(self):

        # *******8 set baud rate ***********
        # if (mcp2515_configRate(canSpeed, clock)):
        # mcp2515_setRegister(MCP_CNF1, 0x00)
        # mcp2515_setRegister(MCP_CNF2, 0xF0)
        # mcp2515_setRegister(MCP_CNF3, 0x86)

        self._set_register(_CNF1, 0x00)
        self._set_register(_CNF2, 0xF0)
        self._set_register(_CNF3, 0x86)
        sleep(0.010)

    def _reset(self):
        self._buffer[0] = _RESET
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=1)
        sleep(0.010)

    def _set_mode(self, mode):
        stat_reg = self._read_register(_CANSTAT)
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
            self._mod_register(_CANCTRL, _MODE_MASK, mode)

            status = self._read_register(_CANSTAT)
            if (status & _MODE_MASK) == mode:
                return True

            # timeout
            if ((monotonic_ns() - start_time_ns) / 1000000) > 200:
                raise RuntimeError("Timeout setting Mode")

    def _mod_register(self, register_addr, mask, new_value):
        """There appears to be an interface on the MCP2515 that allows for
        setting a register using a mask"""
        self._buffer[0] = _BITMOD
        self._buffer[1] = register_addr
        self._buffer[2] = mask
        self._buffer[3] = new_value
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=4)

    # def _disable_wake(self):
    #     self._mod_register(_CANINTF, _WAKIF, 0)

    def _read_register(self, regsiter_addr):
        self._buffer[0] = _READ
        self._buffer[1] = regsiter_addr

        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=2)
            self._buffer[0] = 0
            spi.write_readinto(
                self._buffer, self._buffer, out_start=0, out_end=1, in_start=0, in_end=1
            )

        return self._buffer[0]

    def _read_status(self):
        self._buffer[0] = _READ_STATUS
        with self.bus_device_obj as spi:
            spi.write_readinto(
                self._buffer, self._buffer, out_start=0, out_end=1, in_start=0, in_end=1
            )
        return self._buffer[0]

    def _set_register(self, regsiter_addr, register_value):
        self._buffer[0] = _WRITE
        self._buffer[1] = regsiter_addr
        self._buffer[2] = register_value
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=3)
