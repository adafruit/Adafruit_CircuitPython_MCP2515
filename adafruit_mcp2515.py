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
from collections import namedtuple
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

_LOAD_TX0 = const(0x40)
_LOAD_TX1 = const(0x42)
_LOAD_TX2 = const(0x44)
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

_TX0IF = const(0x04)
_TX1IF = const(0x08)
_TX2IF = const(0x10)

# bits/flags
_RX0IF = const(0x01)
_RX1IF = const(0x02)
# _TX0IF = const(0x04)
# _TX1IF = const(0x08)
# _TX2IF = const(0x10)
# _ERRIF = const(0x20)
_WAKIF = const(0x40)
# _MERRF = const(0x80)

_TXB_EXIDE_M = const(0x08)

# CANINTF Register Bits

# masks
_MODE_MASK = const(0xE0)

_RXB_RX_MASK = const(0x60)
_RXB_BUKT_MASK = const((1 << 2))
_RXB_RX_STDEXT = const(0x00)

_STAT_TX0_PENDING = const(0x04)
_STAT_TX1_PENDING = const(0x10)
_STAT_TX2_PENDING = const(0x40)

_STAT_TX_PENDING_MASK = const(_STAT_TX0_PENDING | _STAT_TX1_PENDING
                              | _STAT_TX2_PENDING)


_EXTENDED_ID_MASK = ((1<<18)-1) # bottom 18 bits
_SEND_TIMEOUT_MS = const(500)  # 500ms

# perhaps this will be stateful later?
TransmitBuffer = namedtuple("TransmitBuffer",
                            ['CTRL_REG', 'STD_ID_REG', 'INT_FLAG_MASK', 'LOAD_CMD'])


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
        self._id_buffer = bytearray(4)

        self._tx_buffers = []
        self._init_buffers()
        self.initialize()

    def _init_buffers(self):

        self._tx_buffers = [
            TransmitBuffer(CTRL_REG=_TXB0CTRL,
                           STD_ID_REG=_TXB0SIDH,
                           INT_FLAG_MASK=_TX0IF,
                           LOAD_CMD=_LOAD_TX0),
            TransmitBuffer(CTRL_REG=_TXB1CTRL,
                           STD_ID_REG=_TXB1SIDH,
                           INT_FLAG_MASK=_TX1IF,
                           LOAD_CMD=_LOAD_TX1),
            TransmitBuffer(CTRL_REG=_TXB2CTRL,
                           STD_ID_REG=_TXB2SIDH,
                           INT_FLAG_MASK=_TX2IF,
                           LOAD_CMD=_LOAD_TX2)
        ]

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
            _RXB0CTRL,
            _RXB_RX_MASK | _RXB_BUKT_MASK,
            _RXB_RX_STDEXT | _RXB_BUKT_MASK,
        )

        self._mod_register(_RXB1CTRL, _RXB_RX_MASK, _RXB_RX_STDEXT)

        self._set_mode(_MODE_NORMAL)
        return res

    def send_buffer(self,
                    message_buffer,
                    tx_id=0x00,
                    ext=None,
                    rtr=False,
                    wait_sent=True):
        """send a message buffer"""
        # send example call:
        # CAN.sendMsgBuf(tx_id=0x00, ext=0, rtrBit=0, len=8, buf=stmp, wait_sent=true)

        # TODO: Timeout
        tx_buff = self._get_tx_buffer()  # info = addr.

        write_canMsg(tx_buffer_addr, tx_id, ext, rtrBit, len, byte* buf)
        if not wait_sent:
            return True
        sleep(0.010)

        send_confirmed = False
        timed_out = False
        start = monotonic_ns()
        while not send_confirmed:
            if _has_elapsed(start, _SEND_TIMEOUT_MS):
                raise RuntimeError(
                    "Timeout occoured waiting for transmit confirmation")
            # the status register address is whatever tx_buff_n is, minus one?
            tx_buff_status = self._read_register(tx_buff.CTRL_REG)
            print("Status of chosen buffer:", tx_buff_status)
            # tx_buffer_status = mcp2515_readRegister(txbuf_n - 1)  # read send buff ctrl reg
            # check for 0x08/0b00001000 being un-set
            # TODO: double check this polarity
            send_confirmed = (tx_buff_status & 0x08) == 0

        return True

    def _write_message(self, tx_buffer, send_id, ext, rtr, messeage_buffer):

        load_command = tx_buffer.LOAD_CMD

        # add the RTR_MASK bits to len to get DLC, if rtr
        dlc = len(message_buffer)
        if rtr:
            dlc |= _RTR_MASK

        # get id buffer segment

        # mcp2515_id_to_buf(ext, id, self._id_buffer)
        self._load_id_buffer(ext, id, self._id_buffer)

        # this splits up the id header, dlc (len, rtr status), and message buffer
        # TODO: check if we can send in one buffer, in which case `id_buffer` isn't needed

        spi_readwrite(load_command)
        for (i = 0 i < 4 i++)
            spi_write(tbufdata[i])
        }
        spi_write(dlc)
        for (i = 0 i < len && i < CAN_MAX_CHAR_IN_MESSAGE i++)
            spi_write(buf[i])
        }

        mcp2515_start_transmit(buffer_sidh_addr)

    def _load_id_buffer(self, send_id, extended_id=False):

        if extended_id:
            # set extended id
            # pack the bottom 18 bits(extended id) into the first four bytes of
            # the buffer:
            # 0: n/a 1: xxxxxxEXID[17:16] 2:EXID[15:8] 3: EXID[7:0]
            extended_id = send_id & _EXTENDED_ID_MASK # bottom 18 bits
            pack_into(">I", self._buffer, 0, extended_id)
            # buff[3] = (byte)(canid & 0xFF) # EID[7:0]
            # buff[2] = (byte)(canid >> 8) # EID[15:8]
            # buff[1] = (byte)(canid & 0x03)  # EID[17:16]


            # need to set top of buffer[1], so get the current value
            extid_msbits = self._buffer[1]
            # get the next 2 bytes of the given ID
            canid = 0xFFFF & (send_id >> 16) # MSBytes

            # standard ID is the remaining 6 bits
            std_id = (canid & 0xFC) >> 2
            std_id_too = (send_id & 0x00FC0000) >> 16
            print("std_id", std_id, "stid_too_", std_id_too)
            ms_bytes =  ( std_id<<5 | _TXB_EXIDE_M | extid_msbits)
            pack_into(">H", self._buffer, 0, ms_bytes)

            # buff[0] = (byte)(canid >> 5) # top 3 bits of STDID
            # final buff: 0x6F5 0x69 0xCA 0xFE
        else:
            std_id = (canid & 0xFC) >> 2
            pack_into(">H", self._buffer, 0, std_id)

            # buff[0] = (byte)(canid >> 3) # top 5 bits to idx 0
            # buff[1] = (byte)((canid & 0x07) << 5) # bottom 3 bits to top of 1
            # buff[3] = 0
            # buff[2] = 0


    @property
    def _tx_buffers_in_use(self):
        # the ref code allows for reserving buffers, but didn't see any way
        # to use them. maybe un-reserve then use?
        # TODO: this should return a tuple of busy states
        # byte status = mcp2515_readStatus() & MCP_STAT_TX_PENDING_MASK
        status = self._read_status()
        print("Status byte:", "{:#010b}".format(status))
        return (bool(status & _STAT_TX0_PENDING),
                bool(status & _STAT_TX1_PENDING),
                bool(status & _STAT_TX2_PENDING))

    #
    def _get_tx_buffer(self):
        """Get the address of the next available tx buffer and unset
        its interrupt bit in _CANINTF"""
        # check all buffers by looking for match on
        txs_busy = self._tx_buffers_in_use
        if all(txs_busy):
            print("none available!")
            return None
        buffer_index = txs_busy.index(False)  # => self._tx_buffers
        print("Available buffer at index", buffer_index)
        tx_buffer = self._tx_buffers[buffer_index]

        print("Available buffer:", tx_buffer)

        self._mod_register(_CANINTF, tx_buffer.INT_FLAG_MASK, 0)
        return tx_buffer

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
            spi.write_readinto(self._buffer,
                               self._buffer,
                               out_start=0,
                               out_end=1,
                               in_start=0,
                               in_end=1)

        return self._buffer[0]

    def _read_status(self):
        self._buffer[0] = _READ_STATUS
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=1)
            spi.readinto(self._buffer, start=0, end=1)
        return self._buffer[0]

    def _set_register(self, regsiter_addr, register_value):
        self._buffer[0] = _WRITE
        self._buffer[1] = regsiter_addr
        self._buffer[2] = register_value
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=3)
