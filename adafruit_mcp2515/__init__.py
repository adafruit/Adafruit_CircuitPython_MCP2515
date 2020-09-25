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

from collections import namedtuple
from struct import unpack_from, pack_into  # pylint:disable=unused-import
from time import sleep, monotonic, monotonic_ns  # pylint:disable=unused-import
from micropython import const
import adafruit_bus_device.spi_device as spi_device
from .canio import *

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MCP2515.git"

# modes
_MODE_NORMAL = const(0x00)
_MODE_SLEEP = const(0x20)
_MODE_LOOPBACK = const(0x40)
_MODE_LISTENONLY = const(0x60)
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

_SEND_TX0 = const(0x81)
_SEND_TX1 = const(0x82)
_SEND_TX2 = const(0x84)
_SEND_ALL = const(0x87)

_READ_RX0 = const(0x90)
_READ_RX1 = const(0x94)

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
_TXB_TXREQ_M = const(0x08)  # TX request/completion bit

# CANINTF Register Bits

# masks
_MODE_MASK = const(0xE0)

_RXB_RX_MASK = const(0x60)
_RXB_BUKT_MASK = const((1 << 2))
_RXB_RX_STDEXT = const(0x00)

_STAT_RXIF_MASK = const(0x03)
_RTR_MASK = const(0x40)

_STAT_TXIF_MASK = const(0xA8)
_STAT_TX0_PENDING = const(0x04)
_STAT_TX1_PENDING = const(0x10)
_STAT_TX2_PENDING = const(0x40)

_STAT_TX_PENDING_MASK = const(_STAT_TX0_PENDING | _STAT_TX1_PENDING | _STAT_TX2_PENDING)

_EXTENDED_ID_MASK = (1 << 18) - 1  # bottom 18 bits
_SEND_TIMEOUT_MS = const(5)  # 500ms
_MAX_CAN_MSG_LEN = 8  # ?!
# perhaps this will be stateful later?
TransmitBuffer = namedtuple(
    "TransmitBuffer",
    ["CTRL_REG", "STD_ID_REG", "INT_FLAG_MASK", "LOAD_CMD", "SEND_CMD"],
)

# perhaps this will be stateful later? #TODO : dedup with above
ReceiveBuffer = namedtuple(
    "TransmitBuffer",
    ["CTRL_REG", "STD_ID_REG", "INT_FLAG_MASK", "LOAD_CMD", "SEND_CMD"],
)


_BAUD_RATES = {
    1000000: (0x00, 0xD0, 0x82),
    500000: (0x00, 0xF0, 0x86),
    250000: (0x41, 0xF1, 0x85),
    200000: (0x01, 0xFA, 0x87),
    125000: (0x03, 0xF0, 0x86),
    100000: (0x03, 0xFA, 0x87),
    95000: (0x03, 0xAD, 0x07),
    83300: (0x03, 0xBE, 0x07),
    80000: (0x03, 0xFF, 0x87),
    50000: (0x07, 0xFA, 0x87),
    40000: (0x07, 0xFF, 0x87),
    33000: (0x09, 0xBE, 0x07),
    31250: (0x0F, 0xF1, 0x85),
    25000: (0x0F, 0xBA, 0x07),
    20000: (0x0F, 0xFF, 0x87),
    10000: (0x1F, 0xFF, 0x87),
    5000: (0x3F, 0xFF, 0x87),
    666000: (0x00, 0xA0, 0x04),
}


def _has_elapsed(start, timeout_ms):
    return (monotonic_ns() - start) / 1000000 > timeout_ms


# doesn't obey debug=False for now
def _split_bin(int_number):
    bin_str = "{:032b}".format(int_number)
    bit_width = 32
    for start_idx in range(bit_width / 4):
        print(" " + bin_str[start_idx * 4 : (start_idx + 1) * 4], end="")
    print("")


def _tx_buffer_status_decode(status_byte):
    out_str = "Status: "
    # when CAN_H is disconnected?: 0x18
    out_str += "\nStatus of chosen buffer: %s\n" % hex(status_byte)
    if status_byte & 0x40:
        out_str += " Message ABORTED"
    if status_byte & 0x20:
        out_str += " Message LOST ARBITRATION"
    if status_byte & 0x10:
        out_str += " TRANSMIT ERROR"
    if status_byte & 0x8:
        out_str += " Transmit Requested"
    else:
        out_str += " Message sent"
    out_str += " Priority: " + ["LAST", "LOW", "MEDIUM", "HIGH"][status_byte & 0x3]

    return out_str


class MCP2515:
    """Library for the MCP2515 CANbus controller"""

    def __init__(
        self,
        spi_bus,
        cs_pin,
        *,
        baudrate: int = 250000,
        loopback: bool = False,
        silent: bool = False,
        auto_restart: bool = False,
        debug: bool = False
    ):
        """[summary]

        Args:
            spi_bus (busio.SPI): The SPI bus used to communicate with the MCP2515
            cs_pin (digitalio.DigitalInOut): SPI bus enable pin
            baudrate (int, optional):  The bit rate of the bus in Hz. All devices on the bus must \
                agree on this value. Defaults to 250000.
            loopback (bool, optional): When True the rx pin’s value is ignored, and the device \
                receives the packets it sends. Defaults to False.
            silent (bool, optional): When True the tx pin is always driven to the high logic level.\
                 This mode can be used to “sniff” a CAN bus without interfering.. Defaults to False.
            auto_restart (bool, optional): If True, will restart communications after entering \
                bus-off state. Defaults to False.
            debug (bool, optional): If True, will enable printing debug information. Defaults to \
                False.
        """
        if loopback and silent:
            raise AttributeError("only loopback or silent mode can bet set, not both")
        self._auto_restart = auto_restart
        self._debug = debug
        self.bus_device_obj = spi_device.SPIDevice(spi_bus, cs_pin)
        self._cs_pin = cs_pin
        self._buffer = bytearray(255)
        self._id_buffer = bytearray(4)
        self._unread_message_queue = []
        self._tx_buffers = []
        self._mode = None

        self._init_buffers()
        self.initialize(baudrate, loopback=loopback, silent=silent)

    def _init_buffers(self):

        self._tx_buffers = [
            TransmitBuffer(
                CTRL_REG=_TXB0CTRL,
                STD_ID_REG=_TXB0SIDH,
                INT_FLAG_MASK=_TX0IF,
                LOAD_CMD=_LOAD_TX0,
                SEND_CMD=_SEND_TX0,
            ),
            TransmitBuffer(
                CTRL_REG=_TXB1CTRL,
                STD_ID_REG=_TXB1SIDH,
                INT_FLAG_MASK=_TX1IF,
                LOAD_CMD=_LOAD_TX1,
                SEND_CMD=_SEND_TX1,
            ),
            TransmitBuffer(
                CTRL_REG=_TXB2CTRL,
                STD_ID_REG=_TXB2SIDH,
                INT_FLAG_MASK=_TX2IF,
                LOAD_CMD=_LOAD_TX2,
                SEND_CMD=_SEND_TX2,
            ),
        ]

    def initialize(self, baudrate, loopback=False, silent=False):
        """Return the sensor to the default configuration"""

        self._reset()
        # our mode set skips checking for sleep
        res = self._set_mode(_MODE_CONFIG)
        if not res:
            raise RuntimeError("Unable to set config mode")
        # SET MODE (FAILABLE)

        self._set_baud_rate(baudrate)

        # intialize TX and RX registers
        for idx in range(14):
            self._set_register(_TXB0CTRL + idx, 0)
            self._set_register(_TXB1CTRL + idx, 0)
            self._set_register(_TXB2CTRL + idx, 0)

        self._set_register(_RXB0CTRL, 0)
        self._set_register(_RXB1CTRL, 0)

        # # # interrupt mode
        # TODO: WHAT IS THIS
        self._set_register(_CANINTE, _RX0IF | _RX1IF)
        sleep(0.010)
        self._mod_register(
            _RXB0CTRL, _RXB_RX_MASK | _RXB_BUKT_MASK, _RXB_RX_STDEXT | _RXB_BUKT_MASK,
        )

        self._mod_register(_RXB1CTRL, _RXB_RX_MASK, _RXB_RX_STDEXT)
        if loopback:
            new_mode = _MODE_LOOPBACK
        elif silent:
            new_mode = _MODE_LISTENONLY
        else:
            new_mode = _MODE_NORMAL

        res = self._set_mode(new_mode)
        if not res:
            raise RuntimeError("Unable to set mode")
        return res

    def send(self, message_obj, wait_sent=True):  # pylint:disable=too-many-arguments
        """send a message buffer"""
        # send example call:
        # CAN.sendMsgBuf(tx_id=0x00, ext=0, rtrBit=0, len=8, buf=stmp, wait_sent=true)

        # TODO: Timeout
        tx_buff = self._get_tx_buffer()  # info = addr.

        # write_canMsg(tx_buffer_addr, tx_id, ext, rtrBit, len, byte * buf)

        # TODO: set buffer priority
        self._write_message(tx_buff, message_obj)
        if not wait_sent:
            return True
        sleep(0.010)

        send_confirmed = False
        start = monotonic_ns()
        while not send_confirmed:
            if _has_elapsed(start, _SEND_TIMEOUT_MS):
                raise RuntimeError("Timeout occoured waiting for transmit confirmation")
            # the status register address is whatever tx_buff_n is, minus one?
            tx_buff_status = self._read_register(tx_buff.CTRL_REG)
            self._dbg(_tx_buffer_status_decode(tx_buff_status))
            # tx_buffer_status = mcp2515_readRegister(txbuf_n - 1)  # read send buff ctrl reg
            # check for 0x08/0b00001000 being un-set
            send_confirmed = (tx_buff_status & _TXB_TXREQ_M) == 0

        return True

    @property
    def unread_message_count(self):
        """The number of messages that have been received but not read with `read_message`

        Returns:
            int: The unread message count
        """
        self._read_from_rx_buffers()

        return len(self._unread_message_queue)

    def read_message(self):
        """Read the next available message

        Returns:
            `canio.Message`: The next available message or None if one is not available
        """
        if self.unread_message_count == 0:
            return None

        return self._unread_message_queue.pop(0)

    def _read_rx_buffer(self, read_command):
        with self.bus_device_obj as spi:
            self._buffer[0] = read_command
            spi.write_readinto(
                self._buffer,  # because the reference does similar
                self._buffer,
                out_start=0,
                out_end=1,
                in_start=0,
                in_end=1,
            )

            spi.readinto(self._buffer, end=15)

        raw_idz = unpack_from(">I", self._buffer)[0]
        is_extended_id = (raw_idz & (1 << 19)) > 0
        if is_extended_id:
            sender_id = raw_idz & ((1 << 18) - 1)
        else:
            sender_id = (raw_idz & ((0b1111111111100000) << 16)) >> (16 + 5)

        dlc = self._buffer[4]
        # length is max 8
        message_length = min(8, dlc & 0xF)

        if (dlc & _RTR_MASK) > 0:
            frame_obj = RemoteTransmissionRequest(
                sender_id, message_length, extended=is_extended_id
            )
        else:
            frame_obj = Message(
                sender_id,
                data=bytes(self._buffer[5 : 5 + message_length]),
                extended=is_extended_id,
            )

        self._unread_message_queue.append(frame_obj)

    def _read_from_rx_buffers(self):
        """Read the next available message into the given `bytearray`

        Args:
            msg_buffer (bytearray): The buffer to load the message into
        """
        status = self._read_status()

        # TODO: read and store all available messages
        if status & 0b1:
            self._read_rx_buffer(_READ_RX0)

        if status & 0b10:
            self._read_rx_buffer(_READ_RX1)

    # pylint:disable=too-many-arguments
    def _write_message(self, tx_buffer, message_obj):

        if isinstance(message_obj, RemoteTransmissionRequest):
            dlc = message_obj.length
        else:
            dlc = len(message_obj.data)

        if dlc > _MAX_CAN_MSG_LEN:
            raise AttributeError("Message/RTR length must be <=%d" % _MAX_CAN_MSG_LEN)
        load_command = tx_buffer.LOAD_CMD

        if isinstance(message_obj, RemoteTransmissionRequest):
            dlc |= _RTR_MASK

        # get id buffer segment

        self._load_id_buffer(message_obj.id, message_obj.extended)

        # this splits up the id header, dlc (len, rtr status), and message buffer
        # TODO: check if we can send in one buffer, in which case `id_buffer` isn't needed

        with self.bus_device_obj as spi:
            # send write command for the given buffer
            self._buffer[0] = load_command
            # spi.write(self._buffer, end=1)
            spi.write_readinto(
                self._buffer,  # because the reference does similar
                self._buffer,
                out_start=0,
                out_end=1,
                in_start=0,
                in_end=1,
            )

            # send id bytes
            spi.write(self._id_buffer, end=4)

            # send DLC

            spi.write(bytearray([dlc]))
            # send message bytes, limit to 8?
            if isinstance(message_obj, Message):
                spi.write(message_obj.data, end=8)

        # send the frame based on the current buffers
        self._start_transmit(tx_buffer)

    # pylint:enable=too-many-arguments

    # TODO: Priority
    def _start_transmit(self, tx_buffer):
        #
        self._buffer[0] = tx_buffer.SEND_CMD
        with self.bus_device_obj as spi:
            spi.write_readinto(
                self._buffer,  # because the reference does similar
                self._buffer,
                out_start=0,
                out_end=1,
                in_start=0,
                in_end=1,
            )

    def _load_id_buffer(self, send_id, extended_id=False):
        _split_bin(send_id)
        if extended_id:

            extended_id = send_id & _EXTENDED_ID_MASK  # bottom 18 bits
            self._dbg("\tExtended ID:", "0x{:04X}".format(extended_id))
            pack_into(">I", self._id_buffer, 0, extended_id)

            id_buff_int = unpack_from(">I", self._id_buffer)[0]
            _split_bin(id_buff_int)

            # need to set top of buffer[1], so get the current value
            extid_msbits = self._id_buffer[1]
            # get the next 2 bytes of the given ID

            std_id = (send_id & 0x00FC0000) >> 18

            self._dbg("\tStd: ID", std_id)
            ms_bytes = std_id << 5 | _TXB_EXIDE_M | extid_msbits
            pack_into(">H", self._id_buffer, 0, ms_bytes)

            id_buff_int = unpack_from(">I", self._id_buffer)[0]
            _split_bin(id_buff_int)

        else:
            self._dbg("normal ID")
            # TODO: dry with the above
            std_id = send_id & 0x3FF  # The actual ID?
            self._dbg("\tStd: ID", hex(std_id))
            pack_into(">H", self._id_buffer, 0, std_id << 5)
            self._dbg("\tID Buffer:", self._id_buffer)

        id_buff_int = unpack_from(">I", self._id_buffer)[0]
        _split_bin(id_buff_int)

    @property
    def _tx_buffers_in_use(self):
        # the ref code allows for reserving buffers, but didn't see any way
        # to use them. maybe un-reserve then use?
        # TODO: this should return a tuple of busy states
        # byte status = mcp2515_readStatus() & MCP_STAT_TX_PENDING_MASK
        status = self._read_status()
        self._dbg("Status byte:", "{:#010b}".format(status))
        return (
            bool(status & _STAT_TX0_PENDING),
            bool(status & _STAT_TX1_PENDING),
            bool(status & _STAT_TX2_PENDING),
        )

    def _get_tx_buffer(self):
        """Get the address of the next available tx buffer and unset
        its interrupt bit in _CANINTF"""
        # check all buffers by looking for match on
        txs_busy = self._tx_buffers_in_use
        if all(txs_busy):
            self._dbg("none available!")
            return None
        buffer_index = txs_busy.index(False)  # => self._tx_buffers
        tx_buffer = self._tx_buffers[buffer_index]

        self._mod_register(_CANINTF, tx_buffer.INT_FLAG_MASK, 0)
        return tx_buffer

    def _set_baud_rate(self, baudrate=500000):

        # *******8 set baud rate ***********
        cnf1, cnf2, cnf3 = _BAUD_RATES[baudrate]

        self._dbg(
            "baud rate:",
            baudrate,
            "cnf1:",
            hex(cnf1),
            "cnf2:",
            hex(cnf2),
            "cnf3:",
            hex(cnf3),
        )

        self._set_register(_CNF1, cnf1)
        self._set_register(_CNF2, cnf2)
        self._set_register(_CNF3, cnf3)
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
            return True
        new_mode_set = self._request_new_mode(mode)
        if new_mode_set:
            self._mode = mode
        return new_mode_set

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
            spi.write(self._buffer, end=1)
            spi.readinto(self._buffer, start=0, end=1)
        return self._buffer[0]

    def _set_register(self, regsiter_addr, register_value):
        self._buffer[0] = _WRITE
        self._buffer[1] = regsiter_addr
        self._buffer[2] = register_value
        with self.bus_device_obj as spi:
            spi.write(self._buffer, end=3)

    ######## CANIO API METHODS #############

    @property
    def baudrate(self):
        """ The baud rate (read-only)"""

    @property
    def transmit_error_count(self):
        """ The number of transmit errors (read-only). Increased for a detected transmission error,\
             decreased for successful transmission. Limited to the range from 0 to 255 inclusive. \
                 Also called TEC."""

    @property
    def receive_error_count(self):
        """ The number of receive errors (read-only). Increased for a detected reception error, \
            decreased for successful reception. Limited to the range from 0 to 255 inclusive. Also
         called REC."""

    @property
    def error_warning_state_count(self):
        """ The number of times the controller enterted the Error Warning state (read-only). This\
             number wraps around to 0 after an implementation-defined number of errors."""

    @property
    def error_passive_state_count(self):
        """ The number of times the controller enterted the Error Passive state (read-only). This\
             number wraps around to 0 after an implementation-defined number of errors."""

    @property
    def bus_off_state_count(self):
        """ The number of times the controller enterted the Bus Off state (read-only). This number\
             wraps around to 0 after an implementation-defined number of errors."""

    @property
    def state(self):  # State
        """The current state of the bus. """

    @property
    def loopback(self):  # bool
        """True if the device was created in loopback mode, False otherwise"""
        return self._mode == _MODE_LOOPBACK

    @property
    def silent(self):  # bool
        """True if the device was created in silent mode, False otherwise"""
        return self._mode == _MODE_LISTENONLY

    def restart(self):
        """If the device is in the bus off state, restart it."""

    def listen(self, match=None, *, timeout: float = 10):
        """Start receiving messages that match any one of the filters.

        Creating a listener is an expensive operation and can interfere with reception of messages
        by other listeners.

    There is an implementation-defined maximum number of listeners and limit to the complexity of
    the filters.

    If the hardware cannot support all the requested matches, a ValueError is raised. Note that \
        generally there are some number of hardware filters shared among all fifos.

    A message can be received by at most one Listener. If more than one listener matches a message,\
         it is undefined which one actually receives it.

    An empty filter list causes all messages to be accepted.

    Timeout dictates how long `receive()` and `next()` will block.

        Args:
            match (Optional[Sequence[Match]], optional): [description]. Defaults to None.
            timeout (float, optional): [description]. Defaults to 10.

        Returns:
            Listener: [description]
        """
        self._dbg("match:", match)
        return Listener(self, timeout)

    # def send(self, message):
    #     """Send a message on the bus with the given data and id. If the message could not be sent
    #      due to a full fifo or a bus error condition, RuntimeError is raised.

    #     Args:
    #         message (canio.Message): The message to send. Must be a valid `canio.Message`
    #     """
    #     self.send_buffer(
    #         message.data, message.id, extended_id=message.extended, rtr=message.rtr
    #     )

    def deinit(self):
        """Deinitialize this object, freeing its hardware resources"""
        self._cs_pin.deinit()

    def __enter__(self):
        """Returns self, to allow the object to be used in a The with statement statement for \
            resource control"""
        return self

    def __exit__(self, unused1, unused2, unused3):
        """Calls deinit()"""
        self.deinit()

    ##################### End canio API ################

    def _dbg(self, *args, **kwargs):
        if self._debug:
            print("DBG::\t\t", *args, **kwargs)
