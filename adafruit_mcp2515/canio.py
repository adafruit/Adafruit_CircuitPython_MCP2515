# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""Python implementation of the CircuitPython core `canio` API"""
# pylint:disable=too-few-public-methods


class Message:
    """A class representing a message to send on a `canio` bus
    """

    # pylint:disable=too-many-arguments,invalid-name,redefined-builtin
    def __init__(self, id, data=None, size=None, rtr=False, extended=False):
        """Create a `Message` to send

        Args:
            id (int): The numeric ID of the message
            data (bytes): The content of the message
            size (int): The amount of data requested, for an rtr
            rtr (bool): True if the message represents an rtr (Remote Transmission Request)
            extended (bool): True if the
        Raises:
            AttributeError: If `data` of type `bytes` is not provided for a non-RTR message
            AttributeError: If `data` is larger than 8 bytes
            AttributeError: If `size` is set for a non-RTR message
        """
        if (data is None or not isinstance(data, bytes)) and not rtr:
            raise AttributeError(
                "non-RTR canio.Message must have a `data` argument of type `bytes`"
            )
        if len(data) > 8:
            raise AttributeError(
                "`canio.Message` object data must be of length 8 or less"
            )
        if size is not None and not rtr:
            raise AttributeError("non-RTR canio.Message should not set a `size`")

        self._rtr = rtr
        self.id = id
        self._data = data
        self.extended = extended
        self.size = None

    @property
    def data(self):
        """The content of the message, or dummy content in the case of an rtr"""
        return self._data

    @data.setter
    def data(self, new_data):
        if self.rtr:
            self.rtr = False
        if len(new_data) > 8:
            raise AttributeError(
                "`canio.Message` object data must be of length 8 or less"
            )
        self._data = new_data

    @property
    def rtr(self):
        """True if the message represents a remote transmission request (RTR). Setting rtr to true\
            zeros out data
        """
        return self._rtr

    @rtr.setter
    def rtr(self, is_rtr):
        if is_rtr and self._data:
            self._data = bytes(len(self._data))
        self._rtr = True


#### Leftovers from the initial version of the MCP lib, to be removed/replaced with canio classes
class CANIdentifier:
    """A class representing the ID of a `canio` message"""

    def __init__(self):
        self.id = None  # pylint:disable=invalid-name
        self.is_extended = False


class CANMessage:
    """A class representing a CAN bus message"""

    def __init__(self):
        self.bytes = bytearray(8)
        self.rtr = False
        self.can_id = CANIdentifier()
