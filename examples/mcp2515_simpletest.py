# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
from time import sleep
import board
import busio
from digitalio import DigitalInOut
from adafruit_mcp2515.canio import Message
from adafruit_mcp2515 import MCP2515 as CAN


cs = DigitalInOut(board.D5)
cs.switch_to_output()
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)

can_bus = CAN(
    spi, cs, loopback=True, silent=True
)  # use loopback to test without another device
while True:
    with can_bus.listen(timeout=1.0) as listener:

        message = Message(id=0x1234ABCD, data=b"adafruit", extended=True)
        can_bus.send(message)
        message_count = listener.in_waiting()
        print(message_count, "messages available")
        for _i in range(message_count):
            msg = listener.receive()
            print("Message from ", hex(msg.id))
            print("message data:", msg.data)
            print("")
    sleep(1)
