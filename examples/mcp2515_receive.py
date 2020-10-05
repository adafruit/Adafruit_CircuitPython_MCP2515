# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import digitalio
import board
import busio
from adafruit_mcp2515.canio import Timer
from adafruit_mcp2515.canio import RemoteTransmissionRequest, Message
from adafruit_mcp2515 import MCP2515 as CAN

cs = digitalio.DigitalInOut(board.D5)
cs.switch_to_output()
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
mcp = CAN(spi, cs)

t = Timer(timeout=5)

while True:
    # print occationally to show we're alive
    if t.expired:
        print(".", end="")
        t.rewind_to(1)
    with mcp.listen(timeout=1.0) as listener:
        message_count = listener.in_waiting()

        if message_count == 0:
            continue
        for _i in range(message_count):
            print(message_count, "messages available")
            msg = listener.receive()
            print("Message received from ", hex(msg.id))
            if isinstance(msg, Message):
                print("Message data:", msg.data)
                message_str = "::".join(["0x{:02X}".format(i) for i in msg.data])
                print(message_str)

            if isinstance(msg, RemoteTransmissionRequest):
                print("RTR length:", msg.length)
            print("")
