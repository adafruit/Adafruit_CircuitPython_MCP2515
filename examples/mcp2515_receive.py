# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

from time import sleep
import board
import busio
import digitalio
import adafruit_mcp2515

CS_PIN = 5
cs = digitalio.DigitalInOut(board.D5)
cs.direction = digitalio.Direction.OUTPUT
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
mcp = adafruit_mcp2515.MCP2515(spi, cs)

while True:
    with mcp.listen(timeout=1.0) as listener:
        message_count = listener.in_waiting()
        if message_count == 0:
            continue
        print(message_count, "messages available")
        msg = listener.read()
        print("Message received from ", hex(msg.id))

        message_str = "::".join(["0x{:02X}".format(i) for i in msg.data])
        print(message_str)
        print("")
        sleep(0.5)
