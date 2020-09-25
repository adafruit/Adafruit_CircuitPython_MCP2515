# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

from time import sleep
from board import SPI, D5 as CS_PIN
import digitalio
import adafruit_mcp2515
from adafruit_mcp2515.canio import Message

cs = digitalio.DigitalInOut(CS_PIN)
cs.switch_to_output()
mcp = adafruit_mcp2515.MCP2515(SPI(), cs)

max_ext_id = 0x3FFFF
mb1 = [0xDE, 0xAD, 0xBE, 0xEF]
mb2 = [0xCA, 0xFE, 0xFA, 0xDE]
can_id = 0b100000000000000001
while True:
    mb1.insert(0, mb2.pop())
    mb2.insert(0, mb1.pop())
    message = Message(id=0xFFAA, data=bytes(mb1 + mb2), extended=True)
    mcp.send(message)
    sleep(0.3)
    print("*" * 40)
