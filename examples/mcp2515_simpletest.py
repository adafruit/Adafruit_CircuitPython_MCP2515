# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

from time import sleep
import board
import busio
import digitalio
import adafruit_mcp2515
from adafruit_mcp2515.canio import Message

# canio reference:
# import canio
# from board import *

# can = canio.CAN(board.CAN_RX, board.CAN_TX, baudrate=1000000)
# message = canio.Message(id=0x0408, data="adafruit"
# can.write(message))
# can.deinit()

CS_PIN = 5
cs = digitalio.DigitalInOut(board.D5)
cs.direction = digitalio.Direction.OUTPUT
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
mcp = adafruit_mcp2515.MCP2515(spi, cs)
# mcp.send_buffer(tx_id=0x00, ext=0, rtrBit=0, len=8, buf=stmp, wait_sent=true)
max_ext_id = 0x3FFFF
mb1 = [0xDE, 0xAD, 0xBE, 0xEF]
mb2 = [0xCA, 0xFE, 0xFA, 0xDE]
can_id = 0b100000000000000001
while True:
    mb1.insert(0, mb2.pop())
    mb2.insert(0, mb1.pop())
    message = Message(id=0xFFAA, data=bytes(mb1 + mb2), extended=True)
    mcp.write(message)
    sleep(0.3)
    print("*" * 40)
