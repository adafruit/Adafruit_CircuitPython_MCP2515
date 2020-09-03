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
# spi = board.SPI()
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
mcp = adafruit_mcp2515.MCP2515(spi, cs)
# mcp.send_buffer(tx_id=0x00, ext=0, rtrBit=0, len=8, buf=stmp, wait_sent=true)
max_ext_id = 0x3FFFF
mb1 = [0xDE, 0xAD, 0xBE, 0xEF]
mb2 = [0xCA, 0xFE, 0xFA, 0xDE]
while True:
    mb1.insert(0, mb2.pop(-1))
    mb2.insert(0, mb1.pop(-1))
    mcp.send_buffer(bytearray(mb1 + mb2), 0b100000000000000001, extended_id=True)
    # mcp.send_buffer(bytearray(mb1 + mb2),0xF4, extended_id=False)
    sleep(0.3)
    print("*" * 40)
