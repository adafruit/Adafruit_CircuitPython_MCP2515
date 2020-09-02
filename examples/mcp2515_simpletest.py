# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
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
print("out of Init code!")
# mcp.send_buffer(tx_id=0x00, ext=0, rtrBit=0, len=8, buf=stmp, wait_sent=true)

message_buffer = bytearray([0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xFA, 0xDE])
mcp.send_buffer(message_buffer, 0x13)
