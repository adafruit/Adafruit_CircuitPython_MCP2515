# SPDX-FileCopyrightText: Copyright (c) 2020 Kevin Schlosser for Adafruit Industries
#
# SPDX-License-Identifier: MIT


import digitalio
import board
import busio
from adafruit_mcp2515 import Bitrate
from adafruit_mcp2515 import MCP2515 as CAN

bitrate = Bitrate(333000, sample_point=65.0, sjw=2, calc_tolerance=1.5)
cs = digitalio.DigitalInOut(board.D5)
cs.switch_to_output()
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
mcp = CAN(spi, cs, baudrate=bitrate, silent=True)

