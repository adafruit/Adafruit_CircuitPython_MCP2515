# SPDX-FileCopyrightText: Copyright (c) 2020 Kevin Schlosser for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# enumerating all timing registers for a baudrate would be
# used for a selection process where the user would be provided
# a list of choices based on the baudrate they have provided.
# The user can then select which one they would like to use and
# that object can then be passed to the constructor for the interface.

from adafruit_mcp2515 import Bitrate

bitrates = Bitrate.get_bitrates(
    333000,
    Bitrate.TimingConstants.MCP251x16Const(),
    calc_tolerance=1.5
)

for bitrate in bitrates:
    print('bitrate:', bitrate.bitrate)
    print('sample point:', bitrate.sample_point)
    print('sjw:', bitrate.sjw)
    print('cnf1:', hex(bitrate.cnf1)[2:].upper().zfill(2))
    print('cnf2:', hex(bitrate.cnf1)[2:].upper().zfill(2))
    print('cnf3:', hex(bitrate.cnf1)[2:].upper().zfill(2))
    print()
