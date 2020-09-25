# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# import board
# import busio
# import digitalio
# import adafruit_mcp2515
# from adafruit_mcp2515.canio import Timer


from digitalio import DigitalInOut
from board import D5 as CS_PIN, SPI
from adafruit_mcp2515.canio import Timer
from adafruit_mcp2515 import MCP2515 as CAN

# from board import CAN_RX, CAN_TX


def bus():
    cs = DigitalInOut(CS_PIN)
    cs.switch_to_output()
    return CAN(SPI(), cs)


# cs = digitalio.DigitalInOut(board.D5)
# cs.direction = digitalio.Direction.OUTPUT
# spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
# can = adafruit_mcp2515.MCP2515(spi, cs)

t = Timer(timeout=5)
while True:
    with bus() as can, can.listen(timeout=1.0) as listener:
        message_count = listener.in_waiting()
        print(message_count, "messages available")
        if message_count == 0:
            continue
        for _i in range(message_count):
            msg = listener.receive()
            print("Message received from ", hex(msg.id))
            print("message data:", msg.data)
            message_str = "::".join(["0x{:02X}".format(i) for i in msg.data])
            print(message_str)
            print("RTR:", msg.rtr)
            print("")
        # instead of sleeping, pool for messages to fill queue
        t.rewind_to(5)
        while not t.expired:
            message_count = listener.in_waiting()
