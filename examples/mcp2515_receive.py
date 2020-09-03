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

msg_buffer = bytearray(8)
# CAN.checkReceive())
# CAN.readMsgBuf(&len, buf);

# unsigned long canId = CAN.getCanId()
while True:
    msg_count = mcp.unread_message_count
    if msg_count > 0:
        print(msg_count, "messages available")
        mcp.read_message_into(msg_buffer)
        send_id = mcp.last_send_id
        print("Message received from ", hex(send_id))
        message_str = "::".join(["0x{:02X}".format(i) for i in msg_buffer])
        print(message_str)
        print("")
    sleep(0.5)
