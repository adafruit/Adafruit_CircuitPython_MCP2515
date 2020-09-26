# SPDX-FileCopyrightText: Copyright (c) 2020 Jeff Epler for Adafruit Industries
#
# SPDX-License-Identifier: MIT

try:
    from canio import (
        Message,
        RemoteTransmissionRequest,
        Match,
        # BusState,
    )
    from board import CAN_RX, CAN_TX

    def builtin_bus_factory():
        return CAN(rx=CAN_RX, tx=CAN_TX, baudrate=1000000, loopback=True)


except ImportError as e:
    print("no native canio, trying mcp")
    from digitalio import DigitalInOut

    import board

    from adafruit_mcp2515.canio import (
        Message,
        RemoteTransmissionRequest,
        Match,
        # BusState,
    )
    from adafruit_mcp2515 import MCP2515 as CAN

    def builtin_bus_factory():
        cs = DigitalInOut(board.D5)
        cs.switch_to_output()
        return CAN(board.SPI(), cs, baudrate=1000000, loopback=True, debug=True)


max_standard_id = 0x7FF
max_extended_id = 0x1FFFFFFF
lengths = (0, 1, 2, 3, 4, 5, 6, 7, 8)


def test_message(_can=builtin_bus_factory):
    print("Testing Message")
    assert Message(id=0, data=b"").id == 0
    assert Message(id=1, data=b"").id == 1
    for i in lengths:
        b = bytes(range(i))
        assert Message(id=0, data=b).data == b
    assert not Message(id=0, data=b"").extended
    assert not Message(id=0, extended=False, data=b"").extended
    assert Message(id=0, extended=True, data=b"").extended


def test_rtr_constructor():

    print("Testing RemoteTransmissionRequest")
    assert RemoteTransmissionRequest(id=0, length=0).id == 0
    assert RemoteTransmissionRequest(id=1, length=0).id == 1
    assert (
        RemoteTransmissionRequest(id=0x1FFFFFFF, extended=True, length=1).id
        == 0x1FFFFFFF
    ), "Max extended ID not set properly"
    for i in lengths:
        assert RemoteTransmissionRequest(id=0, length=i).length == i
    assert not RemoteTransmissionRequest(id=0, length=1).extended
    assert not RemoteTransmissionRequest(id=0, extended=False, length=1).extended
    assert RemoteTransmissionRequest(id=0, extended=True, length=1).extended


def test_rtr_receive(can=builtin_bus_factory):

    with can() as b, b.listen(timeout=0.1) as l:

        for length in lengths:
            print("Test messages of length", length)

            mo = RemoteTransmissionRequest(id=0x5555555, extended=True, length=length)

            b.send(mo)
            mi = l.receive()
            assert mi
            assert isinstance(mi, RemoteTransmissionRequest)
            assert mi.id == 0x5555555, "Extended ID does not match: 0x{:07X}".format(
                mi.id
            )
            assert mi.extended
            assert mi.length == length

            mo = RemoteTransmissionRequest(id=max_standard_id, length=length)
            b.send(mo)
            mi = l.receive()
            assert mi
            assert isinstance(mi, RemoteTransmissionRequest)
            assert mi.id == max_standard_id
            assert mi.length == length

            mo = RemoteTransmissionRequest(id=0x555, length=length)
            b.send(mo)
            mi = l.receive()
            assert mi
            assert isinstance(mi, RemoteTransmissionRequest)
            assert mi.id == 0x555
            assert mi.length == length

            mo = RemoteTransmissionRequest(
                id=max_extended_id, extended=True, length=length
            )
            assert mo.extended
            b.send(mo)
            mi = l.receive()
            assert mi
            assert isinstance(mi, RemoteTransmissionRequest)
            assert mi.id == max_extended_id
            assert mi.length == length

            data = bytes(range(length))
            mo = Message(id=0, data=data)
            b.send(mo)
            mi = l.receive()
            assert mi
            assert isinstance(mi, Message)
            assert mi.data == data

            mo = Message(id=max_extended_id, extended=True, data=data)
            b.send(mo)
            mi = l.receive()
            assert mi
            assert isinstance(mi, Message)
            assert mi.data == data


def test_filters1(can=builtin_bus_factory):

    matches = [
        Match(0x408),
        Match(0x700, mask=0x7F0),
        Match(0x4081975, extended=True),
        Match(0x888800, mask=0xFFFFF0, extended=True),
    ]

    print("Test filters")
    with can() as b, b.listen(matches, timeout=0.1) as l:
        # exact ID matching
        mo = RemoteTransmissionRequest(id=0x408, length=0)
        b.send(mo)
        mi = l.receive()
        assert mi

        # mask matching
        mo = RemoteTransmissionRequest(id=0x704, length=0)
        b.send(mo)
        mi = l.receive()
        assert mi

        # non-matching
        mo = RemoteTransmissionRequest(id=0x409, length=0)
        b.send(mo)
        mi = l.receive()
        assert not mi

        # Extended
        # exact ID matching
        mo = Message(id=0x4081975, extended=True, data=b"")
        b.send(mo)
        mi = l.receive()
        assert mi

        # mask matching
        mo = Message(id=0x888804, extended=True, data=b"")
        b.send(mo)
        mi = l.receive()
        assert mi

        # non-matching
        mo = Message(id=0x4081985, extended=True, data=b"")
        b.send(mo)
        mi = l.receive()
        assert not mi


def test_filters2(can=builtin_bus_factory):
    matches = [
        Match(0x408),
        Match(0x700, mask=0x7F0),
        Match(0x4081975, extended=True),
        Match(0x888800, mask=0xFFFFF0, extended=True),
    ]
    print("Test filters 2")
    with can() as b, b.listen(matches[0:2], timeout=0.1) as l1, b.listen(
        matches[2:4], timeout=0.1
    ) as l2:
        # exact ID matching
        mo = RemoteTransmissionRequest(id=0x408, length=0)
        b.send(mo)
        mi1 = l1.receive()
        mi2 = l2.receive()
        assert mi1
        assert not mi2

        # mask matching
        mo = RemoteTransmissionRequest(id=0x704, length=0)
        b.send(mo)
        mi1 = l1.receive()
        mi2 = l2.receive()
        assert mi1
        assert not mi2

        # non-matching
        mo = RemoteTransmissionRequest(id=0x409, length=0)
        b.send(mo)
        mi1 = l1.receive()
        mi2 = l2.receive()
        assert not mi1
        assert not mi2

        # Extended
        # exact ID matching
        mo = Message(id=0x4081975, extended=True, data=b"")
        b.send(mo)
        mi1 = l1.receive()
        mi2 = l2.receive()
        assert not mi1
        assert mi2

        # mask matching
        mo = Message(id=0x888804, extended=True, data=b"")
        b.send(mo)
        mi1 = l1.receive()
        mi2 = l2.receive()
        assert not mi1
        assert mi2

        # non-matching
        mo = Message(id=0x4081985, extended=True, data=b"")
        b.send(mo)
        mi1 = l1.receive()
        mi2 = l2.receive()
        assert not mi1
        assert not mi2


def test_iter(can=builtin_bus_factory):

    print("Test iter()")
    with can() as b, b.listen(timeout=0.1) as l:
        assert iter(l) is l
        ### This mo is taken from the last mo= statement from the above scope
        # previously only worked because it was left over
        mo = Message(id=0x4081985, extended=True, data=b"")

        mo.data = b"\xff"
        b.send(mo)
        for i, mi in zip(range(3), l):
            print(i, mi.data)
            assert mi
            assert mi.data == mo.data
            mo.data = bytes((i,))
            b.send(mo)
        mi = next(l)
        assert mi
        assert mi.data == mo.data
        try:
            next(l)
        except StopIteration:
            print("StopIteration")


test_suite = [
    test_message,
    test_rtr_constructor,
    test_rtr_receive,
    test_filters1,
    test_filters2,
    test_iter,
]
for test in test_suite:
    test()
