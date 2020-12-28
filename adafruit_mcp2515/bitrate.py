# SPDX-FileCopyrightText: Copyright (c) 2020 Kevin Schlosser for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_mcp2515.bitrate`
================================================================================

Calculator for the 3 timing registers needed to set the
baudrate(bitrate) on the mcp2515 interface.


* Author(s): Kevin Schlosser
"""

from typing import Optional, Union


def _get_cia_sample_point(bitrate):
    if bitrate > 800000:
        sampl_pt = 75.0
    elif bitrate > 500000:
        sampl_pt = 80.0
    else:
        sampl_pt = 87.5

    return sampl_pt


class _TimingConst(object):
    """
    Constants used to calculate the bit timing registers

    If wanting to expand the bitrate calculations so it will support a
    different interface family this class would need to be subclassed
    and the approptiate values set to properly calculate the timing registers
    for that interface family.
    """
    tseg1_min = 4
    tseg1_max = 16
    tseg2_min = 2
    tseg2_max = 8
    sjw_max = 4
    brp_min = 1
    brp_max = 64
    brp_inc = 1
    fsys = 16000000
    brp_extension = 1


class _MCP251xConst(_TimingConst):
    tseg1_min = 3
    tseg1_max = 16
    tseg2_min = 2
    tseg2_max = 8
    sjw_max = 4
    brp_min = 1
    brp_max = 64
    brp_inc = 1


# class _MSCanConst(_TimingConst):
#     tseg1_min = 4
#     tseg1_max = 16
#     tseg2_min = 2
#     tseg2_max = 8
#     sjw_max = 4
#     brp_min = 1
#     brp_max = 64
#     brp_inc = 1
#
#
# class _AT91Const(_TimingConst):
#     tseg1_min = 4
#     tseg1_max = 16
#     tseg2_min = 2
#     tseg2_max = 8
#     sjw_max = 4
#     brp_min = 2
#     brp_max = 128
#     brp_inc = 1
#
#
# class _FlexCanConst(_TimingConst):
#     tseg1_min = 4
#     tseg1_max = 16
#     tseg2_min = 2
#     tseg2_max = 8
#     sjw_max = 4
#     brp_min = 1
#     brp_max = 256
#     brp_inc = 1


class TimingConstants(object):
    """
    Constants used to calculate the timing registers
    """
    class MCP251x16Const(_MCP251xConst):
        fsys = 16000000

    class MPC251x32Const(_MCP251xConst):
        fsys = 32000000

    # class MSCan32Const(_MSCanConst):
    #     fsys = 32000000
    #
    # class MSCan33Const(_MSCanConst):
    #     fsys = 33000000
    #
    # class MSCan333Const(_MSCanConst):
    #     fsys = 33300000
    #
    # class MSCan33333333Const(_MSCanConst):
    #     fsys = 33333333
    #
    # class MSCanMPC51211Const(_MSCanConst):
    #     fsys = 66660000
    #
    # class MSCanMPC51212Const(_MSCanConst):
    #     fsys = 66666666
    #
    # class AT91RonetixConst(_AT91Const):
    #     fsys = 99532800
    #
    # class AT91100Const(_AT91Const):
    #     fsys = 100000000
    #
    # class FlexCanMX28Const(_FlexCanConst):
    #     fsys = 24000000
    #
    # class FlexCanMX6Const(_FlexCanConst):
    #     fsys = 30000000
    #
    # class FlexCan49Const(_FlexCanConst):
    #     fsys = 49875000
    #
    # class FlexCan66Const(_FlexCanConst):
    #     fsys = 66000000
    #
    # class FlexCan665Const(_FlexCanConst):
    #     fsys = 66500000
    #
    # class FlexCan666Const(_FlexCanConst):
    #     fsys = 66666666
    #
    # class FlexCanVYBRIDConst(_FlexCanConst):
    #     fsys = 83368421
    #
    # class SJA1000Const(_TimingConst):
    #     tseg1_min = 1
    #     tseg1_max = 16
    #     tseg2_min = 1
    #     tseg2_max = 8
    #     sjw_max = 4
    #     brp_min = 1
    #     brp_max = 64
    #     brp_inc = 1
    #     fsys = 16000000
    #
    # class TIHeccConst(_TimingConst):
    #     tseg1_min = 1
    #     tseg1_max = 16
    #     tseg2_min = 1
    #     tseg2_max = 8
    #     sjw_max = 4
    #     brp_min = 1
    #     brp_max = 256
    #     brp_inc = 1
    #     fsys = 26000000
    #
    # class RCARCanConst(_TimingConst):
    #     tseg1_min = 4
    #     tseg1_max = 16
    #     tseg2_min = 2
    #     tseg2_max = 8
    #     sjw_max = 4
    #     brp_min = 1
    #     brp_max = 1024
    #     brp_inc = 1
    #     fsys = 130000000


class Bitrate(object):
    """
    Bitrate timing register calculator.

    This class is used to calculate the values used to set the
    3 timing registers (CNF1, CNF2 and CNF3) on the MCP2515 interface.

    This calculator also allows the user to be able to "tweak" the register
    values in order to obtain the optimal settings needed for the CAN-Bus
    network they are using the interface on.

    ***NOTE***
    This calculator may not return an identical match to the baudrate (bitrate)
    the user is wanting to use. This is OK and normal, the interface not be able
    to directly support a given bitrate because of it's system clock. Due to how
    CAN-Bus has been engineered this problem has be taken into considertion and
    the specification does allow for "wiggle room". Changing the Synchronization
    Jump Width (SJW) is what controls how much wiggle room there is. The values
    the sjw can be set to is 1-4, 4 being the most wiggle room. There is a network
    performance hit each time a synchronization occurs so it is ideal to keep the
    bitrate as close as possible to the network bitrate.
    """
    TimingConstants = TimingConstants

    sync_seg = 1

    def __init__(
            self,
            bitrate,
            sample_point: Optional[Union[int, float]] = None,
            sjw: int = 1,
            number_of_samples: int = 1,
            calc_tolerance: float = 2.0,
            bus_length: int = 1,
            transceiver_delay: int = 150
    ):
        """

        Simple to use tool to calculate the bit timing registers.

        .. code-block:: python

        bt = Bitrate(500000, sample_point=87.5, sjw=1)
        if bt.calc_bit_timing(TimingConstants.MCP251x16Const()):
            print(
                bt.bitrate,
                ':',
                hex(bt.cnf1)[2:].upper().zfill(2),
                hex(bt.cnf2)[2:].upper().zfill(2),
                hex(bt.cnf3)[2:].upper().zfill(2)
            )
        else:
            print('calculations failed')


        outputs: `500000 : 00 B5 01`

        If the user is to select a bitrate you can enumerate all
        possible timing register values for a specific bitrate.

        .. code-block:: python

        for bt in Bitrate.get_bitrates(
            500000,
            TimingConstants.MCP251x16Const(),
            cia_compliant=False,
            calc_tolerance=0.0
        ):
            print(
                'sample_point: {0:.2f}'.format(bt.sample_point),
                'sjw:',
                bt.sjw,
                'registers:',
                hex(bt.cnf1)[2:].upper().zfill(2),
                hex(bt.cnf2)[2:].upper().zfill(2),
                hex(bt.cnf3)[2:].upper().zfill(2)
            )

        outputs:

            sample_point: 50.00 sjw: 1 registers: 00 9A 07
            sample_point: 50.00 sjw: 2 registers: 40 9A 07
            sample_point: 50.00 sjw: 3 registers: 80 9A 07
            sample_point: 50.00 sjw: 4 registers: C0 9A 07
            sample_point: 56.25 sjw: 1 registers: 00 A2 06
            sample_point: 56.25 sjw: 2 registers: 40 A2 06
            sample_point: 56.25 sjw: 3 registers: 80 A2 06
            etc.....


        :param bitrate: target bitrate
        :type bitrate: int

        :param sample_point: target sample point, if no sample point is
        provided then the program will calculate what the CiA recommended
        sample point based on the provided bitrate.
        :type sample_point: int, float

        :param sjw: synchronization jump width
        :type sjw: int

        :param number_of_samples: not used
        :type number_of_samples: int

        :param calc_tolerance: allowed percentage deviation from the target bitrate
        :type calc_tolerance: float

        :param bus_length: length of the physical canbus network
        :type bus_length: int

        :param transceiver_delay: processing delay of the nodes on the network in nanoseconds
        :type transceiver_delay: int
        """

        self._nominal_bitrate = bitrate
        if sample_point is None:
            self._nominal_sample_point = _get_cia_sample_point(bitrate)
        else:
            self._nominal_sample_point = sample_point

        self._calc_tolerance = calc_tolerance
        self._bus_length = bus_length
        self._transceiver_delay = transceiver_delay

        self._bitrate = None
        self._fsys = 0
        self._brp_extension = 0
        self._tseg1 = 0
        self._tseg2 = 0
        self._sjw = sjw

        if number_of_samples not in (1, 3):
            raise ValueError("number_of_samples must be 1 or 3")

        self._number_of_samples = number_of_samples

    @property
    def is_cis_sample_point(self) -> bool:
        """
        Checks to see if the sample point and bitrate
        conform to CiA (CAN in Automation) standards

        :return: `True` or `False`
        :rtype: bool
        """
        cia_sample_point = _get_cia_sample_point(self.bitrate)
        return self.sample_point == cia_sample_point

    @staticmethod
    def _calculate_bitrates(
        nominal_bitrate,
        calc_tolerance,
        cia_compliant,
        timing_const
    ):

        tmp = timing_const.fsys / nominal_bitrate / 2
        for brp in range(1, timing_const.brp_max + 1):
            tq = tmp / brp
            rtq = round(tq)

            if 32 >= rtq >= 4:
                for tseg1 in range(2, 19):
                    tseg2 = rtq - tseg1
                    if tseg1 < tseg2 or tseg2 > 8 or tseg2 < 2:
                        continue

                    err = -(tq / rtq - 1)
                    err = round(err * 1e4) / 1e4

                    bitrate = int(round(nominal_bitrate * (1 - err)))
                    br_err = (
                        abs(bitrate - nominal_bitrate) /
                        nominal_bitrate
                    ) * 100

                    if br_err > calc_tolerance:
                        continue

                    for sjw in range(1, 5):
                        br = Bitrate(
                            nominal_bitrate,
                            sjw=sjw,
                            calc_tolerance=calc_tolerance
                        )

                        br._bitrate = bitrate
                        br._brp = brp
                        br._tseg1 = tseg1
                        br._tseg2 = tseg2
                        br._fsys = timing_const.fsys
                        br._brp_extension = timing_const.brp_extension

                        if not (
                            cia_compliant or
                            (cia_compliant and br.is_cis_sample_point)
                        ):
                            yield br

    @staticmethod
    def get_bitrates(
        nominal_bitrate: int,
        timing_const: _TimingConst,
        cia_compliant: bool = False,
        calc_tolerance: float = 0.0
    ):
        """
        Enumerates all timing registers available for a given bitrate.

        :param nominal_bitrate: target bitrate
        :type nominal_bitrate: int

        :param timing_const: _TimingConst subclass that holds information about a specific interface family.
        :type timing_const: _TimingConst

        :param cia_compliant: Only return CiA compliant registers.
        :type cia_compliant: bool

        :param calc_tolerance: deviation from the target bitrate. This is a percentage value.
        :type calc_tolerance: bool

        :rtype: generator

        """
        return iter(Bitrate._calculate_bitrates(
            nominal_bitrate,
            calc_tolerance,
            cia_compliant,
            timing_const
        ))

    def __can_calc_bittiming(self, timing_const):
        nominal_bitrate = self._nominal_bitrate
        nominal_sample_point = self._nominal_sample_point

        match = None

        tmp = timing_const.fsys / nominal_bitrate / 2
        for brp in range(1, timing_const.brp_max + 1):
            tq = tmp / brp
            rtq = round(tq)

            if 32 >= rtq >= 4:
                for tseg1 in range(2, 19):
                    tseg2 = rtq - tseg1
                    if tseg1 < tseg2 or tseg2 > 8 or tseg2 < 2:
                        continue

                    err = -(tq / rtq - 1)
                    err = round(err * 1e4) / 1e4

                    sample_point = round(tseg1 / rtq * 1e4) / 100
                    bitrate = int(round(nominal_bitrate * (1 - err)))

                    sp_err = (
                        abs(sample_point - nominal_sample_point) /
                        nominal_sample_point
                    ) * 100
                    br_err = (abs(bitrate - nominal_bitrate) / nominal_bitrate) * 100

                    if br_err > self._calc_tolerance:
                        continue

                    tmp_match = (br_err, sp_err, bitrate, brp, tseg1, tseg2)

                    if (
                            bitrate == nominal_bitrate and
                            sample_point == nominal_sample_point
                    ):
                        match = tmp_match
                        break
                    elif match is None:
                        match = tmp_match

                    elif match[0] > br_err and match[1] > sp_err:
                        match = tmp_match
                else:
                    continue

                break

        if match is None:
            return False

        self._bitrate, self._brp, self._tseg1, self._tseg2 = match[2:]
        self._fsys = timing_const.fsys
        self._brp_extension = timing_const.brp_extension

        return True

    def calc_bit_timing(self, timing_const: _TimingConst) -> bool:
        """
        Calculate Bit Timings

        Mainly used internally for calculating timings from a bitrate and optionally a sample point

        :param timing_const: subclass of :class: `_TimingConst`
        :return: `True` if successful else `False`
        :rtype: bool

        :raises: ValueError if not able to calculate the timings
        """
        if self._bitrate is not None:
            return True

        return self.__can_calc_bittiming(timing_const)

    @property
    def bus_length(self) -> int:
        """
        Physical length of the CAN-Bus network

        Work in progress

        :return: length of network cabling
        :rtype: int
        """
        return self._bus_length

    @property
    def transceiver_delay(self) -> int:
        """
        Processing delay of a CAN-Bus node.

        Work in progress

        :return: nanosecond resolution of delay
        :rtype: int
        """
        return self._transceiver_delay

    @property
    def nominal_sample_point(self) -> float:
        """
        Target sample point supplied by the user
        when constructing an instance of this class.

        If no sample point was supplied then a CiA compliant
        sample point was generated based on the bitrate.

        :return: target sample point
        :rtype: float
        """
        return self._nominal_sample_point

    @property
    def sample_point(self) -> float:
        """
        Closest matching sample point after calculation of the bittiming registers.
        :return: best sample point match
        :rtype: float
        """
        sample_point = round(self.tseg1 / self.btq * 1e4) / 100.0
        return sample_point

    @property
    def sample_point_error(self) -> float:
        """
        Difference between the nominal sample point and the calculated sample point

        :return: difference represented as a 0-100 percentage difference.
        :rtype: float
        """
        err = (
            abs(self.sample_point - self.nominal_sample_point) /
            self.nominal_sample_point
        ) * 100
        return err

    @property
    def nominal_bitrate(self) -> int:
        """
        Bitrate supplised by the user when constructing an instance of this class

        :return: user supplised bitrate
        :rtype: int
        """
        return self._nominal_bitrate

    @property
    def bitrate(self) -> int:
        """
        Calculated bitrate

        Closest matching bitrate to the user supplied bitrate.
        There is an error window that can be supplied when constructing this class.

        :return: calculated bitrate
        :rtype: int
        """
        return self._bitrate

    @property
    def bitrate_error(self) -> float:
        """
        Difference between the nominal bitrate and the calculated bitrate

        :return: difference represented as a 0-100 percentage difference.
        :rtype: float
        """
        err = (abs(self.bitrate - self.nominal_bitrate) / self.nominal_bitrate) * 100
        return err

    @property
    def tq(self) -> float:
        """
        Time Quantum

        The length of the time quantum (tq), which is the basic time unit of the bit time,
        is defined by the CAN controller’s system clock fsys and the Baud Rate Prescaler (brp)

        :return: tq
        :rtype: float
        """
        tmp = self._fsys / self._nominal_bitrate / 2
        tq = tmp / self.brp
        return tq

    @property
    def btq(self) -> int:
        """
        Rounded Time Quantum (tq)

        :return: floored rounded tq
        :rtype: int
        """
        return int(self.tq)

    @property
    def prop_delay(self) -> int:
        """
        Total delay caused by physical devices and media.

        :return: delay
        :rtype: int
        """
        prop_delay = 2 * ((self._bus_length * 5) + self._transceiver_delay)
        return prop_delay

    @property
    def prop_seg(self) -> int:
        """
        Propigation Time Segment

        Used to compensate physical delay times within the network
        :rtype: int
        """
        return round(self.tseg1 / 2)

    @property
    def phase_seg1(self) -> int:
        """
        Phase Buffer Segment 1

        Used to compensate for the oscillator tolerance.
        The phase_seg1 and phase_seg2 smay be lengthened or shortened by synchronization.

        :rtype: int
        """
        phase_seg1 = self.tseg1 - self.prop_seg
        return phase_seg1

    @property
    def phase_seg2(self) -> int:
        """
        Phase Buffer Segment 2

        Used to compensate for the oscillator tolerance.
        The phase_seg1 and phase_seg2 smay be lengthened or shortened by synchronization.

        :rtype: int
        """
        phase_seg2 = self.tseg2 - 1

        return phase_seg2

    @property
    def tseg1(self) -> int:
        """

        :rtype: int
        """
        return self._tseg1

    @property
    def tseg2(self) -> int:
        """

        :rtype: int
        """
        return self._tseg2

    @property
    def nbt(self) -> int:
        """
        Nominal Bit Time

        :rtype: int
        """
        return self.sync_seg + self.tseg1 + self.tseg2 + 2

    @property
    def brp(self) -> int:
        """
        Bitrate Prescalar

        :rtype: int
        """
        return self._brp

    @property
    def sjw(self) -> int:
        """
        Synchronization Jump Width

        Used to compensate for the oscillator tolerance.

        :rtype: int
        """
        return self._sjw

    @property
    def number_of_samples(self) -> int:
        """
        Not used
        :return:
        """
        return self._number_of_samples

    @property
    def fsys(self) -> int:
        """
        CAN controller’s system clock (fsys)

        :return:
        """
        return self._fsys

    # @property
    # def btr0(self) -> int:
    #     """
    #     Bit Timing register used in sja1000 based interfaces
    #
    #     :rtype: int
    #     """
    #     return self.brp - 1 + (self.sjw - 1) * 64
    #
    # @property
    # def btr1(self) -> int:
    #     """
    #     Bit Timing register used in sja1000 based interfaces
    #
    #     :rtype: int
    #     """
    #     return self.tseg1 - 2 + (self.tseg2 - 1) * 16
    #
    # @property
    # def can0bt(self) -> int:
    #     """
    #     Bit Timing register used in Silabs based interfaces
    #
    #     :rtype: int
    #     """
    #     return self.brp + 64 * (self.sjw - 1) + self.tseg1 * 256 + self.tseg2 * 4096
    #
    # @property
    # def canbr(self) -> int:
    #     """
    #     Bit Timing register used in Atmel based interfaces
    #
    #     :rtype: int
    #     """
    #     br = (
    #             (self.phase_seg2 - 1) |
    #             ((self.phase_seg1 - 1) << 4) |
    #             ((self.prop_seg - 1) << 8) |
    #             ((self.sjw - 1) << 12) |
    #             ((self.brp - 1) << 16)
    #     )
    #     return br
    #
    # @property
    # def canctrl(self) -> int:
    #     """
    #     Bit Timing register used in Microchip based interfaces
    #
    #     :rtype: int
    #     """
    #
    #     ctrl = (
    #             ((self.brp - 1) << 24) |
    #             ((self.sjw - 1) << 22) |
    #             ((self.phase_seg1 - 1) << 19) |
    #             ((self.phase_seg2 - 1) << 16) |
    #             ((self.prop_seg - 1) << 0)
    #     )
    #
    #     return ctrl

    @property
    def cnf1(self):
        """
        Bit Timing register used in Microchip based interfaces

        :rtype: int
        """
        return self.brp - 1 + (self.sjw - 1) * 64

    @property
    def cnf2(self) -> int:
        """
        Bit Timing register used in Microchip based interfaces

        :rtype: int
        """
        return self.prop_seg - 2 + (self.phase_seg1 - 1) * 8 + 128

    @property
    def cnf3(self) -> int:
        """
        Bit Timing register used in Microchip based interfaces

        :rtype: int
        """
        return self.tseg2 - 1

    # @property
    # def canbtc(self) -> int:
    #     """
    #     Bit Timing register used in Texas Instruments based interfaces
    #
    #     :rtype: int
    #     """
    #     can_btc = (self.phase_seg2 - 1) & 0x7
    #     can_btc |= ((self.phase_seg1 + self.prop_seg - 1) & 0xF) << 3
    #     can_btc |= ((self.sjw - 1) & 0x3) << 8
    #     can_btc |= ((self.brp - 1) & 0xFF) << 16
    #
    #     return can_btc
    #
    # @property
    # def cxconr(self) -> int:
    #     """
    #     Bit Timing register used in Renesas based interfaces
    #
    #     :rtype: int
    #     """
    #     cxconr = self.brp - 1 + (self.prop_seg - 1) * 32
    #     cxconr += self.phase_seg1 - 2 + (self.tseg2 - 1) * 8 + (self.sjw - 1) * 64
    #     return cxconr * 256
    #
    # @property
    # def cibcr(self) -> int:
    #     """
    #     Bit Timing register used in Renesas based interfaces
    #
    #     :rtype: int
    #     """
    #
    #     def _bcr_tseg1(x):
    #         return (x & 0x0f) << 20
    #
    #     def _bcr_bpr(x):
    #         return (x & 0x3ff) << 8
    #
    #     def _bcr_sjw(x):
    #         return (x & 0x3) << 4
    #
    #     def _bcr_tseg2(x):
    #         return x & 0x07
    #
    #     bcr = (
    #             _bcr_tseg1(self.phase_seg1 + self.prop_seg - 1) |
    #             _bcr_bpr(self.brp - 1) |
    #             _bcr_sjw(self.sjw - 1) |
    #             _bcr_tseg2(self.phase_seg2 - 1)
    #     )
    #     return bcr << 8

    def __str__(self) -> str:
        res = [
            f"bitrate: {self.bitrate} bits/s",
            f"nominal bitrate: {self.nominal_bitrate} bits/s",
            f"bitrate error: {self.bitrate_error:.2f}%",
            f"cia compliant: {self.is_cis_sample_point}",
            f"sample point: {self.sample_point:.2f}%",
            f"nominal sample point: {self.nominal_sample_point:.2f}%",
            f"sample point error: {self.sample_point_error:.2f}%",
            f"number of samples: {self.number_of_samples}",
            f"bus length: {self.bus_length}",
            f"transceiver delay: {self.transceiver_delay}",
            f"FSYS: {self.fsys / 1000000}mHz",
            f"SYNC_SEG: {self.sync_seg}",
            f"TQ: {self.tq}",
            f"PROP_DELAY: {self.prop_delay}",
            f"PROP_SEG: {self.prop_seg}",
            f"PHASE_SEG1: {self.phase_seg1}",
            f"PHASE_SEG2: {self.phase_seg2}",
            f"NBT: {self.nbt}",
            f"BTQ: {self.btq}",
            f"TSEG1: {self.tseg1}",
            f"TSEG2: {self.tseg2}",
            f"BRP: {self.brp}",
            f"SJW: {self.sjw}",
            # f"BTR0: {self.btr0:02X}h",
            # f"BTR1: {self.btr1:02X}h",
            # f"CAN0BT: {self.can0bt:08X}h",
            # f"CANBR: {self.canbr:08X}h",
            # f"CANCTRL: {self.canctrl:08X}h",
            f"CNF1: {self.cnf1:02X}h",
            f"CNF2: {self.cnf2:02X}h",
            f"CNF3: {self.cnf3:02X}h",
            # f"CANBTC: {self.canbtc:08X}h",
            # f"CIBCR: {self.cibcr:08X}h",
            # f"CxCONR: {self.cxconr:08X}h"
        ]

        return "\n".join(res)

    def __repr__(self) -> str:
        kwargs = dict(
            fsys=self.fsys,
            bitrate=self.bitrate,
            nominal_bitrate=self.nominal_bitrate,
            bitrate_error="{0:.2f}".format(self.bitrate_error),
            sample_point="{0:.2f}".format(self.sample_point),
            nominal_sample_point="{0:.2f}".format(self.nominal_sample_point),
            sample_point_error="{0:.2f}".format(self.sample_point_error),
            number_of_samples=self.number_of_samples,
            bus_length=self.bus_length,
            transceiver_delay=self.transceiver_delay,
            sync_seg=self.sync_seg,
            tq=self.tq,
            prop_delay=self.prop_delay,
            prop_seg=self.prop_seg,
            phase_seg1=self.phase_seg1,
            phase_seg2=self.phase_seg2,
            nbt=self.nbt,
            tseg1=self.tseg1,
            tseg2=self.tseg2,
            brp=self.brp,
            sjw=self.sjw,
            # btr0='0x' + hex(self.btr0)[2:].upper().zfill(2),
            # btr1='0x' + hex(self.btr1)[2:].upper().zfill(2),
            # can0bt='0x' + hex(self.can0bt)[2:].upper().zfill(8),
            # canbr='0x' + hex(self.canbr)[2:].upper().zfill(8),
            # canctrl='0x' + hex(self.canctrl)[2:].upper().zfill(8),
            cnf1='0x' + hex(self.cnf1)[2:].upper().zfill(2),
            cnf2='0x' + hex(self.cnf2)[2:].upper().zfill(2),
            cnf3='0x' + hex(self.cnf3)[2:].upper().zfill(2),
            # canbtc='0x' + hex(self.canbtc)[2:].upper().zfill(8),
            # cibcr='0x' + hex(self.cibcr)[2:].upper().zfill(8),
            # cxconr='0x' + hex(self.cxconr)[2:].upper().zfill(8),
        )

        args = ", ".join(f"{key}={value}" for key, value in kwargs.items())
        return f"bitrate.Bitrate({args})"
