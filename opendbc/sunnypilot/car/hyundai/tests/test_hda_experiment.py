import unittest

from opendbc.can import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags, Buttons


def make_cp():
  CP = structs.CarParams()
  CP.carFingerprint = CAR.HYUNDAI_PALISADE_LX3
  CP.flags = (HyundaiFlags.CANFD | HyundaiFlags.CANFD_ANGLE_STEERING | HyundaiFlags.CANFD_LKA_STEER_MSG |
              HyundaiFlags.CANFD_LKA_STEER_MSG_ALT | HyundaiFlags.CANFD_ALT_BUTTONS).value
  return CP


def decode(dbc, name, bus, addr, dat):
  p = CANParser(dbc, [(name, 0)], bus)
  p.update([(0, [(addr, dat, bus)])])
  return dict(p.vl[name])


class TestHdaSuppressionExperiment(unittest.TestCase):
  def setUp(self):
    self.CP = make_cp()
    self.dbc = DBC[self.CP.carFingerprint][Bus.pt]
    self.packer = CANPacker(self.dbc)
    self.CAN = CanBus(self.CP, lka_steering=True)

  def _lkas(self, hide):
    msgs = hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, True, True, 0.5, 12.3, 2, hide_lfa_status=hide)
    self.assertEqual(len(msgs), 1)
    addr, dat, bus = msgs[0]
    self.assertEqual((addr, bus), (0x110, self.CAN.ACAN))
    return decode(self.dbc, "LKAS_ALT", bus, addr, dat)

  def test_experiment_a_hides_lfa_status_only(self):
    base = self._lkas(False)
    exp = self._lkas(True)
    self.assertEqual((base["LKA_RcgSta"], base["LKA_SysIndReq"]), (3, 2))
    self.assertEqual((exp["LKA_RcgSta"], exp["LKA_SysIndReq"]), (0, 1))
    # everything the MDPS steers to is untouched
    for k in ("LKAS_ANGLE_ACTIVE", "ADAS_StrAnglReqVal", "ADAS_ACIAnglTqRedcGainVal", "StrTqReqVal", "ActToiSta", "Damping_Gain"):
      self.assertEqual(base[k], exp[k], k)
    self.assertEqual(exp["LKAS_ANGLE_ACTIVE"], 2)
    self.assertAlmostEqual(exp["ADAS_StrAnglReqVal"], 12.3, places=1)

  def test_suppress_lfa_zeroes_byte_7_lane_fields_only(self):
    block = {f"BYTE{i}": 0x33 for i in range(3, 32)}
    block["COUNTER"] = 7
    addr, dat, bus = hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, block, True)
    self.assertEqual((addr, bus), (0x362, self.CAN.ACAN))
    self.assertEqual(dat[7], 0)
    for i in range(8, 32):
      self.assertEqual(dat[i], 0x33, i)

  def test_wheel_button_frames(self):
    for btn, expected in ((Buttons.RES_ACCEL, 0x01), (Buttons.CANCEL, 0x08)):
      addr, dat, bus = hyundaicanfd.create_wheel_buttons_alt(self.packer, self.CAN, 16, btn)
      self.assertEqual((addr, bus), (0x10B, self.CAN.ECAN))
      self.assertEqual(dat[2], 16)
      self.assertEqual(dat[10], expected, btn)
      self.assertEqual(sum(dat[3:10]) + sum(dat[11:]), 0)


if __name__ == "__main__":
  unittest.main()
