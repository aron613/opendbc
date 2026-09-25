import unittest

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai.stock_lfa import StockLfaDisabler, PULSE_FRAMES, RECHECK_FRAMES, MAX_ATTEMPTS


class TestStockLfaDisabler(unittest.TestCase):
  def _run(self, d, n, stock_lfa_on=True, want_lateral=True):
    return [d.update(stock_lfa_on, want_lateral) for _ in range(n)]

  def test_idle_when_stock_lfa_off(self):
    d = StockLfaDisabler()
    self.assertFalse(any(self._run(d, 500, stock_lfa_on=False)))

  def test_idle_when_lateral_not_wanted(self):
    # this is also the HDA-road gate window: the stock system is the one steering, do not switch it off
    d = StockLfaDisabler()
    self.assertFalse(any(self._run(d, 500, want_lateral=False)))

  def test_pulse_shape_matches_camera(self):
    d = StockLfaDisabler()
    out = self._run(d, PULSE_FRAMES + RECHECK_FRAMES)
    self.assertEqual(out[:PULSE_FRAMES], [True] * PULSE_FRAMES)
    self.assertFalse(any(out[PULSE_FRAMES:]))

  def test_retries_after_recheck_then_stops(self):
    d = StockLfaDisabler()
    out = self._run(d, (PULSE_FRAMES + RECHECK_FRAMES) * (MAX_ATTEMPTS + 2))
    # one pulse per attempt, no more
    edges = [i for i, v in enumerate(out) if v and not (i and out[i - 1])]
    self.assertEqual(len(edges), MAX_ATTEMPTS)
    self.assertEqual(sum(out), PULSE_FRAMES * MAX_ATTEMPTS)
    self.assertEqual(d.attempts, MAX_ATTEMPTS)
    # and it backs off for good while the icon stays on
    self.assertFalse(any(self._run(d, 2000)))

  def test_icon_clearing_rearms_for_next_time(self):
    d = StockLfaDisabler()
    self._run(d, PULSE_FRAMES)          # one pulse sent
    self._run(d, 10, stock_lfa_on=False)  # the ADRV switched it off
    self.assertEqual(d.attempts, 0)
    # it comes back on later: a fresh set of attempts, starting immediately
    self.assertTrue(d.update(True, True))

  def test_lateral_no_longer_wanted_rearms(self):
    d = StockLfaDisabler()
    self._run(d, (PULSE_FRAMES + RECHECK_FRAMES) * MAX_ATTEMPTS)
    self.assertEqual(d.attempts, MAX_ATTEMPTS)
    self._run(d, 1, want_lateral=False)
    self.assertEqual(d.attempts, 0)
    self.assertTrue(d.update(True, True))

  def test_pulse_is_not_cut_short_by_the_icon_clearing_mid_pulse(self):
    # the ADRV needs the whole pulse; it only reacts ~0.16 s later, so the icon cannot clear mid-pulse in practice,
    # but if the signal glitches we stop pressing rather than holding the bit
    d = StockLfaDisabler()
    self.assertTrue(d.update(True, True))
    self.assertFalse(d.update(False, True))
    self.assertEqual(d.pulse_frames, 0)


class TestStockLfaOnTheCar(unittest.TestCase):
  """the whole chain: LFA_ICON from the car -> pulse -> LFA_BUTTON bit in the LKAS_ALT frame we transmit"""
  def setUp(self):
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR, HyundaiFlags
    CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_PALISADE_LX3)
    # get_non_essential_params skips the fingerprint-derived bus flags; the car puts LKAS_ALT (0x110) on A-CAN
    CP.flags |= (HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT).value
    self.CI = CarInterface(CP, CarInterface.get_non_essential_params_sp(CP, CAR.HYUNDAI_PALISADE_LX3))

  def _step(self, mads_enabled, stock_lfa_icon, hda_road_active, n=1):
    lfa_button = []
    for _ in range(n):
      CC_SP = structs.CarControlSP()
      CC_SP.mads.enabled = mads_enabled
      self.CI.update([])  # no CAN data, so set the two car-side inputs by hand afterwards
      self.CI.CS.stock_lfa_icon = stock_lfa_icon
      self.CI.CS.hda_road_active = hda_road_active
      _, can_sends = self.CI.apply(structs.CarControl().as_reader(), CC_SP, 0)
      lkas = [m for m in can_sends if m[0] == 0x110]
      self.assertEqual(len(lkas), 1)
      lfa_button.append(bool(lkas[0][1][7] & 1))
    return lfa_button

  def test_pulses_only_when_the_car_lane_centering_is_on_and_we_want_the_wheel(self):
    self.assertFalse(any(self._step(True, 0, False, n=20)))           # car's lane centering off
    self.assertFalse(any(self._step(False, 2, False, n=20)))          # MADS not armed
    self.assertFalse(any(self._step(True, 2, True, n=20)))            # HDA-road gate: leave the stock system alone
    out = self._step(True, 2, False, n=PULSE_FRAMES + 5)
    self.assertEqual(out, [True] * PULSE_FRAMES + [False] * 5)


if __name__ == "__main__":
  unittest.main()
