import unittest

from opendbc.car import structs
from opendbc.sunnypilot.car.hyundai.stock_lfa import (StockLfaDisabler, PULSE_FRAMES, RECHECK_FRAMES, MAX_ATTEMPTS,
                                                      ICON_CONFIRM_FRAMES, ICON_WAIT_FRAMES, LFA_ICON_TRANSITION)

ACTIVE = 2  # LFA_ICON: the car's own lane centering is steering
STANDBY = 1
OFF = 0


class TestStockLfaDisabler(unittest.TestCase):
  def _run(self, d, n, icon=ACTIVE, hda_window=False, want_lateral=True, pressed=False):
    return [d.update(icon, hda_window, want_lateral, pressed) for _ in range(n)]

  @staticmethod
  def _pulses(out):
    """[[first frame index, length], ...] of each run of True"""
    runs = []
    for i, v in enumerate(out):
      if v and (i == 0 or not out[i - 1]):
        runs.append([i, 1])
      elif v:
        runs[-1][1] += 1
    return runs

  def test_idle_when_stock_lfa_off(self):
    d = StockLfaDisabler()
    self.assertFalse(any(self._run(d, 500, icon=OFF)))

  def test_idle_when_lateral_not_wanted(self):
    d = StockLfaDisabler()
    self.assertFalse(any(self._run(d, 500, want_lateral=False)))

  def test_presses_inside_an_hda_window_without_being_asked(self):
    d = StockLfaDisabler()
    out = self._run(d, ICON_CONFIRM_FRAMES + PULSE_FRAMES, hda_window=True)
    self.assertEqual(self._pulses(out), [[ICON_CONFIRM_FRAMES - 1, PULSE_FRAMES]])
    self.assertTrue(d.auto_request)
    self.assertFalse(d.driver_request)
    self.assertTrue(d.requested)
    # it releases the gate the same way once the car lets go
    self._run(d, 3, icon=OFF, hda_window=True)
    self.assertTrue(d.take_back)
    # and it never presses while the icon reads off (rule 1)
    self.assertFalse(any(self._run(d, 400, icon=OFF, hda_window=True)))

  def test_suppress_failure_is_reported(self):
    d = StockLfaDisabler()
    self._run(d, ICON_CONFIRM_FRAMES + (PULSE_FRAMES + RECHECK_FRAMES) * MAX_ATTEMPTS + 1, hda_window=True)
    self.assertTrue(d.failed)
    self.assertTrue(d.auto_request)
    self.assertFalse(d.driver_request)

  def test_driver_press_is_distinguished_from_the_automatic_request(self):
    d = StockLfaDisabler()
    self._run(d, 1, hda_window=True, pressed=True)
    self.assertTrue(d.driver_request)
    self.assertTrue(d.auto_request)

  def test_latch_waits_for_the_icon_to_come_on_at_the_hda_arm(self):
    # route 0000009b at 1242.7 s: the ADAS ECU lights the HDA icon 0.05 s before it switches its lane centering on, and
    # releasing the gate in that gap made it flutter on, off, on inside 60 ms
    d = StockLfaDisabler()
    for _ in range(5):  # the window is open, the icon has not come on yet
      d.update(OFF, True, True, False)
      self.assertFalse(d.take_back)
    d.update(ACTIVE, True, True, False)
    self.assertFalse(d.take_back)
    self.assertTrue(d.icon_was_on)
    # now that it has been on, its going off is the real success
    d.update(OFF, True, True, False)
    self.assertTrue(d.take_back)

  def test_latch_falls_back_when_the_icon_never_comes_on(self):
    # nothing to suppress, so do not leave lateral paused with nobody steering
    d = StockLfaDisabler()
    out = [d.update(OFF, True, True, False) or d.take_back for _ in range(ICON_WAIT_FRAMES + 2)]
    self.assertFalse(any(out[:ICON_WAIT_FRAMES]))
    self.assertTrue(d.take_back)

  def test_latch_resets_when_the_window_closes(self):
    d = StockLfaDisabler()
    d.update(ACTIVE, True, True, False)
    self.assertTrue(d.icon_was_on)
    d.update(ACTIVE, False, True, False)
    self.assertFalse(d.icon_was_on)
    self.assertEqual(d.request_frames, 0)

  def test_driver_request_releases_the_gate_once_the_stock_system_lets_go(self):
    d = StockLfaDisabler()
    self._run(d, 1, hda_window=True, pressed=True)
    self.assertFalse(d.take_back)
    # the ADRV switches its lane centering off while still showing HDA on this road
    self._run(d, 5, icon=OFF, hda_window=True)
    self.assertTrue(d.take_back)
    self.assertFalse(d.requested)
    # ...and the latch drops when the HDA window ends
    self._run(d, 1, icon=OFF, hda_window=False)
    self.assertFalse(d.take_back)
    self.assertFalse(d.driver_request)

  def test_transition_state_holds_instead_of_pulsing_again(self):
    # LFA_ICON 3 for ~1 s is the ADRV on its way out; another press there toggles it back on
    d = StockLfaDisabler()
    self._run(d, ICON_CONFIRM_FRAMES + PULSE_FRAMES + RECHECK_FRAMES - 1)
    self.assertEqual(d.attempts, 1)
    self.assertFalse(any(self._run(d, 400, icon=LFA_ICON_TRANSITION)))
    self.assertEqual(d.attempts, 1)
    # once it settles back on the active state, the next attempt goes out
    self.assertTrue(d.update(ACTIVE, False, True, False))
    self.assertEqual(d.attempts, 2)

  def test_icon_is_confirmed_over_two_messages_before_the_first_press(self):
    # rule 1: the icon comes from a 20 Hz message and the button is a toggle, so a stale or glitched "on" must not
    # produce a press that switches the car's lane centering ON
    d = StockLfaDisabler()
    out = self._run(d, ICON_CONFIRM_FRAMES + PULSE_FRAMES)
    self.assertEqual(self._pulses(out), [[ICON_CONFIRM_FRAMES - 1, PULSE_FRAMES]])

  def test_pulse_shape_and_recheck_interval(self):
    d = StockLfaDisabler()
    out = self._run(d, ICON_CONFIRM_FRAMES + PULSE_FRAMES + RECHECK_FRAMES - 1)
    self.assertEqual(self._pulses(out), [[ICON_CONFIRM_FRAMES - 1, PULSE_FRAMES]])
    self.assertTrue(d.update(ACTIVE, False, True, False))  # second attempt, 1.53 s after the first

  def test_backs_off_after_max_attempts_and_reports_failure(self):
    d = StockLfaDisabler()
    out = self._run(d, ICON_CONFIRM_FRAMES + (PULSE_FRAMES + RECHECK_FRAMES) * (MAX_ATTEMPTS + 2))
    runs = self._pulses(out)
    self.assertEqual(len(runs), MAX_ATTEMPTS)
    self.assertEqual([r[1] for r in runs], [PULSE_FRAMES] * MAX_ATTEMPTS)
    self.assertTrue(d.failed)
    self.assertFalse(any(self._run(d, 2000)))

  def test_icon_clearing_rearms_and_clears_the_failure(self):
    d = StockLfaDisabler()
    self._run(d, ICON_CONFIRM_FRAMES + (PULSE_FRAMES + RECHECK_FRAMES) * MAX_ATTEMPTS + 1)
    self.assertTrue(d.failed)
    self._run(d, 5, icon=OFF)
    self.assertFalse(d.failed)
    self.assertEqual(d.attempts, 0)
    # it comes back on later: a fresh set of attempts, after the icon is confirmed again
    out = self._run(d, ICON_CONFIRM_FRAMES, icon=STANDBY)
    self.assertEqual(self._pulses(out), [[ICON_CONFIRM_FRAMES - 1, 1]])

  def test_a_new_press_rearms_a_failed_request(self):
    d = StockLfaDisabler()
    self._run(d, 1, hda_window=True, pressed=True)
    self._run(d, ICON_CONFIRM_FRAMES + (PULSE_FRAMES + RECHECK_FRAMES) * MAX_ATTEMPTS + 1, hda_window=True)
    self.assertTrue(d.failed)
    self.assertTrue(d.update(ACTIVE, True, True, True))
    self.assertFalse(d.failed)
    self.assertEqual(d.attempts, 1)

  def test_pulse_finishes_when_the_intent_drops_mid_pulse(self):
    # route 0000007a at 174.54 s: MADS went off one frame into a pulse and only a single frame went out
    d = StockLfaDisabler()
    self._run(d, ICON_CONFIRM_FRAMES - 1)
    self.assertTrue(d.update(ACTIVE, False, True, False))
    self.assertEqual(self._run(d, PULSE_FRAMES - 1, want_lateral=False), [True] * (PULSE_FRAMES - 1))
    self.assertFalse(d.update(ACTIVE, False, False, False))

  def test_pulse_stops_if_the_icon_clears_mid_pulse(self):
    d = StockLfaDisabler()
    self._run(d, ICON_CONFIRM_FRAMES - 1)
    self.assertTrue(d.update(ACTIVE, False, True, False))
    self.assertFalse(d.update(OFF, False, True, False))
    self.assertEqual(d.pulse_frames, 0)


class TestStockLfaOnTheCar(unittest.TestCase):
  """the whole chain: LFA_ICON and the driver's button from the car -> LFA_BUTTON bit in the LKAS_ALT frame we send"""
  def setUp(self):
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR, HyundaiFlags
    CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_PALISADE_LX3)
    # get_non_essential_params skips the fingerprint-derived bus flags; the car puts LKAS_ALT (0x110) on A-CAN
    CP.flags |= (HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT).value
    self.CI = CarInterface(CP, CarInterface.get_non_essential_params_sp(CP, CAR.HYUNDAI_PALISADE_LX3))

  def _step(self, mads_enabled, lfa_icon, hda_window, n=1, pressed=False):
    lfa_button = []
    for _ in range(n):
      CC_SP = structs.CarControlSP()
      CC_SP.mads.enabled = mads_enabled
      self.CI.update([])  # no CAN data, so set the car-side inputs by hand afterwards
      self.CI.CS.stock_lfa_icon = lfa_icon
      self.CI.CS.stock_hda_window = hda_window
      if pressed:
        be = structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.lkas, pressed=True)
        self.CI.CS.out.buttonEvents = [be]
      _, can_sends = self.CI.apply(structs.CarControl().as_reader(), CC_SP, 0)
      lkas = [m for m in can_sends if m[0] == 0x110]
      self.assertEqual(len(lkas), 1)
      lfa_button.append(bool(lkas[0][1][7] & 1))
    return lfa_button

  def test_pulses_only_when_the_car_lane_centering_is_on_and_we_want_the_wheel(self):
    self.assertFalse(any(self._step(True, OFF, False, n=20)))      # car's lane centering off
    self.assertFalse(any(self._step(False, ACTIVE, False, n=20)))  # MADS not armed
    out = self._step(True, ACTIVE, False, n=PULSE_FRAMES + 5)
    self.assertEqual(out, [True] * PULSE_FRAMES + [False] * 5)

  def test_pulses_inside_an_hda_window_with_no_toggle_and_no_press(self):
    out = self._step(True, ACTIVE, True, n=ICON_CONFIRM_FRAMES + PULSE_FRAMES)
    first = ICON_CONFIRM_FRAMES - 1
    self.assertEqual(out[:first], [False] * first)
    self.assertEqual(out[first:first + PULSE_FRAMES], [True] * PULSE_FRAMES)
    self.assertTrue(self.CI.CS.stock_lfa_auto_requested)
    self.assertFalse(self.CI.CS.stock_lfa_requested)
    self._step(True, OFF, True, n=2)
    self.assertTrue(self.CI.CS.stock_lfa_take_back)

  def test_the_gate_does_not_flutter_at_an_hda_arm(self):
    # replay of route 0000009b at 1242.7 s: window opens, icon comes on 5 frames later
    for _ in range(5):
      self._step(True, OFF, True)
      self.assertFalse(self.CI.CS.stock_lfa_take_back)
    self._step(True, ACTIVE, True, n=10)
    self.assertFalse(self.CI.CS.stock_lfa_take_back)
    self._step(True, OFF, True, n=2)
    self.assertTrue(self.CI.CS.stock_lfa_take_back)

  def test_driver_press_in_an_hda_window_is_reported_as_the_drivers(self):
    self._step(True, ACTIVE, True, n=ICON_CONFIRM_FRAMES + PULSE_FRAMES)  # the automatic request has run
    self.assertTrue(self.CI.CS.stock_lfa_auto_requested)
    self.assertFalse(self.CI.CS.stock_lfa_requested)
    # the driver asks as well: a fresh pulse, and the alert becomes theirs
    self.assertTrue(self._step(True, ACTIVE, True, n=1, pressed=True)[0])
    self.assertEqual(self._step(True, ACTIVE, True, n=PULSE_FRAMES - 1), [True] * (PULSE_FRAMES - 1))
    self.assertTrue(self.CI.CS.stock_lfa_requested)
    self.assertFalse(self.CI.CS.stock_lfa_auto_requested)
    self.assertFalse(self.CI.CS.stock_lfa_take_back)
    self._step(True, OFF, True, n=2)
    self.assertTrue(self.CI.CS.stock_lfa_take_back)


if __name__ == "__main__":
  unittest.main()
