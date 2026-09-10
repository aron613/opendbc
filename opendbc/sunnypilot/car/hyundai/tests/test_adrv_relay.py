import unittest

from opendbc.sunnypilot.car.hyundai.adrv_relay import AdrvRelayWatchdog, PERSIST_FRAMES, HOLD_FRAMES


class TestAdrvRelayWatchdog(unittest.TestCase):
  def _run(self, wd, n, cmd, relay, cmd_active=True, relay_active=True):
    fault = False
    for _ in range(n):
      fault = wd.update(cmd[0], cmd[1], cmd_active, relay[0], relay[1], relay_active)
    return fault

  def test_faithful_relay_never_faults(self):
    wd = AdrvRelayWatchdog()
    # relay lag: angle off by up to 1.4 deg, gain off by one rate-limit step, as logged on the LX3 and the Sportage
    self.assertFalse(self._run(wd, 2000, (10.0, 0.85), (8.6, 0.836)))
    self.assertEqual(wd.mismatch_frames, 0)

  def test_substitution_faults_after_persist(self):
    wd = AdrvRelayWatchdog()
    # route 00000018 at t=105: we sent +8.6 / 0.28, the ADRV put +1.4 / 0.01 on the bus
    self.assertFalse(self._run(wd, PERSIST_FRAMES - 1, (8.6, 0.28), (1.4, 0.01)))
    self.assertTrue(self._run(wd, 1, (8.6, 0.28), (1.4, 0.01)))

  def test_gain_only_divergence_faults(self):
    wd = AdrvRelayWatchdog()
    self.assertTrue(self._run(wd, PERSIST_FRAMES, (3.0, 0.85), (3.0, 0.30)))

  def test_angle_only_divergence_faults(self):
    wd = AdrvRelayWatchdog()
    # the first frame sees a large command rate (from 0 to 10 deg) and is tolerated; the mismatch counts from frame 2
    self.assertFalse(self._run(wd, PERSIST_FRAMES, (10.0, 0.85), (5.0, 0.85)))
    self.assertTrue(self._run(wd, 1, (10.0, 0.85), (5.0, 0.85)))

  def test_fast_command_lag_tolerated(self):
    wd = AdrvRelayWatchdog()
    # 300 deg/s sweep with the relay one frame (3 deg) behind: tolerance grows with the command rate
    fault = False
    angle = 0.0
    for _ in range(200):
      prev = angle
      angle += 3.0
      fault = wd.update(angle, 0.85, True, prev - 1.0, 0.85, True)
    self.assertFalse(fault)

  def test_saturated_angle_not_compared(self):
    wd = AdrvRelayWatchdog()
    # LFA_ALT caps at 176.7 deg: a 190 deg command relayed as 176.7 is not a mismatch (gain still is)
    self.assertFalse(self._run(wd, 200, (190.0, 0.5), (176.7, 0.5)))
    self.assertTrue(self._run(wd, PERSIST_FRAMES, (190.0, 0.5), (176.7, 0.2)))

  def test_relay_inactive_while_commanding_faults(self):
    wd = AdrvRelayWatchdog()
    self.assertTrue(self._run(wd, PERSIST_FRAMES, (0.0, 0.85), (0.0, 0.85), relay_active=False))

  def test_inactive_command_never_faults_and_resets(self):
    wd = AdrvRelayWatchdog()
    self._run(wd, PERSIST_FRAMES - 1, (8.6, 0.28), (1.4, 0.01))
    self.assertFalse(self._run(wd, 1, (8.6, 0.28), (1.4, 0.01), cmd_active=False))
    self.assertEqual(wd.mismatch_frames, 0)
    self.assertFalse(self._run(wd, 500, (0.0, 0.0), (50.0, 0.0), cmd_active=False, relay_active=False))

  def test_hold_then_clear(self):
    wd = AdrvRelayWatchdog()
    self.assertTrue(self._run(wd, PERSIST_FRAMES, (8.6, 0.28), (1.4, 0.01)))
    # lateral dropped: we go inactive and the relay agrees, but the fault must hold for HOLD_FRAMES
    self.assertTrue(self._run(wd, HOLD_FRAMES - 1, (0.0, 0.0), (0.0, 0.0), cmd_active=False))
    self.assertFalse(self._run(wd, 1, (0.0, 0.0), (0.0, 0.0), cmd_active=False))

  def test_hold_extends_while_mismatch_continues(self):
    wd = AdrvRelayWatchdog()
    self._run(wd, PERSIST_FRAMES + HOLD_FRAMES, (8.6, 0.28), (1.4, 0.01))
    self.assertTrue(wd.fault)


if __name__ == "__main__":
  unittest.main()
