import unittest

from opendbc.sunnypilot.car.hyundai.adrv_relay import (AdrvRelayWatchdog, PERSIST_FRAMES, HOLD_FRAMES, PRESSED_GRACE_FRAMES,
                                                       RELAY_ANGLE_MAX, RELAY_SLEW_DEG_S, DT)


class FakeRelay:
  """what a faithful ADRV does: clamp, slew, one frame late"""
  def __init__(self):
    self.angle = 0.0

  def step(self, cmd_angle):
    target = max(-RELAY_ANGLE_MAX, min(RELAY_ANGLE_MAX, cmd_angle))
    step = RELAY_SLEW_DEG_S * DT
    self.angle += max(-step, min(step, target - self.angle))
    return self.angle


class TestAdrvRelayWatchdog(unittest.TestCase):
  def _run(self, wd, n, cmd, relay, cmd_active=True, relay_active=True, pressed=False):
    fault = False
    for _ in range(n):
      fault = wd.update(cmd[0], cmd[1], cmd_active, relay[0], relay[1], relay_active, pressed)
    return fault

  def _settle(self, wd, angle=0.0, gain=0.85):
    # engage from rest with the relay already at the command, so later steps only test the intended effect
    self._run(wd, 5, (angle, 0.0), (angle, 0.0), cmd_active=False, relay_active=False)
    self._run(wd, 100, (angle, gain), (angle, gain))
    self.assertEqual(wd.mismatch_frames, 0)
    self.assertFalse(wd.fault)

  def test_faithful_relay_never_faults(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd, 10.0)
    # steady lag of up to 1.4 deg and one rate-limit step of gain, as logged
    self.assertFalse(self._run(wd, 2000, (10.0, 0.85), (8.6, 0.836)))
    self.assertEqual(wd.mismatch_frames, 0)

  def test_substitution_faults_after_persist(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd, 8.6, 0.28)
    # route 00000018 at t=105: we sent +8.6 / 0.28, the ADRV put +1.4 / 0.01 on the bus
    self.assertFalse(self._run(wd, PERSIST_FRAMES - 1, (8.6, 0.28), (1.4, 0.01)))
    self.assertTrue(self._run(wd, 1, (8.6, 0.28), (1.4, 0.01)))

  def test_gain_only_divergence_faults(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd, 3.0)
    self.assertTrue(self._run(wd, PERSIST_FRAMES, (3.0, 0.85), (3.0, 0.30)))

  def test_angle_only_divergence_faults(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd, 10.0)
    self.assertTrue(self._run(wd, PERSIST_FRAMES, (10.0, 0.85), (5.0, 0.85)))

  def test_slew_limited_relay_tolerated(self):
    # route 0000001b 143.1-143.6 s: our command unwinds at 250-500 deg/s from beyond the clamp while the relay slews
    wd = AdrvRelayWatchdog()
    relay = FakeRelay()
    self._settle(wd, -100.0)
    relay.angle = -100.0
    cmd = -100.0
    fault = False
    for _ in range(80):   # out to -340 deg at 300 deg/s: relay clamps at -176.7
      cmd -= 3.0
      fault = wd.update(cmd, 0.10, True, relay.step(cmd), 0.10, True) or fault
    for _ in range(140):  # back to +80 deg at 300 deg/s: relay slews behind
      cmd += 3.0
      fault = wd.update(cmd, 0.10, True, relay.step(cmd), 0.10, True) or fault
    self.assertFalse(fault)
    self.assertEqual(wd.mismatch_frames, 0)

  def test_reengage_sprint_tolerated(self):
    # route 0000001b 257.0 s: on re-engage the relay rests at its idle value and slews toward a command 200 deg away
    wd = AdrvRelayWatchdog()
    relay = FakeRelay()
    relay.angle = -120.0
    self._run(wd, 50, (0.0, 0.0), (-120.0, 0.0), cmd_active=False, relay_active=False)
    cmd = -319.0
    fault = False
    for _ in range(100):
      cmd = min(cmd + 5.0, -76.0)  # 500 deg/s sprint
      fault = wd.update(cmd, 0.10, True, relay.step(cmd), 0.10, True) or fault
    self.assertFalse(fault)

  def test_pressed_skips_and_grace(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd, 8.6, 0.28)
    # a real substitution while the driver overrides is not counted...
    self.assertFalse(self._run(wd, 300, (8.6, 0.28), (1.4, 0.01), pressed=True))
    self.assertEqual(wd.mismatch_frames, 0)
    # ...nor during the grace after release...
    self.assertFalse(self._run(wd, PRESSED_GRACE_FRAMES, (8.6, 0.28), (1.4, 0.01)))
    self.assertEqual(wd.mismatch_frames, 0)
    # ...but it is caught PERSIST_FRAMES after the grace ends
    self.assertFalse(self._run(wd, PERSIST_FRAMES - 1, (8.6, 0.28), (1.4, 0.01)))
    self.assertTrue(self._run(wd, 1, (8.6, 0.28), (1.4, 0.01)))

  def test_relay_inactive_while_commanding_faults(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd)
    self.assertTrue(self._run(wd, PERSIST_FRAMES, (0.0, 0.85), (0.0, 0.85), relay_active=False))

  def test_inactive_command_never_faults_and_resets(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd, 8.6, 0.28)
    self._run(wd, PERSIST_FRAMES - 1, (8.6, 0.28), (1.4, 0.01))
    self.assertFalse(self._run(wd, 1, (8.6, 0.28), (1.4, 0.01), cmd_active=False))
    self.assertEqual(wd.mismatch_frames, 0)
    self.assertFalse(self._run(wd, 500, (0.0, 0.0), (50.0, 0.0), cmd_active=False, relay_active=False))

  def test_hold_then_clear(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd, 8.6, 0.28)
    self.assertTrue(self._run(wd, PERSIST_FRAMES, (8.6, 0.28), (1.4, 0.01)))
    self.assertTrue(self._run(wd, HOLD_FRAMES - 1, (0.0, 0.0), (0.0, 0.0), cmd_active=False))
    self.assertFalse(self._run(wd, 1, (0.0, 0.0), (0.0, 0.0), cmd_active=False))

  def test_hold_extends_while_mismatch_continues(self):
    wd = AdrvRelayWatchdog()
    self._settle(wd, 8.6, 0.28)
    self._run(wd, PERSIST_FRAMES + HOLD_FRAMES, (8.6, 0.28), (1.4, 0.01))
    self.assertTrue(wd.fault)


if __name__ == "__main__":
  unittest.main()
