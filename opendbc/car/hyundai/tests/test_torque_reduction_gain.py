import unittest

from opendbc.car.hyundai.carcontroller import compute_torque_reduction_gain
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.values import PLATFORMS


def converged_gain(torque, v_ego, fast_handoff):
  # run the rate limiter until it settles on the target for this torque/speed
  gain = 0.6
  for _ in range(400):
    gain = compute_torque_reduction_gain(torque, v_ego, True, gain, fast_handoff=fast_handoff)
  return gain


class TestTorqueReductionGain(unittest.TestCase):
  V_23MPH = 23 / 2.237

  def test_default_curve_unchanged(self):
    # regression points for every other angle-steering car (targets of the stock table at 23 mph)
    for torque, expected in ((0, 0.85), (150, 0.596), (250, 0.596), (300, 0.54), (400, 0.38), (525, 0.18), (700, 0.176)):
      self.assertAlmostEqual(converged_gain(torque, self.V_23MPH, False), expected, delta=0.02, msg=f"torque {torque}")

  def test_fast_handoff_reaches_floor_by_300_to_400(self):
    floor = converged_gain(1000, self.V_23MPH, True)
    self.assertAlmostEqual(floor, 0.10, delta=0.01)
    for torque in (350, 400, 470):
      self.assertAlmostEqual(converged_gain(torque, self.V_23MPH, True), floor, delta=0.01, msg=f"torque {torque}")
    self.assertLess(converged_gain(300, self.V_23MPH, True), 0.35)

  def test_fast_handoff_keeps_nudge_region(self):
    # below the 175-unit steerOverride threshold the two curves are identical: resting hands keep the shelf
    for v in (2.0, 5.0, self.V_23MPH, 20.0):
      for torque in (0, 60, 100, 125, 150, 170):
        self.assertAlmostEqual(converged_gain(torque, v, True), converged_gain(torque, v, False), delta=0.005,
                               msg=f"v {v} torque {torque}")

  def test_fast_handoff_monotonic_and_bounded(self):
    for v in (0.0, 2.0, 5.0, self.V_23MPH, 20.0, 30.0):
      prev = 2.0
      for torque in range(0, 800, 25):
        g = converged_gain(torque, v, True)
        self.assertGreaterEqual(g, 0.0)
        self.assertLessEqual(g, 1.0)
        self.assertLessEqual(g, prev + 1e-9, msg=f"v {v} torque {torque}")
        self.assertAlmostEqual(g / 0.004, round(g / 0.004), delta=1e-6)  # raw 0-250 for the panda
        prev = g

  def test_fast_handoff_drop_time(self):
    # a 400-unit push from the shelf should reach the floor within the rate limit's ~0.3 s
    gain = 0.6
    frames = 0
    while gain > 0.19 and frames < 100:
      gain = compute_torque_reduction_gain(400, self.V_23MPH, True, gain, fast_handoff=True)
      frames += 1
    self.assertLessEqual(frames, 35)

  def test_fast_handoff_floor_is_stock_low_speed_floor(self):
    # 0.10 at every speed; stock rises to 0.30 by 22 m/s
    for v in (0.0, 2.0, 5.0, self.V_23MPH, 20.0, 30.0):
      self.assertAlmostEqual(converged_gain(1000, v, True), 0.10, delta=0.01, msg=f"v {v}")
    self.assertAlmostEqual(converged_gain(1000, 22.0, False), 0.30, delta=0.01)
    self.assertAlmostEqual(converged_gain(1000, 2.0, False), 0.10, delta=0.01)

  def test_inactive_is_zero(self):
    self.assertEqual(compute_torque_reduction_gain(400, self.V_23MPH, False, 0.0, fast_handoff=True), 0.0)

  def test_flag_only_on_lx3(self):
    for platform in PLATFORMS:
      if platform.startswith(("HYUNDAI", "KIA", "GENESIS")):
        has = bool(PLATFORMS[platform].config.flags & HyundaiFlags.CANFD_FAST_OVERRIDE_HANDOFF)
        self.assertEqual(has, platform == CAR.HYUNDAI_PALISADE_LX3, platform)


if __name__ == "__main__":
  unittest.main()
