"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Watchdog for the ADAS ECU (ADRV) steering relay on LKA-steer CAN-FD cars.
#
# On these cars openpilot's LKAS_ALT (0x110) goes out on A-CAN and the ADRV re-transmits it to the MDPS as LFA_ALT
# (0xCB) on E-CAN. The MDPS only listens to 0xCB. On the 2026 Palisade (LX3) the ADRV stops relaying and substitutes its
# own request as soon as stock ACC engages (route 69fb86b6677ce882/00000018--30cfc70d81: 0xCB angle diverged from ours
# by 3-8 deg and its gain sat at 0.0-0.4 while we sent 0.85, and the MDPS followed 0xCB to 0.44 deg). While the relay
# is faithful the two differ by at most ~0.5 deg (1.4 deg peaks at high angle rates from the ~1 frame relay lag) and
# the gain matches to within one rate-limit step (0.014).
#
# Thresholds, from those two populations:
#   GAIN_TOL   0.10   > 7x the relay's worst-case step, < half of the smallest substitution gap seen (0.2)
#   ANGLE_TOL  2.0 deg plus 0.03 s of the command's own angle rate, so the relay lag on fast commands is not a mismatch
#   ANGLE_SAT  170 deg: LFA_ALT saturates at 176.7 deg, so larger commands cannot be compared
#   PERSIST    30 frames (0.3 s at 100 Hz): the substitution lasted 20+ s; relay glitches at cruise transitions on the
#              Sportage HEV reference route lasted a few frames
#   HOLD       500 frames (5 s): once tripped, lateral is dropped, which makes the relay agree again immediately; the
#              hold stops the fault from clearing and re-arming every 0.3 s
GAIN_TOL = 0.10
ANGLE_TOL_DEG = 2.0
ANGLE_RATE_TOL_S = 0.03
ANGLE_SAT_DEG = 170.0
PERSIST_FRAMES = 30
HOLD_FRAMES = 500


class AdrvRelayWatchdog:
  def __init__(self):
    self.mismatch_frames = 0
    self.hold_frames = 0
    self.prev_cmd_angle = 0.0
    self.mismatch = False

  @property
  def fault(self) -> bool:
    return self.hold_frames > 0

  def update(self, cmd_angle: float, cmd_gain: float, cmd_active: bool,
             relay_angle: float, relay_gain: float, relay_active: bool) -> bool:
    """One 100 Hz step. Returns True while the relay fault is asserted."""
    angle_rate = abs(cmd_angle - self.prev_cmd_angle) / 0.01
    self.prev_cmd_angle = cmd_angle

    if cmd_active:
      gain_mismatch = abs(cmd_gain - relay_gain) > GAIN_TOL
      angle_tol = ANGLE_TOL_DEG + ANGLE_RATE_TOL_S * angle_rate
      angle_mismatch = abs(cmd_angle) < ANGLE_SAT_DEG and abs(cmd_angle - relay_angle) > angle_tol
      self.mismatch = (not relay_active) or gain_mismatch or angle_mismatch
    else:
      self.mismatch = False

    self.mismatch_frames = self.mismatch_frames + 1 if self.mismatch else 0
    if self.mismatch_frames >= PERSIST_FRAMES:
      self.hold_frames = HOLD_FRAMES
    elif self.hold_frames > 0:
      self.hold_frames -= 1

    return self.fault
