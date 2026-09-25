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
# by 3-8 deg and its gain sat at 0.0-0.4 while we sent 0.85, and the MDPS followed 0xCB to 0.44 deg).
#
# The relay is not a byte copy: the ADRV clamps the angle to +/-176.7 deg and slews it at ~200 deg/s (route
# 69fb86b6677ce882/0000001b--e709e280e9, 143.2-143.6 s and 257.3-257.6 s: 0xCB unwound at 210 and 200 deg/s while our
# command moved at 250-500 deg/s). So 0xCB is compared against a model of the relay, not against the raw command:
#   model = clamp(cmd, +/-RELAY_ANGLE_MAX) followed by a RELAY_SLEW_DEG_S rate limit, tracking 0xCB while inactive
# With that model the faithful relay stays within ~0.5 deg (1.4 deg peaks).
#
# While the driver overrides (steeringPressed) the MDPS follows the driver whatever 0xCB says, and our command can run
# far from the wheel; a mismatch there carries no information, so the comparison is skipped, plus PRESSED_GRACE_FRAMES
# after release for the relay to catch up. Both false trips on route 0000001b were inside or right after an override.
#
# Thresholds:
#   GAIN_TOL   0.10   > 7x the relay's worst-case step (0.014), < half of the smallest substitution gap seen (0.2)
#   ANGLE_TOL  2.0 deg plus 0.03 s of the model's own rate (one to three frames of relay latency)
#   PERSIST    30 frames (0.3 s at 100 Hz): the substitution lasted 20+ s
#   HOLD       500 frames (5 s): once tripped, lateral is dropped, which makes the relay agree again immediately; the
#              hold stops the fault from clearing and re-arming every 0.3 s. The hold only runs down once the relay is
#              genuinely idle (see IDLE_GAIN_TOL): on route 69fb86b6677ce882/0000005e--ef39be0c84 the ADRV held 0xCB
#              active with its own 0.4-0.6 gain for minutes after our lateral dropped, and a plain 5 s release re-armed
#              openpilot into a fight it cannot win 12 times, each time with a full-screen takeover alert.
GAIN_TOL = 0.10
ANGLE_TOL_DEG = 2.0
ANGLE_RATE_TOL_S = 0.03
RELAY_ANGLE_MAX = 176.7
RELAY_SLEW_DEG_S = 250.0
IDLE_GAIN_TOL = 0.05  # 0xCB rests at gain 0.0 when the ADRV is not steering; well below the 0.2 smallest seen in use
PRESSED_GRACE_FRAMES = 50
PERSIST_FRAMES = 30
HOLD_FRAMES = 500
DT = 0.01


class AdrvRelayWatchdog:
  def __init__(self):
    self.mismatch_frames = 0
    self.hold_frames = 0
    self.grace_frames = 0
    self.model_angle = 0.0
    self.mismatch = False
    self.relay_idle = True

  @property
  def fault(self) -> bool:
    return self.hold_frames > 0

  def update(self, cmd_angle: float, cmd_gain: float, cmd_active: bool,
             relay_angle: float, relay_gain: float, relay_active: bool, steering_pressed: bool = False) -> bool:
    """One 100 Hz step. Returns True while the relay fault is asserted."""
    # relay model: what a faithful ADRV would be putting on 0xCB right now
    if cmd_active:
      target = max(-RELAY_ANGLE_MAX, min(RELAY_ANGLE_MAX, cmd_angle))
      step = RELAY_SLEW_DEG_S * DT
      prev = self.model_angle
      self.model_angle = prev + max(-step, min(step, target - prev))
      model_rate = abs(self.model_angle - prev) / DT
    else:
      # while we are inactive the relay carries whatever the ADRV rests at; start the model from there on re-engage
      self.model_angle = relay_angle
      model_rate = 0.0

    in_override = steering_pressed or self.grace_frames > 0
    if steering_pressed:
      self.grace_frames = PRESSED_GRACE_FRAMES
    elif self.grace_frames > 0:
      self.grace_frames -= 1

    if cmd_active and not in_override:
      gain_mismatch = abs(cmd_gain - relay_gain) > GAIN_TOL
      angle_mismatch = abs(self.model_angle - relay_angle) > ANGLE_TOL_DEG + ANGLE_RATE_TOL_S * model_rate
      self.mismatch = (not relay_active) or gain_mismatch or angle_mismatch
    else:
      self.mismatch = False

    # the relay has let go when it is inactive, or active but carrying no torque-reduction gain of its own
    self.relay_idle = (not relay_active) or (relay_gain <= IDLE_GAIN_TOL)

    self.mismatch_frames = self.mismatch_frames + 1 if self.mismatch else 0
    if self.mismatch_frames >= PERSIST_FRAMES:
      self.hold_frames = HOLD_FRAMES
    elif self.hold_frames > 0 and self.relay_idle:
      # only let the fault clear once whoever took the relay has released it, so lateral re-arms into an idle relay
      # instead of into the stock system's own request
      self.hold_frames -= 1

    return self.fault
