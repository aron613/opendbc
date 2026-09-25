"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Switches the car's own lane centering off on LKA-steer CAN-FD cars (2026 Palisade LX3).
#
# Mechanism, from route 69fb86b6677ce882/00000008--c7bfd877d8, where the panda was still in elm327 and the camera's
# LKAS_ALT (0x110) reached the ADAS ECU: each LFA button press on WHEEL_BUTTONS_ALT (0x10B) made the camera pulse
# LKAS_ALT.LFA_BUTTON for 2-4 frames, and ~0.16 s later the ADRV toggled LFA_ICON in LFAHDA_CLUSTER (0x1E0). Four
# presses, four toggles. That bit is the only input that turns the car's lane centering on or off.
#
# With openpilot driving, panda statically blocks the camera's 0x110 (it is in the TX list with check_relay) and our
# own 0x110 has always sent LFA_BUTTON = 0, so the LFA button cannot reach the ADRV at all. On route
# 69fb86b6677ce882/0000005e--ef39be0c84 the ADRV switched its own lane centering on when it armed HDA (LFA_ICON 0 -> 2
# at 977.63 s) and it stayed on for the remaining 12 minutes: ten LFA presses, fourteen cruise-main presses and an ACC
# main-off did nothing, 0xCB kept the ADRV's own angle and 0.4-0.6 gain, and only ignition-off cleared it.
#
# So we pulse the bit ourselves, in the camera's shape, whenever the car's lane centering is on and openpilot wants the
# wheel. It is a toggle, so each pulse is followed by a re-check of LFA_ICON well past the 0.16 s the ADRV took to
# respond, and there is a hard cap on attempts so a car that ignores the bit is left alone instead of being toggled
# back and forth.
PULSE_FRAMES = 3     # 30 ms at 100 Hz, inside the camera's 2-4 frame pulse
RECHECK_FRAMES = 50  # 0.5 s before looking at LFA_ICON again: 3x the ADRV's 0.16 s response
MAX_ATTEMPTS = 3


class StockLfaDisabler:
  def __init__(self):
    self.pulse_frames = 0
    self.wait_frames = 0
    self.attempts = 0

  def update(self, stock_lfa_on: bool, want_lateral: bool) -> bool:
    """One 100 Hz step. Returns True while LKAS_ALT.LFA_BUTTON should be set this frame."""
    if not (stock_lfa_on and want_lateral):
      # the car's lane centering is already off, or we are not asking for the wheel (which includes the HDA-road gate
      # window, where the stock system is deliberately the one steering). Re-arm a fresh set of attempts.
      self.pulse_frames = 0
      self.wait_frames = 0
      self.attempts = 0
      return False

    if self.pulse_frames > 0:
      self.pulse_frames -= 1
      return True

    if self.wait_frames > 0:
      self.wait_frames -= 1
      return False

    if self.attempts >= MAX_ATTEMPTS:
      # it is not listening: stop pressing. The relay watchdog keeps lateral out of the fight.
      return False

    self.attempts += 1
    self.pulse_frames = PULSE_FRAMES - 1
    self.wait_frames = RECHECK_FRAMES
    return True
