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
# presses, four toggles. That bit is the only input that turns the car's lane centering on or off: the ADRV sees the
# button on 0x10B itself and ignores it there (route .../0000005e, ten presses, no response).
#
# With openpilot driving, panda statically blocks the camera's 0x110 (it is in the TX list with check_relay) and our own
# 0x110 used to send LFA_BUTTON = 0 always, so the button could not reach the ADRV at all. On route .../0000005e the
# ADRV switched its own lane centering on when it armed HDA (LFA_ICON 0 -> 2) and it stayed on for the remaining twelve
# minutes; only ignition-off cleared it.
#
# So we pulse the bit ourselves, in the camera's shape. Route .../0000007a is the first drive with this: after the
# driver's brake cancel we sent four pulses and the icon went out 1.5 s after the cancel, openpilot took the wheel with
# the relay tracking us to 0.10 deg, and the whole drive had no watchdog trip. Two of those four pulses were wasted, and
# one of them toggled the feature back on, because the ADRV passes through LFA_ICON = 3 for about a second on its way
# out and the old 0.5 s re-check fired into that transition. Hence RECHECK_FRAMES and the state-3 hold below.
#
# During an HDA-road gate window we do not press on our own: there we have deliberately handed the stock system the
# wheel, and switching it off would leave nobody steering. A driver LFA press in that window is different, it is them
# asking for the wheel back, so it arms the same pulse sequence and, once the stock system lets go, releases the gate
# (take_back) so openpilot can steer even though the ADAS ECU still shows HDA on this road. Whether the ADRV honors the
# bit in the middle of an HDA window is the open question this is meant to answer: on .../0000007a the two pulses it did
# honor both landed within 0.08 s of HDA_ICON going dark.
LFA_ICON_OFF = 0
LFA_ICON_TRANSITION = 3  # shown for ~1 s while the ADRV moves out of the active state (2 -> 3 -> 0, or 3 -> 1)

PULSE_FRAMES = 3       # 30 ms at 100 Hz, inside the camera's 2-4 frame pulse
RECHECK_FRAMES = 150   # 1.5 s: longer than the ADRV's own ~1 s transition, so a retry cannot toggle it back on
MAX_ATTEMPTS = 3


class StockLfaDisabler:
  def __init__(self):
    self.pulse_frames = 0
    self.wait_frames = 0
    self.attempts = 0

    self.driver_request = False  # the driver pressed LFA while we were yielding: they want the wheel back
    self.requested = False       # a request is in flight (for the UI)
    self.failed = False          # out of attempts with the car's lane centering still on
    self.take_back = False       # the driver asked and the stock system let go: release the HDA-road gate

  def _rearm(self) -> None:
    self.pulse_frames = 0
    self.wait_frames = 0
    self.attempts = 0
    self.failed = False

  def update(self, lfa_icon: int, hda_window: bool, want_lateral: bool, lfa_pressed: bool) -> bool:
    """One 100 Hz step. Returns True while LKAS_ALT.LFA_BUTTON should be set this frame."""
    stock_on = lfa_icon != LFA_ICON_OFF

    if lfa_pressed and hda_window and stock_on:
      self.driver_request = True
      self._rearm()
    if not hda_window:
      self.driver_request = False

    self.take_back = self.driver_request and not stock_on
    self.requested = self.driver_request and stock_on

    # once a pulse has started, finish it: the ADRV wants a 2-4 frame press, and on route .../0000007a one pulse was cut
    # to a single frame when MADS dropped mid-pulse (that one was honored, but do not rely on it)
    if self.pulse_frames > 0 and stock_on:
      self.pulse_frames -= 1
      return True

    # nothing to do: the car's lane centering is already off, we are not asking for the wheel, or we are inside an
    # HDA-road gate window that the driver has not asked us to take back. Re-arm a fresh set of attempts.
    if not (stock_on and want_lateral and (not hda_window or self.driver_request)):
      self._rearm()
      return False

    if lfa_icon == LFA_ICON_TRANSITION:
      # the ADRV is already on its way out of the active state; pressing again here only toggles it back on
      return False

    if self.wait_frames > 0:
      self.wait_frames -= 1
      return False

    if self.attempts >= MAX_ATTEMPTS:
      # it is not listening: stop pressing and say so. The relay watchdog keeps lateral out of the fight.
      self.failed = True
      return False

    self.attempts += 1
    self.pulse_frames = PULSE_FRAMES - 1
    self.wait_frames = RECHECK_FRAMES
    return True
