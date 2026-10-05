"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Switches the car's own lane centering off on LKA-steer CAN-FD cars (2026 Palisade LX3).
#
# Mechanism, from route 69fb86b6677ce882/00000008--c7bfd877d8, where the panda was still in elm327 and the camera's
# LKAS_ALT (0x110) reached the ADAS ECU: each LFA button press on WHEEL_BUTTONS_ALT (0x10B) made the camera pulse
# LKAS_ALT.LFA_BUTTON for 2-4 frames, and 0.15-0.18 s later the ADRV toggled LFA_ICON in LFAHDA_CLUSTER (0x1E0). Four
# presses, four toggles. That bit is the only input that turns the car's lane centering on or off: the ADRV sees the
# button on 0x10B itself and ignores it there (route .../0000005e, ten presses, no response). With openpilot driving,
# panda statically blocks the camera's 0x110, so without this the button cannot reach the ADRV at all and the car's lane
# centering, once HDA switches it on, stays on until ignition-off (route .../0000005e, twelve minutes).
#
# Verified on route .../0000007a (spoof) and .../00000087 (driver take-back): our pulse is honored with the same
# 0.15-0.18 s latency as the camera's, four take-backs out of four, and with a pulse the ADRV's own 0xCB gain reaches
# zero 0.6-0.9 s after a cancel, against 1.6 s or never without one.
#
# THREE HARD RULES, all from the logs:
#  1. Never pulse while LFA_ICON reads off. The button is a toggle, so a pulse there switches the car's lane centering
#     ON. That is also why the icon has to be confirmed over two 20 Hz messages (ICON_CONFIRM_FRAMES) before the first
#     pulse of a sequence.
#  2. Never pulse on our own inside an HDA window unless auto-suppress is on: there we have deliberately handed the
#     stock system the wheel, and switching it off would leave nobody steering. A driver LFA press is different, it is
#     them asking for the wheel back.
#  3. Wait out the transition state. The ADRV shows LFA_ICON = 3 for about a second on its way out of the active state;
#     a pulse there only toggles it back on (route .../0000007a: of four pulses two were wasted and one re-armed it).
LFA_ICON_OFF = 0
LFA_ICON_TRANSITION = 3

PULSE_FRAMES = 3          # 30 ms at 100 Hz, inside the camera's 2-4 frame pulse
RECHECK_FRAMES = 150      # 1.5 s, longer than the ADRV's ~1 s transition, so a retry cannot toggle it back on
ICON_CONFIRM_FRAMES = 6   # 60 ms: LFA_ICON must read non-zero across two 20 Hz messages before we press
ICON_WAIT_FRAMES = 50     # 0.5 s: how long a request waits for the icon to come on at all before giving up on it
MAX_ATTEMPTS = 3


class StockLfaDisabler:
  def __init__(self):
    self.pulse_frames = 0
    self.wait_frames = 0
    self.attempts = 0
    self.icon_on_frames = 0

    self.driver_request = False  # the driver pressed LFA while we were yielding: they want the wheel back
    self.auto_request = False    # auto-suppress is on and the ADAS ECU has armed HDA on this road
    self.request_frames = 0      # frames since the current request opened
    self.icon_was_on = False     # the car's lane centering has been seen on during this request
    self.requested = False       # a request is in flight (for the UI)
    self.failed = False          # out of attempts with the car's lane centering still on
    self.take_back = False       # the request worked: release the HDA-road gate so openpilot steers

  def _rearm(self) -> None:
    self.pulse_frames = 0
    self.wait_frames = 0
    self.attempts = 0
    self.failed = False

  def update(self, lfa_icon: int, hda_window: bool, want_lateral: bool, lfa_pressed: bool,
             auto_suppress: bool = False) -> bool:
    """One 100 Hz step. Returns True while LKAS_ALT.LFA_BUTTON should be set this frame."""
    stock_on = lfa_icon != LFA_ICON_OFF
    self.icon_on_frames = self.icon_on_frames + 1 if stock_on else 0

    if lfa_pressed and hda_window and stock_on:
      self.driver_request = True
      self._rearm()
    if not hda_window:
      self.driver_request = False
    self.auto_request = auto_suppress and hda_window

    request = self.driver_request or self.auto_request
    if request:
      self.request_frames += 1
      self.icon_was_on = self.icon_was_on or stock_on
    else:
      self.request_frames = 0
      self.icon_was_on = False

    # The request worked, or there was nothing to switch off: let openpilot steer even though the ADAS ECU still shows
    # HDA on this road. The icon has to have been seen on first, because the ADRV switches its lane centering on about
    # 0.05 s after the HDA icon and releasing the gate in that gap makes it flutter (route
    # 69fb86b6677ce882/0000009b--ff814efa26 at 1242.7 s: on, off, on inside 60 ms, which paused and resumed lateral
    # twice for no reason). If the icon never comes on at all, fall back to releasing after ICON_WAIT_FRAMES: there is
    # nothing to suppress and nobody should be left steering.
    self.take_back = request and not stock_on and (self.icon_was_on or self.request_frames > ICON_WAIT_FRAMES)
    self.requested = request and stock_on

    # rule 1, second half: once a pulse has started, finish it, but only while the icon still reads on. On route
    # .../0000007a one pulse was cut to a single frame when MADS dropped mid-pulse; that one was honored, but the ADRV
    # asks for 2-4 frames.
    if self.pulse_frames > 0 and stock_on:
      self.pulse_frames -= 1
      return True

    # nothing to do: the car's lane centering is already off, we are not asking for the wheel, or we are inside an
    # HDA window with neither a driver request nor auto-suppress (rule 2). Re-arm a fresh set of attempts.
    if not (stock_on and want_lateral and (not hda_window or self.driver_request or self.auto_request)):
      self._rearm()
      return False

    if lfa_icon == LFA_ICON_TRANSITION:
      return False  # rule 3

    if self.icon_on_frames < ICON_CONFIRM_FRAMES and self.attempts == 0:
      return False  # rule 1: confirm the icon over two messages before the first press of a sequence

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
