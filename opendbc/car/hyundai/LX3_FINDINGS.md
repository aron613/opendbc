# Palisade LX3 (2026, gas, HDA II/LFA2) — porting findings

Owner's vehicle: 2026 Hyundai Palisade Calligraphy, gas (non-hybrid), LX3 platform, HDA2, CAN-FD, comma 3X.
Harness: comma Hyundai N with the CAN H/L pairs manually swapped from stock pinout ("verified working" per owner, but see **Open issue 2** below — this has not been re-validated against what this code actually expects).

Routes referenced:
- Before pin flip: `69fb86b6677ce882/00000000--3284f58264/0`
- After pin flip: `69fb86b6677ce882/00000003--94eb544029` (3 segments)
- Deliberate button-press test: `69fb86b6677ce882/00000008--c7bfd877d8` (7 segments)

## Bus layout (from the owner's own captured logs, after the pin flip)

- Bus 1: powertrain/steering (`STEERING_SENSORS` 0x125, `LFA_ALT` 0xCB, `SCC_CONTROL` 0x1A0, `CRUISE_BUTTONS_ALT` 0x1AA, `LFAHDA_CLUSTER` 0x1E0, `CCNC_0x161` 0x161 — everything of interest lives here)
- Bus 0: camera/radar
- Bus 2: mirror of bus 0

## Fingerprint

Platform code `LX3` confirmed in ECU firmware (radar `0x7d0`, camera `0x7c4`). No `eps` (MDPS) firmware response — consistent with every other `CANFD_ANGLE_STEERING` car already in this codebase, none of which have an `eps` FW entry either. Fingerprint added as `CAR.HYUNDAI_PALISADE_LX3` (`HyundaiFlags.CANFD_ANGLE_STEERING | HyundaiFlags.CANFD_ALT_BUTTONS`).

## Steering messages

- `0x125` `STEERING_SENSORS` — measured angle, confirmed present continuously on bus 1, decodes sanely (full-lock sweep both directions observed, ±510°).
- `0xCB` `LFA_ALT` — the angle command message (confirmed to exist in the shared CAN-FD dbc; this is what `carcontroller.py` sends for angle-steering cars via `create_steering_messages` under the "LFA" message name — same address).

## Button message: `CRUISE_BUTTONS_ALT` (0x1AA) present, but its button fields are dead

- Standard `CRUISE_BUTTONS` (0x1CF, 8 bytes) **never appears on any bus**, in ~550s of combined logs across two routes.
- `CRUISE_BUTTONS_ALT` (0x1AA, 16 bytes) is present continuously on bus 1 — confirms `CANFD_ALT_BUTTONS` is the correct flag for the *address*.
- However, across a route with deliberate presses of every cruise button (main, RES, SET, CANCEL, gap×4, LFA×4), the DBC's named fields in this message never moved:
  - `CRUISE_BUTTONS` (bit 36, 3 bits): always 0
  - `ADAPTIVE_CRUISE_MAIN_BTN` (bit 34): always 0
  - `LDA_BTN` (bit 39): always 0
  - `NORMAL_CRUISE_MAIN_BTN` (bit 41): always 0 (byte 5, which never changes at all in this message)
- The bytes that *do* change in 0x1AA (bytes 4, 6, 8, and low bits of 10/11) are rolling counters/heartbeats (binary-counter halving pattern confirmed), not button state.
- One genuine sparse signal was found at byte 9 / byte 10 bit 7 (mirrored), 5 discrete pulses, but all 5 occur while the vehicle is *moving*, not during the parked test — best (unconfirmed) guess is turn-signal-stalk activity, not a button.

**Conclusion: on LX3, cruise/LFA button state is not carried in `CRUISE_BUTTONS_ALT` at all.** `CANFD_ALT_BUTTONS` gets the parser looking at the right *address*, but the fields carstate.py currently reads from it (`CRUISE_BUTTONS`, `ADAPTIVE_CRUISE_MAIN_BTN`, `LDA_BTN`) will never produce a button event on this car.

## Where the real state lives instead

**`SCC_CONTROL` (0x1A0/416, sender ADRV)** — `MainMode_ACC` (bit 66), `ACCMode` (bit 68, 3 bits), `DISTANCE_SETTING` (bit 88, 3 bits) all correctly reflect main-cruise-on, engage, and cancel, but **only once the vehicle is moving** — pressing these buttons while parked produced no observable state change at all (the ACC system appears to refuse to arm while stationary; not a decode failure).

**`LFAHDA_CLUSTER` (0x1E0/480) `LFA_ICON`** (bit 47, 2 bits) and **`CCNC_0x161` (0x161/353) `LFA_ICON`** (bit 224, 4 bits) both toggled 0↔1 at the *identical* timestamps, matching the owner's described LFA on/off/on/off sequence (4 transitions while parked, one more pair while driving). `LFA_ICON == 0` corresponds to LFA off.

## Open issue 1 (critical, blocks using LFA_ICON as a safety signal going forward)

`carcontroller.py` **unconditionally transmits both of these exact messages once this car is recognized**:
- `create_steering_messages` sends the whole `"LFA"` message (0x12A) every frame for angle-steering cars (our flag config) — it never sets `LFA_BUTTON`, so that field defaults to 0 regardless of the real button. (Separately, this is *not* the same message as `LFAHDA_CLUSTER`/0x1E0 — the two carry different signals.)
- `create_lfahda_cluster` sends `LFAHDA_CLUSTER` (0x1E0) every 5 frames, with `LFA_ICON` computed purely from `CC.enabled`/MADS internal state (`opendbc/sunnypilot/car/hyundai/mads.py`), **not read from any physical button**.

This means the clean 4-toggle `LFA_ICON` pattern found in the test route is trustworthy *only because that route was almost certainly recorded while the car was still unrecognized* (fingerprint reported "MOCK" prior to this session's fixes, so this custom `CarController` was never running and the real ADAS ECU was still broadcasting `LFAHDA_CLUSTER` on its own). **Once this branch is actually flashed and the car is recognized, openpilot itself becomes the writer of `LFAHDA_CLUSTER`, and reading it back would be circular** — it would only ever equal what `mads.py` already computed, telling us nothing about the real steering-wheel button.

`CCNC_0x161` is never written anywhere in this codebase, so it remains a candidate for a genuinely independent signal — but only if it is confirmed to be a real, separate ECU broadcast rather than a downstream relay of the same `LFAHDA_CLUSTER` content we'll be spoofing on the same physical bus once our code is active. That has not been verified and can't be verified from logs captured before our code ever ran.

**Before wiring anything to `LFA_ICON` as a "stop steering" safety trigger, this branch needs to be flashed and driven, and the button-press test repeated, to see whether `CCNC_0x161`'s `LFA_ICON` still tracks the real button independently of `LFAHDA_CLUSTER`'s openpilot-driven value, or whether it just mirrors it.**

## Open issue 2 (critical, likely blocks the car working at all as currently configured)

`hyundaicanfd.CanBus` assigns bus roles for non-`CANFD_LKA_STEER_MSG` cars (our flag config) as: `ECAN` (powertrain, read by `carstate.py`'s `Bus.pt` parser) = **bus 0**, `ACAN` (camera) = **bus 1**.

But the owner's own confirmed physical mapping (bus 1 = powertrain, bus 0 = camera/radar) is the **exact opposite**. As currently configured, `carstate.py`'s `Bus.pt` parser would be listening on physical bus 0 — camera/radar traffic — and would never see `STEERING_SENSORS`, `SCC_CONTROL`, `CRUISE_BUTTONS_ALT`, or any other powertrain signal at all.

This needs to be resolved before any carstate signal (including the LFA check) can be trusted to read real data on-car. Two possibilities, not yet distinguished:
1. The manual pin flip was unnecessary or wrong for this generation, and stock (un-flipped) N-harness wiring would put powertrain on bus 0 as the code expects (matching every other car using this same code path).
2. The flip is correct/needed for this specific vehicle, and the code needs an explicit bus-role override for this platform (independent of the `CANFD_LKA_STEER_MSG` steering-format flag, since those are conceptually different things that happen to share one flag/parameter today).

Not fixed yet — needs a decision (see the harness note in `values.py` for the raw bus mapping, and this file for why it needs to be reconciled with `CanBus`'s ECAN/ACAN assignment before trusting any carstate signal on-car).
