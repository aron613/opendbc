# Palisade LX3 (2026, gas, HDA II/LFA2) — porting findings

Owner's vehicle: 2026 Hyundai Palisade Calligraphy, gas (non-hybrid), LX3 platform, HDA2, CAN-FD, comma 3X.
Harness: comma Hyundai N with the CAN H/L pairs manually swapped from stock pinout. **Confirmed correct** — see "Bus mapping" below. Keep the flip.

Routes referenced:
- Before pin flip: `69fb86b6677ce882/00000000--3284f58264/0`
- After pin flip: `69fb86b6677ce882/00000003--94eb544029` (3 segments)
- Deliberate button-press test: `69fb86b6677ce882/00000008--c7bfd877d8` (7 segments, car unrecognized → panda in passthrough)
- First run of this branch, car recognized, parked passive test (LFA ×4 on/off, full-lock sweep, no cruise): `69fb86b6677ce882/00000004--925bf85ead` (2 segments)
- Reference (known-working, same `hyundai_n` harness, not the owner's car): `KIA_SPORTAGE_HEV_2026` test route `1635e7fee82dec3b/00000000--aa7efe7199`

## Bus layout (from the owner's own captured logs, after the pin flip)

- Bus 1: powertrain/steering (`STEERING_SENSORS` 0x125, `LFA_ALT` 0xCB, `SCC_CONTROL` 0x1A0, `CRUISE_BUTTONS_ALT` 0x1AA, `LFAHDA_CLUSTER` 0x1E0, `CCNC_0x161` 0x161 — everything of interest lives here)
- Bus 0: camera/radar
- Bus 2: mirror of bus 0

## Bus mapping — resolved

Bus roles for CAN-FD cars are **not** fixed by our static `values.py` flags. `interface.py` auto-detects them per-vehicle from the fingerprint:

```python
cam_can = CanBus(None, fingerprint).CAM                          # bus 2, single panda
lka_steering = 0x50 in fingerprint[cam_can] or 0x110 in fingerprint[cam_can]
CAN = CanBus(None, fingerprint, lka_steering)
```

If `0x50`/`0x110` show up on the camera's own transmit bus (bus 2), `lka_steering` is detected `True` and the code assigns powertrain (`ECAN`) = bus 1, camera (`ACAN`) = bus 0. Otherwise `ECAN` = bus 0, `ACAN` = bus 1. This is independent of the `CANFD_ANGLE_STEERING` flag we set — a car can be (and Sportage HEV 2026 is) both `lka_steering=True` *and* angle-steering at the same time.

Checked all three routes for `0x110` (0x50 never appears in any of them):

| | bus 0 | bus 1 | bus 2 | 0x110 seen on |
|---|---|---|---|---|
| Owner, before flip | 215 IDs (steering/cruise) | 86 IDs (camera/radar) | 29 msgs (nearly dead) | bus 1 |
| Owner, after flip | 87 IDs (camera/radar) | 222 IDs (powertrain) | 150 IDs | bus 0 **and** bus 2 |
| Sportage HEV 2026 reference | 86 IDs | 235 IDs | 148 IDs | bus 0 **and** bus 2 |

The owner's after-flip topology (87 / 222 / 150) closely matches the known-working reference (86 / 235 / 148) — same shape, and the same 0x110-on-bus-0-and-2 signature that drives `lka_steering=True` → `ECAN=1`, which is exactly where the owner's powertrain data actually is. The before-flip topology doesn't resemble the reference at all, and its near-dead bus 2 (29 messages vs. ~150 on a healthy one) indicates that connection was genuinely mis-wired, not merely cross-labeled.

**Verdict: keep the flip. No code-side bus override is needed** — the existing fingerprint-based auto-detection already resolves to the correct bus for this wiring.

**Side effect worth knowing:** because `lka_steering` auto-detects `True` for this car, it will *also* pick up `HyundaiFlags.CANFD_LKA_STEER_MSG` at runtime (dynamically — not something set in `values.py`). For lateral-only operation (`openpilotLongitudinalControl=False`), this means `carcontroller.py` sends steering as `"LKAS"`/`"LKAS_ALT"` (not `"LFA"`), and does **not** send the `"LFA"` (0x12A) or `LFAHDA_CLUSTER` (0x1E0) messages at all — confirmed on-car, see Open issue 1 (resolved) below.

## Fingerprint

Platform code `LX3` confirmed in ECU firmware (radar `0x7d0`, camera `0x7c4`). Fingerprint added as `CAR.HYUNDAI_PALISADE_LX3` (`HyundaiFlags.CANFD_ANGLE_STEERING | HyundaiFlags.CANFD_ALT_BUTTONS`).

**EPS firmware (corrected):** earlier passive routes showed no `eps` (MDPS) response, and this doc previously said so. On the first recognized run (`00000004--925bf85ead`) the MDPS *did* answer at `0x7d4`, bus 1:

```
b'\xf1\x00LX3 MDPS R 1.00 1.03 57700P9000  2551_LX3kI_RLN103'
```

The car still fingerprinted from FW (source `fw`, not fuzzy, 15 FW entries) with that extra ECU present. Other `CANFD_ANGLE_STEERING` cars in this codebase have no `eps` entry, so the earlier "none of them respond" observation was about those fingerprints, not a property of the MDPS.

Runtime flags observed in `carParams` on the recognized run: `CANFD | CANFD_ANGLE_STEERING | CANFD_ALT_BUTTONS | CANFD_LKA_STEER_MSG | CANFD_LKA_STEER_MSG_ALT` (the last two auto-detected, see bus mapping). `safetyParam` = 1200, `openpilotLongitudinalControl` = False, `pcmCruise` = True.

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

## Open issue 1 — resolved (LFA button is `LKAS_ALT.LFA_BUTTON` on the camera bus)

**Question:** is `LFA_ICON` (in `LFAHDA_CLUSTER` 0x1E0 and `CCNC_0x161` 0x161) a genuine car signal we can use as the LFA/MADS button, or is it circular with something we transmit?

**Answer from the recognized passive run (`00000004--925bf85ead`):**

- We never transmit 0x1E0, 0x12A (`LFA`) or 0xCB (`LFA_ALT`). `sendcan` only carried `LKAS_ALT` (0x110) and `CAM_0x362` (0x362) on bus 0 (A-CAN), from t=15.6 s when the panda entered `hyundaiCanfd` safety mode, plus one-shot diagnostic queries. So `LFA_ICON` is **not** circular with our code.
- But `LFA_ICON` **never moved** in this route. Both 0x1E0 and 0x161 were byte-for-byte constant for all 86 s, despite 4 on/off LFA cycles (8 button presses).
- The button itself was found in the **camera's own** `LKAS_ALT` (0x110) message on bus 2: signal `LFA_BUTTON` (bit 56, 1 bit, already in `hyundai_canfd.dbc`) pulsed to 1 for ~30 ms at t = 36.2, 40.1, 43.9, 47.4, 51.2, 55.0, 58.5, 62.0 s — 8 pulses, ~3.7 s apart, matching the presses.
- Why the icon stayed off: in `hyundaiCanfd` mode with `CANFD_LKA_STEER_MSG_ALT`, the panda **blocks** the camera's 0x110 from being forwarded bus 2 → bus 0 (bus-0 receptions of 0x110 stop at t=15.6 s exactly) and openpilot sends its own 0x110 with `LFA_BUTTON = 0`. The ADRV never sees the press, so it never toggles `LFA_ICON`.
- Cross-check on the unrecognized button-press route (`00000008`, panda in `elm327` passthrough, camera 0x110 forwarded untouched): `LFA_BUTTON` pulsed at 87.16, 92.16, 96.68, 102.12, 228.81, 232.45 s and `LFA_ICON` in 0x1E0 toggled ~180 ms after each (87.34, 92.34, 96.83, 102.29, 228.97, 232.62). Causal chain: button → camera `LKAS_ALT.LFA_BUTTON` → forwarded to ADRV → `LFA_ICON`.

**Conclusion:** `LFA_ICON` was a real car signal, but it is *downstream of a message we intercept*, so it is dead whenever openpilot is running. Do not use it. The correct button source is **`LKAS_ALT.LFA_BUTTON` read from the camera bus (bus 2 / `CAN.CAM`)**. We never transmit on that bus, so there is no circularity, and it is unaffected by whatever we put in our own 0x110.

## What blocked engagement on the recognized run (must be fixed before any engagement test)

The car was recognized and openpilot never engaged (as intended: `selfdriveState.enabled` false on all 7618 frames, MADS disabled/unavailable, `cruiseState.enabled` never true, panda `controlsAllowed` false throughout). But it also *could not* have engaged, for two independent reasons:

### A. openpilot side: `carState.canValid` was False on every frame → permanent `canError` ("Unknown Vehicle Variant") from t=16.5 s

Replaying the route's CAN through the car's own parsers (`Bus.pt` on bus 1, `Bus.cam` on bus 2) gives five failing messages:

| Message | Addr | Failure | Notes |
|---|---|---|---|
| `DOORS_SEATBELTS` | 0x411 | never appears on any bus | LX3 equivalent unknown |
| `BLINKERS` | 0x413 | never appears on any bus | LX3 equivalent unknown; blinkers were used in route `00000008` |
| `HOD_FD_01_100ms` | 0x2AF | never appears on any bus | hands-on-detection; LX3 equivalent unknown |
| `GEAR_SHIFTER` | 0x130 | received, checksum OK, **counter +2 per frame** | 4242 frames in 86.6 s ≈ 49 Hz; `MAX_BAD_COUNTER` hit immediately |
| `ACCELERATOR_BRAKE_ALT` | 0x100 | received, checksum OK, **counter +2 per frame** | same 49 Hz / +2 pattern |

`GEAR_ALT` 0x40 shows the identical 49 Hz / +2 pattern (not currently parsed because 0x130 is present). `ACCELERATOR_ALT` 0x105 has a constant counter. Everything else parsed (`WHEEL_SPEEDS`, `STEERING_SENSORS`, `MDPS`, `IMU_01_10ms`, `TCS`, `SCC_CONTROL`, `CRUISE_BUTTONS_ALT`, `ADAS_CMD_50_50ms`, `FR_CMR_02_100ms`, `CAM_0x362`) was valid with good counters and checksums. The +2 pattern looks like the LX3 gateway relaying nominally 100 Hz messages at half rate; the DBC checksum still verifies, so the frames are genuine.

Note also: bus 0 and bus 2 carry a *different* 0x100 (24 bytes) than bus 1 (32 bytes). Only the bus-1 one is `ACCELERATOR_BRAKE_ALT`.

### B. panda side: `safetyRxChecksInvalid` True on 698 of 700 `hyundaiCanfd` frames → `controlsMismatch`, and `controls_allowed` forced false

`opendbc/safety/modes/hyundai_canfd.h`, LKA-steering branch (`hyundai_canfd_lka_steer_msg && !longitudinal`), used `HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)` unconditionally — the comment literally said "Does not use the alt buttons message". That requires `CRUISE_BUTTONS` **0x1CF** on bus 1, which this car never emits (confirmed across every route). The common checks also require either `0x35` or `0x100` with a +1 counter; 0x35 is absent and 0x100 steps by 2, so that check fails too. Either failure alone keeps `controls_allowed` false.

**Fixed on this branch (not yet driven):**

- The LKA-steer branch now selects `HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(1)` (0x1AA instead of 0x1CF) when `CANFD_ALT_BUTTONS` is set. The 0x1CF variant is untouched for cars that use it.
- A new generic `counter_step` field on `CanMsgCheck` (default 0 → +1, so every existing message is unchanged) lets a message declare a fixed counter increment. `HYUNDAI_CANFD_HALF_RATE_GAS_COMMON_RX_CHECKS` uses it to expect exactly +2 on 0x100 — checksum, counter and frequency checks all still apply; only 0x100 is listed (0x35 absent, 0x105 present but its counter never moves, so it must not be an alternative). Selected only when `CANFD_ALT_BUTTONS` **and** the new `HyundaiSafetyFlagsSP.CANFD_HALF_RATE_COUNTERS` (safety_param_sp bit 8) are set; `interface.py` sets that bit from `HyundaiFlags.CANFD_HALF_RATE_COUNTERS`.
- `HYUNDAI_ANGLE_MODEL_HYUNDAI_PALISADE_LX3` (id 11) added with `slip_factor = -0.0005647415830223231`, `steer_ratio = 14.3`, `wheelbase = 2.97`, computed with `calc_slip_factor(VehicleModel(CP))` exactly like the existing entries. The ISO lateral accel/jerk limits are the shared ones in `steer_angle_cmd_checks_vm`; the entry only fixes the physics so they are enforced at this car's angle scale. `test_lateral_jerk_limit` passes for `HYUNDAI_PALISADE_LX3` with it.
- New safety tests: `TestHyundaiCanfdLKASteeringAltAngleAltButtons` (0x1AA satisfies the checks, 0x1CF no longer does, +2 gas counter still rejected without the flag) and `TestHyundaiCanfdLKASteeringAltAngleHalfRateCounters` (+2 accepted, +1 and repeated frames rejected, flag without alt buttons falls back to the standard checks).

### Fix status (openpilot side) — implemented, not yet driven

Implemented on this branch (opendbc), verified only by replaying the two routes above through
`CarInterface` with `carParams` rebuilt via `get_params` (`canValid` holds for the whole route once
fingerprinting finishes; the panda-side blocker below is untouched):

| Need | Where it lives on LX3 | How it was verified |
|---|---|---|
| Driver seatbelt | `SEATBELTS_ALT` 0x3E0 (24 B, ~5 Hz, E-CAN), `DRIVER_SEATBELT` bit 24, 1 = latched | Route `00000008`: bit was 1 the whole drive and dropped to 0 at t=387.5 s, 0.2 s after the shift to P at the end. Route `00000004` (parked test): 0 throughout. |
| Driver door | `DOORS_ALT` 0x3E2 (16 B, ~5 Hz, E-CAN), `DRIVER_DOOR` bit 64 | **Not verified** — no door-open event in either route; position taken from a third-party LX3 HEV DBC. Byte 9 bits 2/4/6 + byte 10 bit 0 all cleared at t=130.67 s right after the shift out of P (auto door lock), so those look like lock states, not door-open. |
| Blinkers | `BLINKERS_ALT` 0x3E3 (16 B, ~5 Hz + event frames, E-CAN). Left: lamp bit 90, active bit 93. Right: lamp bit 92, active bit 95. Each side is a 2-bit field (on = lamp bit, off = the bit below it); byte 12 bit 1 = any indicator active. | Route `00000008`: lamp bits flash at 1.25 Hz inside constant "active" envelopes. Left/right assignment from steering direction: all 3 left bursts sit at positive (left) angles (e.g. +239° mean leaving the parking spot), all 5 right bursts at negative angles (down to −330°). Matches the third-party DBC. |
| Hands-on detection | **Not found.** 0x2AF does not exist on this car. 0x3D4 byte 5 (0x90/0x60) only follows driver *torque* direction (non-zero in 19 % of high-torque samples, 1.6 % otherwise), so it is not a capacitive hands-on signal. | `hands_on_steering_grip` is not read on this platform; it is unused elsewhere anyway. `steeringPressed` comes from MDPS torque and works (16 transitions during the parked sweep). |
| Gear / accelerator counters | `GEAR_SHIFTER` 0x130 and `ACCELERATOR_BRAKE_ALT` 0x100 arrive at ~49 Hz with COUNTER +2 per frame (4238 of 4241 deltas). `GEAR_ALT` 0x40 same pattern; `ACCELERATOR_ALT` 0x105 counter is constant. | New `HyundaiFlags.CANFD_HALF_RATE_COUNTERS` → `CANParser.set_counter_step(msg, 2)` for the gear and accelerator messages. Replay of `00000008` decodes P→R→N→D→N→R→P, brake before the shift to R, gas pulses while driving. |
| LFA / MADS button | `LKAS_ALT` 0x110 on the **camera bus** (bus 2), `LFA_BUTTON` bit 56, ~30 ms pulse per press | `mads.py` reads it for `CANFD_ANGLE_STEERING + CANFD_LKA_STEER_MSG_ALT` cars and `carstate.py` emits `ButtonType.lkas` press/release. Replay: 8/8 presses on `00000004` (t=36.2 … 62.1), 6/6 on `00000008`. |

Flags added: `HyundaiFlags.CANFD_ALT_BODY_MSGS` (0x3E0/0x3E2/0x3E3 instead of 0x411/0x413, no 0x2AF) and
`HyundaiFlags.CANFD_HALF_RATE_COUNTERS`; both set statically on `HYUNDAI_PALISADE_LX3`.

Still open after this: the cruise buttons themselves (RES/SET/CANCEL/main/gap)
which are still not decoded anywhere — `pcmCruise` engagement relies on `SCC_CONTROL.ACCMode` from the car, so
lateral-only should not need them. Note the panda still requires a recent RES/SET/CANCEL/main press *in 0x1AA* before
it honors a cruise-engaged rising edge (`hyundai_common_cruise_state_check`), and those bits never move on this car, so
stock-cruise engagement will not enable openpilot until the real button bits are found. MADS (LFA button) is unaffected.

### Also confirmed on this run

- `STEERING_SENSORS` 0x125: continuous, 100 Hz, valid; full-lock sweep +510.6° (t=68.6) / −509.9° (t=74.8), mirrored in `carState.steeringAngleDeg`.
- No panda faults, no relay malfunction. Alerts were only `startupMaster` and the permanent `canError`; events `canError`, `controlsMismatch`, `locationdTemporaryError` (side effect of invalid carState).
- Benign log noise: "car doesn't match any Neural Network model", iso-tp bad responses during the FW query, athenad websocket exceptions.

## Open issue 2 — resolved

Bus mapping is no longer an open issue. See "Bus mapping — resolved" above: the flip is correct, matches the reference car's topology, and needs no code change.
