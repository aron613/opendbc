# Palisade LX3 (2026, gas, HDA II/LFA2) — porting findings

Owner's vehicle: 2026 Hyundai Palisade Calligraphy, gas (non-hybrid), LX3 platform, HDA2, CAN-FD, comma 3X.
Harness: comma Hyundai N with the CAN H/L pairs manually swapped from stock pinout. **Confirmed correct** — see "Bus mapping" below. Keep the flip.

Routes referenced:
- Before pin flip: `69fb86b6677ce882/00000000--3284f58264/0`
- After pin flip: `69fb86b6677ce882/00000003--94eb544029` (3 segments)
- Deliberate button-press test: `69fb86b6677ce882/00000008--c7bfd877d8` (7 segments, car unrecognized → panda in passthrough)
- First run of this branch, car recognized, parked passive test (LFA ×4 on/off, full-lock sweep, no cruise): `69fb86b6677ce882/00000004--925bf85ead` (2 segments)
- Second run, MADS on, longitudinal off, parked LFA presses + door cycles + ~5 s of driving: `69fb86b6677ce882/00000008--89dc7fcd9c` (see "Run with MADS on" below)
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

**Conclusion: on LX3, cruise/LFA button state is not carried in `CRUISE_BUTTONS_ALT` at all.** `CANFD_ALT_BUTTONS` gets the parser looking at the right *address*, but the fields carstate.py reads from it (`CRUISE_BUTTONS`, `ADAPTIVE_CRUISE_MAIN_BTN`, `LDA_BTN`) will never produce a button event on this car. The real button message is `WHEEL_BUTTONS_ALT` 0x10B, next section.

## Wheel buttons — resolved: `WHEEL_BUTTONS_ALT` (0x10B)

Found by scanning every bit on every bus for toggles within 0.2 s of each button press on the two recognized runs, then
decoding the full payload on the passthrough button-press route (`00000008--c7bfd877d8`), where the panda forwarded the
camera untouched so the cluster and SCC answered every press.

Message: **0x10B, 16 bytes, 25 Hz, E-CAN (bus 1).** Bytes 0-1 are the standard HKG CAN-FD checksum (matches `hkg_can_fd_checksum` on
501/501 logged frames), byte 2 is the counter and it **advances by 2 per frame** (5745 of 5747 deltas, same gateway
half-rate pattern as 0x100/0x130). Not multiplexed: bytes 3-15 hold one fixed pattern (byte 11 = 0x20) and only **byte 10**
ever changes. Byte 10 is a button-ID byte held for ~0.2-0.25 s per press:

| Byte 10 | DBC bit | Car's response on the passthrough route | Button | Signal |
|---|---|---|---|---|
| 0x80 | 87 | `LFA_ICON` (0x1E0 and 0x161) toggled ~0.45 s after every pulse; 6 pulses = 4 parked + 2 driving LFA presses | LFA | `LFA_BTN` |
| 0x08 | 83 | `SCC_CONTROL.MainMode_ACC` and `ACCMode` went 0→1 with `VSetDis` 20 (t=143.3, 219.3); second press turned it off (t=218.4). In Park it only raised cluster `ALERTS_5`=4 | cruise / main (speedometer icon) | `MAIN_BTN` |
| 0x01 | 80 | cluster `SETSPEED_SPEED` and `VSetDis` +1 per press (145.8 … 152.0, 220.5 … 222.5) | RES + | `RES_ACCEL_BTN` |
| 0x02 | 81 | `SETSPEED_SPEED` and `VSetDis` −1 per press (149.2, 224.9 … 227.1) | SET − | `SET_DECEL_BTN` |
| 0x03 | 80+81 | 5 presses in Park (61.6 … 82.4), no cluster or SCC response | **unidentified: gap or cancel** | deliberately unmapped |

The parked part of that route was described as RES, SET, CANCEL, gap×4, main, LFA×4 but the log holds 2× 0x08, 5× 0x03,
4× 0x80, so the parked codes cannot be assigned from the description alone; the Drive segment is unambiguous. 0x03 needs one
short test in Drive with cruise engaged (press gap, then cancel, ~5 s apart) so the cluster distinguishes them. Until then both
openpilot and the panda treat RES+SET-together as *no button*.

The camera's `LKAS_ALT.LFA_BUTTON` (bit 56, bus 2) is an echo of the 0x80 press: it pulses ~20 ms after the 0x10B bit falls.
It is superseded by 0x10B, which is earlier, checksummed, on the validated powertrain bus, and readable by the panda.

**Implemented on this branch** (`HyundaiFlags.CANFD_ALT_WHEEL_BUTTONS`, `HyundaiSafetyFlagsSP.CANFD_ALT_WHEEL_BUTTONS` = safety_param_sp bit 9):

- DBC: `WHEEL_BUTTONS_ALT` with the four bit signals above.
- carstate: `cruise_buttons` (RES/SET), `main_buttons` and `buttons_counter` from 0x10B; `mads.py` reads `LFA_BTN` from it (the camera-echo path is gone). Parser counter step 2.
- panda: `HYUNDAI_CANFD_ALT_WHEEL_BUTTONS_RX_CHECKS` = the alt-buttons + half-rate set **plus** 0x10B (checksum, exact +2 counter, 25 Hz). 0x1AA stays in the set. The rx hook decodes RES/SET/main/LFA from 0x10B for `hyundai_common_cruise_buttons_check` and the MADS button, and skips the 0x1AA field decode (those fields are always zero here and feeding both messages into the recent-press counter would shrink its window). The flag only takes effect on top of `CANFD_ALT_BUTTONS` + `CANFD_HALF_RATE_COUNTERS`; without them the panda falls back to the previous set and ignores 0x10B.
- panda checksum: `hyundai_common_canfd_compute_checksum` had no final XOR for 16-byte frames (only 24 and 32). No 16-byte message was checksum-checked before (every 16-byte RX entry has `ignore_checksum`), so nothing existing changes; the 16-byte constant `0x041D` from `hkg_can_fd_checksum` was added and a test feeds real logged 0x10B frames through the panda.
- Replay of both recognized routes through `CarInterface` with the rebuilt `carParams`: `canValid` on every frame after startup; on `c7bfd877d8` the events are mainCruise ×2 (Park), lkas ×4, mainCruise, accelCruise ×7, decelCruise, mainCruise ×2, accelCruise ×4, decelCruise ×4, lkas ×2 in exactly the logged order, `cruiseState.enabled` follows `ACCMode` (143.3-153.1, 219.3-236.2), and the five 0x03 presses produce no event. On `89dc7fcd9c` all 9 presses are `lkas`.

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

**Conclusion:** `LFA_ICON` was a real car signal, but it is *downstream of a message we intercept*, so it is dead whenever openpilot is running. Do not use it. The camera-bus `LKAS_ALT.LFA_BUTTON` was used as the button source for one run; it has since been replaced by the physical button message `WHEEL_BUTTONS_ALT` 0x10B bit 87 on E-CAN (see "Wheel buttons — resolved"), which precedes the camera echo and is also what the panda reads.

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
| Driver door | `DOORS_ALT` 0x3E2 (16 B, ~5 Hz, E-CAN), `DRIVER_DOOR` bit 64 | **Verified** on `00000008--89dc7fcd9c`: three driver-door open/close cycles (165.3-167.3, 168.6-175.8, 178.1-179.8 s) registered as `doorOpen`. Byte 9 bits 2/4/6 + byte 10 bit 0 all cleared right after the shift out of P (auto door lock), so those are lock states. |
| Blinkers | `BLINKERS_ALT` 0x3E3 (16 B, ~5 Hz + event frames, E-CAN). Left: lamp bit 90, active bit 93. Right: lamp bit 92, active bit 95. Each side is a 2-bit field (on = lamp bit, off = the bit below it); byte 12 bit 1 = any indicator active. | Route `00000008`: lamp bits flash at 1.25 Hz inside constant "active" envelopes. Left/right assignment from steering direction: all 3 left bursts sit at positive (left) angles (e.g. +239° mean leaving the parking spot), all 5 right bursts at negative angles (down to −330°). Matches the third-party DBC. |
| Hands-on detection | **Not found.** 0x2AF does not exist on this car. 0x3D4 byte 5 (0x90/0x60) only follows driver *torque* direction (non-zero in 19 % of high-torque samples, 1.6 % otherwise), so it is not a capacitive hands-on signal. | `hands_on_steering_grip` is not read on this platform; it is unused elsewhere anyway. `steeringPressed` comes from MDPS torque and works (16 transitions during the parked sweep). |
| Gear / accelerator counters | `GEAR_SHIFTER` 0x130 and `ACCELERATOR_BRAKE_ALT` 0x100 arrive at ~49 Hz with COUNTER +2 per frame (4238 of 4241 deltas). `GEAR_ALT` 0x40 same pattern; `ACCELERATOR_ALT` 0x105 counter is constant. | New `HyundaiFlags.CANFD_HALF_RATE_COUNTERS` → `CANParser.set_counter_step(msg, 2)` for the gear and accelerator messages. Replay of `00000008` decodes P→R→N→D→N→R→P, brake before the shift to R, gas pulses while driving. |
| LFA / MADS button | `WHEEL_BUTTONS_ALT` 0x10B (E-CAN), `LFA_BTN` bit 87, held ~0.2 s per press | See "Wheel buttons — resolved". Replay: 6/6 on `c7bfd877d8`, 9/9 on `89dc7fcd9c`, each followed by the car's own `LFA_ICON` toggle on the passthrough route. |
| Cruise buttons | `WHEEL_BUTTONS_ALT` 0x10B: `RES_ACCEL_BTN` bit 80, `SET_DECEL_BTN` bit 81, `MAIN_BTN` bit 83 | Cluster set speed ±1 per RES/SET press, SCC on/off per main press on `c7bfd877d8`. Gap and cancel not yet identified (code 0x03 is one of them). |

Flags added: `HyundaiFlags.CANFD_ALT_BODY_MSGS` (0x3E0/0x3E2/0x3E3 instead of 0x411/0x413, no 0x2AF) and
`HyundaiFlags.CANFD_HALF_RATE_COUNTERS`; both set statically on `HYUNDAI_PALISADE_LX3`.

Still open after this: which of gap / cancel is byte-10 code 0x03 in 0x10B (the other one has not been seen at all). Until
that is tested, openpilot never emits a `gapAdjustCruise`/`cancel` button event on this car and the panda does not count
0x03 as a recent press. RES, SET and main are decoded from 0x10B on both sides now, so the panda's recent-press
requirement for stock-cruise engagement (`hyundai_common_cruise_state_check`) can be met.

## Run with MADS on (`00000008--89dc7fcd9c`) — why every LFA press ended in "Controls Mismatch: Lateral"

Build 1a57f8a25e (0x1AA RX checks + half-rate counter + angle model, LFA from the camera echo). Fingerprinted, `canValid`
throughout, `safetyRxChecksInvalid` false throughout, no TX blocked, no panda faults — the previous run's blockers were gone.

- Nine presses of the LFA button (all nine: 0x10B byte 10 = 0x80; no 0x08 anywhere, so the cruise button was not pressed
  on this run despite what it felt like). Five in Park (MADS toggled on/paused, "Gear not D"), three in Drive.
- Each Drive press: openpilot MADS `enabled/active` at once (from the camera-echo `LFA_BUTTON`), then exactly 2 s later
  `controlsMismatchLateral` → "TAKE CONTROL IMMEDIATELY" and MADS disabled. Panda `controlsAllowedLateral` was false for
  the entire route: its MADS button is read from 0x1AA bit 39 and its ACC-main edge from `SCC_CONTROL` bit 66, and neither ever
  moved. `mads.py` counts 200 frames of "openpilot active, panda not" and disengages. This is what the 0x10B work fixes.
- openpilot never commanded steering: `carControl.latActive` stayed false because MADS was always dropped before `vEgo`
  exceeded the standstill threshold (max 0.5 m/s). The transmitted `LKAS_ALT` kept `LKAS_ANGLE_ACTIVE` = 1 (inactive),
  torque gain 0, angle request tracking the measured angle, identical to the (blocked) camera frame except
  `LKA_SysIndReq` 1 vs 0. `SCC_CONTROL` and the cluster (0x161) never changed. Nothing in the log explains the "held
  off-center" feel; the angle request tracks the MDPS angle (0xEA), which reads ~7 % larger than 0x125.
- Calibration: calibrated, 100 %, 15 blocks, pitch 0.137 rad (within limits).
- Door bit verified (three open/close cycles), seatbelt buckled at 192.8 s, gear P→R→N→D→N→R→D→N→R→P all decoded.

### Also confirmed on this run

- `STEERING_SENSORS` 0x125: continuous, 100 Hz, valid; full-lock sweep +510.6° (t=68.6) / −509.9° (t=74.8), mirrored in `carState.steeringAngleDeg`.
- No panda faults, no relay malfunction. Alerts were only `startupMaster` and the permanent `canError`; events `canError`, `controlsMismatch`, `locationdTemporaryError` (side effect of invalid carState).
- Benign log noise: "car doesn't match any Neural Network model", iso-tp bad responses during the FW query, athenad websocket exceptions.

## Open issue 2 — resolved

Bus mapping is no longer an open issue. See "Bus mapping — resolved" above: the flip is correct, matches the reference car's topology, and needs no code change.

## First road test (`00000011--6f004f6f07`, lane lines, 15-25 mph) and override tuning

122 s active, one LFA engage, no alerts, no panda blocks. Hands-off tracking: command minus MDPS angle mean 0.03°, std 0.38°,
lag under 50 ms, no oscillation, command rate p99 10°/s. The model put the car 1-16 cm right of lane center (not left);
the wheel held +0.9° (left) on straights, which the angle-offset learner (reset that drive, +0.27° by the end) had not
absorbed. Both `paramsd` and `calibrationd` reset at the start of that drive; calibration finished at 98 s and its yaw
ended ~1° from the previous drive, so the perceived left bias is unconfirmed until calibration settles.

**Override stiffness.** `STEER_THRESHOLD` is 175 torque units for every `CANFD_ANGLE_STEERING` car and tripped promptly
(peaks 185-541, similar scale to the Sportage HEV reference route, median override peak 392). The stiff feel came from
`compute_torque_reduction_gain`: at 23 mph the stock table only reaches its floor (~0.19) at ~525 units, so 400-470 unit
pushes left the gain at 0.25-0.40. `HyundaiFlags.CANFD_FAST_OVERRIDE_HANDOFF` (LX3 only) keeps the ceiling, shelf, floor and
the nudge region (bp1/bp2) but starts the drop at 180-200 units (always above the 175 override threshold) and reaches
the floor at 240-360 units instead of 400-700. Panda untouched (it only checks raw gain <= 250 and zero while inactive).
Trade-off: torque spikes above ~200 (rough road) now cut authority sooner, and the recovery ramp (+0.004/frame) is
unchanged, so a brief spike costs up to ~0.5 s of reduced assist. Tests: `tests/test_torque_reduction_gain.py`.

## Stock HDA takes lateral while stock ACC is engaged (`00000018--30cfc70d81`)

How steering reaches the MDPS on this car: our `LKAS_ALT` (0x110) goes out on A-CAN, the ADRV re-transmits it on E-CAN
as `LFA_ALT` (0xCB), and the MDPS steers to 0xCB only. With MADS alone the relay is byte-faithful (angle within 0.3°,
gain identical, one-frame lag; 0xCB has a +1 counter and valid checksum on every frame). 0.7 s after stock ACC engaged
(`ACCMode` 1 at 104.60 s) the ADRV lit `HDA_ICON`, switched `LFA_ICON` to 2 and **substituted its own request on 0xCB**
(angle 3-8° from ours, gain 0.0-0.4 while we sent 0.85). The MDPS followed 0xCB to 0.44° and ignored us by 3.4° on
average; driver torque p90 rose from 65 to 141. The camera's own 0x110 stayed inactive throughout, so the request came
from the ADRV. Substitution ended ~3 s after the brake cancel. The green wheel (`LFA_ICON` = 2 in 0x1E0, an ADRV message
we cannot override) mirrors our active steering once main cruise has been on; it stayed lit because `MainMode_ACC` was
left on until engine off. On the Sportage HEV reference route the ADRV kept relaying openpilot through ACC-on driving,
so the 0x362 "no lane lines" spoof (which is sent on the LX3 too, same byte layout) is not enough here. Likely trigger:
our own 0x110 reporting LFA active (`LKA_RcgSta` 3, `LKAS_ANGLE_ACTIVE` 2, `LKA_SysIndReq` 2 once openpilot is enabled).

**Implemented (opendbc + sunnypilot, no panda changes), gated on `HyundaiFlagsSP.CANFD_ADRV_LATERAL_TAKEOVER`:**

1. `carStateSP.stockLateralActive` = `cruiseState.enabled`. MADS raises `stockLateralActive` ("openpilot steering paused /
   Steer manually"), goes to *paused* whether or not openpilot is PCM-enabled, refuses LFA presses while it lasts
   (`lkasBlockedByStockLateral`: "openpilot Unavailable / Cancel cruise to resume steering", no state change so cancel
   restores the pre-cruise MADS state), and resumes silently when ACC disengages.
2. Relay watchdog (`opendbc/sunnypilot/car/hyundai/adrv_relay.py`): 0xCB vs a *model* of what a faithful ADRV would relay
   from the last 0x110 we sent. The relay is not a byte copy: it clamps the angle at ±176.7° and slews it at ~200°/s
   (route `0000001b`, 143.2-143.6 s and 257.3-257.6 s), while our command can move at 250-500°/s when unwinding an
   override or sprinting on a standstill re-engage. The model clamps at 176.7° and slews at 250°/s, tracking 0xCB while
   inactive. Mismatch = relay inactive while we command, or gain differs by > 0.10, or angle differs from the model by
   > 2° + 0.03 s × model rate. The comparison is skipped while `steeringPressed` and for 0.5 s after release (the MDPS
   follows the driver then, and both false trips on `0000001b` were inside or right after an override). 30 consecutive
   frames (0.3 s) raise `steerFaultTemporary` for 5 s ("Steering Assist Temporarily Unavailable", lateral dropped).
   Replayed against the logs: `0000001b` 0 trips and no mismatch streak ≥ 5 frames (the raw-command version tripped
   at 143.6 and 257.6 s); `00000018` trips at 109.0 s (substitution began 105.2 s, driver overriding until 107.6 s plus
   grace), 180.5 s (0.3 s after the second engage) and 195.7 s, then 222.1 and 259.4 s while the ADRV kept altering
   the relay with main cruise left on.
3. Override gain floor for `CANFD_FAST_OVERRIDE_HANDOFF` lowered to 0.10 at all speeds (stock: 0.10 at 4.5 mph rising to
   0.30 at 49 mph; on `00000017` the 0.18-0.28 floor still left the EPS fighting 400-600 unit overrides).

**Suppression experiments for later (each behind a flag, parked/low-speed first):**

- Send `LKA_RcgSta` 0 and keep `LKA_SysIndReq` 1 in our 0x110 while active; check the MDPS still follows the angle.
- Zero the byte 8/9 lane-quality fields of 0x362 in addition to byte 7.
- Compare which forwarded camera messages change at HDA arming to find the ADRV's lane-availability source.

**`steerTempUnavailableSilent`:** `MDPS_ADAS_AciFltSig_Lv2` = 4 for ~20 ms at crawl speed on `00000017` (84.7 s) and
`00000018` (163.0 s). Harmless so far; watch for it at speed.

## Handoff test (`0000001b--e709e280e9`, build cd5ba319ad)

The gate worked: `stockLateralActive` went true in the same 10 ms frame as `ACCMode` = 1 (156.72 s), MADS paused and our
0x110 went inactive 10 ms later, the alert text appeared after 200 ms, and lateral resumed 20 ms after the brake cancel.
**With our 0x110 inactive the ADRV never armed HDA** (`HDA_ICON` stayed 0, 0xCB stayed inactive), so nobody steered while
cruise was on; the alert now says "Steer manually". Two things were wrong and are fixed above: the watchdog tripped twice
on faithful-but-slewing relay behavior during overrides, and an LFA press while paused went through
`manualSteeringRequired` and switched MADS off, so lateral did not come back after cancel. Override gains with the 0.10
floor: -461 units at 10 mph reached 0.10 in 0.25 s, +400 at 20 mph in 0.70 s. Angle-offset learner: 0.89° → 0.39°
average, calibration stable (50 blocks). Traffic left no clean hands-off straight time for a lane-position number.
