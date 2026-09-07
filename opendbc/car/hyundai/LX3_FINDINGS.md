# Palisade LX3 (2026, gas, HDA II/LFA2) — porting findings

Owner's vehicle: 2026 Hyundai Palisade Calligraphy, gas (non-hybrid), LX3 platform, HDA2, CAN-FD, comma 3X.
Harness: comma Hyundai N with the CAN H/L pairs manually swapped from stock pinout. **Confirmed correct** — see "Bus mapping" below. Keep the flip.

Routes referenced:
- Before pin flip: `69fb86b6677ce882/00000000--3284f58264/0`
- After pin flip: `69fb86b6677ce882/00000003--94eb544029` (3 segments)
- Deliberate button-press test: `69fb86b6677ce882/00000008--c7bfd877d8` (7 segments)
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

**Side effect worth knowing:** because `lka_steering` auto-detects `True` for this car, it will *also* pick up `HyundaiFlags.CANFD_LKA_STEER_MSG` at runtime (dynamically — not something set in `values.py`). For lateral-only operation (`openpilotLongitudinalControl=False`), this means `carcontroller.py` sends steering as `"LKAS"`/`"LKAS_ALT"` (not `"LFA"`), and does **not** send the `"LFA"` (0x12A) or `LFAHDA_CLUSTER` (0x1E0) messages at all — see the revised Open issue 1 below.

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

## Open issue 1 (still open — needs on-car verification, not yet a safety wiring decision)

Earlier analysis (before the bus mapping was resolved) assumed `carcontroller.py` unconditionally transmits both `LFAHDA_CLUSTER` (0x1E0) and the `"LFA"` message (0x12A) once this car is recognized, which would make reading `LFA_ICON` back circular (we'd just be reading what `mads.py` computed from `CC.enabled`, not the real button).

Now that bus mapping is resolved, this looks less likely to be a problem for a **lateral-only** setup:
- `create_lfahda_cluster` is only called `if not lka_steering or lka_steering_long`. With `lka_steering=True` (confirmed above) and `openpilotLongitudinalControl=False`, `lka_steering_long` is `False`, so this evaluates to `False` — **`LFAHDA_CLUSTER` would not be sent by us at all.**
- The `"LFA"` (0x12A) message is only sent by us `if CP.openpilotLongitudinalControl` — also **not sent**, for lateral-only.
- So in lateral-only mode with `lka_steering=True`, our own code shouldn't be writing to either message the original `LFA_ICON` finding relied on, meaning that finding may be genuine rather than circular after all.

**This has been reasoned out twice now from static code reading alone, and reversed once already — it needs to be confirmed with the branch actually running on the car before it's trusted as a safety trigger.** Concretely: after flashing (see deployment steps), watch `LFAHDA_CLUSTER` and `CCNC_0x161` on the bus *without ever pressing engage* and confirm neither one is being written by the device (values keep changing/matching real button presses even though our process is running) before wiring anything to them.

## Open issue 2 — resolved

Bus mapping is no longer an open issue. See "Bus mapping — resolved" above: the flip is correct, matches the reference car's topology, and needs no code change.
