"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class HyundaiSafetyFlagsSP:
  DEFAULT = 0
  ESCC = 1
  LONG_MAIN_CRUISE_TOGGLEABLE = 2
  HAS_LDA_BUTTON = 4
  NON_SCC = 8
  # bits 4-7 carry the angle steering model id (see below)
  CANFD_HALF_RATE_COUNTERS = 256  # ACCELERATOR_BRAKE_ALT (0x100) counter advances by 2 per frame (2026 Palisade LX3)
  CANFD_ALT_WHEEL_BUTTONS = 512  # buttons (LFA/RES/SET/main) come from WHEEL_BUTTONS_ALT (0x10B), not 0x1AA (2026 Palisade LX3)


# Angle steering vehicle model IDs — encoded in safety_param_sp bits [4:7].
# Must stay in sync with the C enum in hyundai_canfd_angle_models.h.
ANGLE_MODEL_SHIFT = 4
ANGLE_MODEL_MASK = 0xF


class HyundaiAngleSteeringModel:
  BASELINE = 0                    # fallback: uses KIA_SPORTAGE_HEV_2026 (most conservative)
  KIA_SPORTAGE_HEV_2026 = 1      # baseline vehicle
  HYUNDAI_IONIQ_5_PE = 2
  KIA_EV6_2025 = 3
  KIA_EV9 = 4
  GENESIS_GV80_2025 = 5
  HYUNDAI_SANTA_FE_HEV_5TH = 6
  HYUNDAI_IONIQ_9 = 7
  KIA_SORENTO_HEV_4TH_LFA2 = 8
  GENESIS_GV70_E_2ND_GEN = 9
  HYUNDAI_AZERA_HEV_7TH = 10
  HYUNDAI_PALISADE_LX3 = 11


# Mapping from CAR platform name → angle steering model ID.
# Platforms not in this map will use BASELINE (0) on the panda.
ANGLE_STEERING_MODEL_BY_CAR: dict[str, int] = {
  "KIA_SPORTAGE_HEV_2026":              HyundaiAngleSteeringModel.KIA_SPORTAGE_HEV_2026,
  "HYUNDAI_IONIQ_5_PE":                 HyundaiAngleSteeringModel.HYUNDAI_IONIQ_5_PE,
  "KIA_EV6_2025":                       HyundaiAngleSteeringModel.KIA_EV6_2025,
  "KIA_EV9":                            HyundaiAngleSteeringModel.KIA_EV9,
  "GENESIS_GV80_2025":                  HyundaiAngleSteeringModel.GENESIS_GV80_2025,
  "HYUNDAI_SANTA_FE_HEV_5TH_GEN":      HyundaiAngleSteeringModel.HYUNDAI_SANTA_FE_HEV_5TH,
  "HYUNDAI_IONIQ_9":                    HyundaiAngleSteeringModel.HYUNDAI_IONIQ_9,
  "KIA_SORENTO_HEV_4TH_GEN_LFA2":      HyundaiAngleSteeringModel.KIA_SORENTO_HEV_4TH_LFA2,
  "GENESIS_GV70_ELECTRIFIED_2ND_GEN":   HyundaiAngleSteeringModel.GENESIS_GV70_E_2ND_GEN,
  "HYUNDAI_AZERA_HEV_7TH_GEN":         HyundaiAngleSteeringModel.HYUNDAI_AZERA_HEV_7TH,
  "HYUNDAI_PALISADE_LX3":               HyundaiAngleSteeringModel.HYUNDAI_PALISADE_LX3,
}


def encode_angle_model_id(model_id: int) -> int:
  """Encode an angle steering model ID into the safety_param_sp bit field."""
  return (model_id & ANGLE_MODEL_MASK) << ANGLE_MODEL_SHIFT


class HyundaiFlagsSP(IntFlag):
  """
    Flags for Hyundai specific quirks within sunnypilot.
  """
  ENHANCED_SCC = 1
  HAS_LFA_BUTTON = 2  # Deprecated in favor of HyundaiFlags.HAS_LDA_BUTTON
  LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE = 2 ** 2
  ENABLE_RADAR_TRACKS_DEPRECATED = 2 ** 3
  LONG_TUNING_DYNAMIC = 2 ** 4
  LONG_TUNING_PREDICTIVE = 2 ** 5
  NON_SCC = 2 ** 6
  NON_SCC_RADAR_FCA = 2 ** 7  # most with FCA come from the camera
  NON_SCC_NO_FCA = 2 ** 8  # not all have FCA
  SPEED_LIMIT_AVAILABLE = 2 ** 9  # platforms with speed limit data available
  HAS_LKAS12 = 2 ** 10
  # The ADAS ECU (ADRV) relays our LKAS_ALT (A-CAN 0x110) to the MDPS as LFA_ALT (E-CAN 0xCB) and, once stock ACC is
  # engaged, arms HDA and substitutes its own lateral request on 0xCB. openpilot lateral must yield while stock ACC is
  # engaged and watch the relay. Seen on the 2026 Palisade (LX3), route 69fb86b6677ce882/00000018--30cfc70d81.
  CANFD_ADRV_LATERAL_TAKEOVER = 2 ** 11
  # HDA suppression experiments (opt-in, LX3 only, see LX3_FINDINGS.md). Either bit bypasses the stock-cruise lateral
  # gate; the ADRV relay watchdog stays armed so a takeover still drops lateral with an alert.
  CANFD_HDA_EXP_LFA_STATUS = 2 ** 12   # A: report LKA_RcgSta 0 and LKA_SysIndReq 1 in our LKAS_ALT while active
  CANFD_HDA_EXP_LANE_BYTES = 2 ** 13   # B: also zero bytes 8 and 9 of the CAM_0x362 lane-line spoof
