"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.hyundai.adrv_relay import AdrvRelayWatchdog

MDPS_ACI_FAULT_DEBOUNCE_FRAMES = 10  # 0.1 s at 100 Hz; the auto start-stop blips last one frame


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0

    # last lateral command the CarController sent: (angle deg, torque-reduction gain, active). Written by the
    # controller each frame so the ADRV relay watchdog below can compare it with what the MDPS actually received.
    self.op_lat_cmd = (0.0, 0.0, False)
    self.adrv_relay_watchdog = AdrvRelayWatchdog()
    self.mdps_aci_fault_frames = 0

  def update_speed_limit(self, cp, cp_cam) -> float:
    speed_limit = 0

    if self.CP.flags & HyundaiFlags.CANFD:
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        bus = cp if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else cp_cam
        speed_limit = bus.vl["FR_CMR_02_100ms"]["ISLW_SpdCluMainDis"]
    else:
      nav, cam = 0, 0
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        nav = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      if self.CP_SP.flags & HyundaiFlagsSP.HAS_LKAS12:
        cam = cp_cam.vl["LKAS12"]["CF_Lkas_TsrSpeed_Display_Clu"]

      speed_limit = cam if cam not in (0, 255) else nav

    if speed_limit in (0, 255):
      speed_limit = 0

    return speed_limit

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser], speed_conv: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      cruise_msg = "LABEL11" if self.CP.flags & HyundaiFlags.EV else \
                   "E_CRUISE_CONTROL" if self.CP.flags & HyundaiFlags.HYBRID else \
                   "EMS16"
      cruise_available_sig = "CC_React" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_M"
      cruise_enabled_sig = "CC_ACT" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_S"
      cruise_speed_msg = "E_EMS11" if self.CP.flags & HyundaiFlags.EV else \
                         "ELECT_GEAR" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "LVR12"
      cruise_speed_sig = "Cruise_Limit_Target" if self.CP.flags & HyundaiFlags.EV else \
                         "SLC_SET_SPEED" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "CF_Lvr_CruiseSet"
      ret.cruiseState.available = cp.vl[cruise_msg][cruise_available_sig] != 0
      ret.cruiseState.enabled = cp.vl[cruise_msg][cruise_enabled_sig] != 0
      ret.cruiseState.speed = cp.vl[cruise_speed_msg][cruise_speed_sig] * speed_conv
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False

      if not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_NO_FCA:
        cp_cruise = cp if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_RADAR_FCA else cp_cam

        aeb_src = "FCA11"
        aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
        aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src]["FCA_CmdAct"] != 0
        ret.stockFcw = aeb_warning and not aeb_braking
        ret.stockAeb = aeb_warning and aeb_braking

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_conv

  def update_canfd_ext(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser],
                       speed_factor: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS"]["aBasis"]

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_factor

    if self.CP_SP.flags & HyundaiFlagsSP.CANFD_ADRV_LATERAL_TAKEOVER:
      # 1. While stock ACC is engaged the ADRV owns lateral (it arms HDA and substitutes its own LFA_ALT request within
      #    ~0.7 s of ACCMode going to 1, and the MDPS follows it, not us). Tell MADS to pause lateral for that time.
      #    With an HDA suppression experiment selected the gate is bypassed on purpose: openpilot keeps steering
      #    through ACC engage and the relay watchdog below is the only protection.
      experiment = bool(self.CP_SP.flags & (HyundaiFlagsSP.CANFD_HDA_EXP_LFA_STATUS | HyundaiFlagsSP.CANFD_HDA_EXP_LANE_BYTES))
      ret_sp.stockLateralActive = ret.cruiseState.enabled and not experiment

      # 2. Relay watchdog: LFA_ALT (0xCB) on E-CAN is what the MDPS steers to. Normally it is a byte-faithful relay of
      #    our last LKAS_ALT. If it stops matching while we are commanding, we have lost authority: raise the
      #    steer-unavailable fault so lateral drops with an alert. See adrv_relay.py for the thresholds and evidence.
      relay = cp.vl["LFA_ALT"]
      cmd_angle, cmd_gain, cmd_active = self.op_lat_cmd
      relay_fault = self.adrv_relay_watchdog.update(cmd_angle, cmd_gain, cmd_active,
                                                    relay["ADAS_StrAnglReqVal"], relay["ADAS_ACIAnglTqRedcGainVal"],
                                                    relay["ADAS_ActvACILvl2Sta"] == 2, ret.steeringPressed)
      # 3. MDPS_ADAS_AciFltSig_Lv2 blips for a single frame during engine auto start-stop restarts at standstill: on
      #    route 69fb86b6677ce882/00000035--a701ae3a3f both blips (490.43 and 589.84 s, 0 mph, ACC stop-and-go) sit
      #    0.3 s before the same set of ECU status bits that flip at every engine restart, and each produced an
      #    audible "Steering Assist Temporarily Unavailable". No documented engine-state signal exists in the CAN-FD
      #    DBC to gate on, so require the flag to persist for 0.1 s instead; the genuine 1.9 s fault at MDPS power-up
      #    still comes through. MDPS_LkaFailSta is not debounced.
      aci_fault = cp.vl["MDPS"]["MDPS_ADAS_AciFltSig_Lv2"] != 0
      self.mdps_aci_fault_frames = self.mdps_aci_fault_frames + 1 if aci_fault else 0
      lka_fail = cp.vl["MDPS"]["MDPS_LkaFailSta"] != 0
      ret.steerFaultTemporary = lka_fail or (self.mdps_aci_fault_frames >= MDPS_ACI_FAULT_DEBOUNCE_FRAMES) or relay_fault
