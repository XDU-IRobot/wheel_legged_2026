/**
 * @file  targets/wheel_legged/input.cc
 * @brief 硬件输入采集与语义折叠实现
 */

#include "include/input.hpp"

#include <algorithm>
#include <cmath>

#include "include/actuators.hpp"
#include "include/globals.hpp"
#include "include/params.hpp"
namespace wheel_legged::control_loop {

namespace {

constexpr int16_t kDr16AxisMaxAbs = params::active::control_loop::kDr16AxisMaxAbs;
constexpr int16_t kWheelSpinThreshold = params::active::control_loop::kWheelSpinThreshold;
constexpr int16_t kWheelActionThreshold = params::active::control_loop::kWheelActionThreshold;
constexpr int16_t kWheelCenterThreshold = params::active::control_loop::kWheelCenterThreshold;
constexpr float kControlLoopDtS = params::active::control_loop::kControlLoopDtS;
constexpr float kRcStickMax = params::active::control_loop::kRcStickMax;
constexpr float kTcMouseMax = params::active::control_loop::kTcMouseMax;
constexpr float kRcYawRateMaxRadS = params::active::control_loop::kRcYawRateMaxRadS;
constexpr float kRcPitchRateMaxRadS = params::active::control_loop::kRcPitchRateMaxRadS;
constexpr float kTcMouseYawRateMaxRadS = params::active::control_loop::kTcMouseYawRateMaxRadS;
constexpr float kTcMousePitchRateMaxRadS = params::active::control_loop::kTcMousePitchRateMaxRadS;
constexpr float kDr16MouseMax = params::active::control_loop::kDr16MouseMax;
constexpr float kDr16MouseYawRateMaxRadS = params::active::control_loop::kDr16MouseYawRateMaxRadS;
constexpr float kDr16MousePitchRateMaxRadS = params::active::control_loop::kDr16MousePitchRateMaxRadS;
constexpr float kPitchTargetMinRad = params::active::control_loop::kPitchTargetMinRad;
constexpr float kPitchTargetMaxRad = params::active::control_loop::kPitchTargetMaxRad;

constexpr uint16_t kRcKeyW = 0x0001;
constexpr uint16_t kRcKeyS = 0x0002;
constexpr uint16_t kRcKeyA = 0x0004;
constexpr uint16_t kRcKeyD = 0x0008;
constexpr uint16_t kRcKeyShift = 0x0010;
constexpr uint16_t kRcKeyCtrl = 0x0020;
constexpr uint16_t kRcKeyQ = 0x0040;
constexpr uint16_t kRcKeyE = 0x0080;
constexpr uint16_t kRcKeyR = 0x0100;
constexpr uint16_t kRcKeyF = 0x0200;
constexpr uint16_t kRcKeyV = 0x4000;
constexpr uint16_t kRcKeyC = 0x2000;
constexpr uint16_t kRcKeyB = 0x8000;
constexpr uint16_t kRcKeyG = 0x0400;
constexpr uint16_t kRcKeyZ = 0x0800;
constexpr uint16_t kRcKeyX = 0x1000;

// 键盘输入斜坡：加速/减速使用不同步进
constexpr float kKeyboardAccelRampStep = params::active::control_loop::kKeyboardAccelRampStep;
constexpr float kKeyboardBrakeRampStep = params::active::control_loop::kKeyboardBrakeRampStep;

}  // namespace

float NormalizeDr16Axis(const int16_t axis, const int16_t axis_max_abs) {
  return rm::modules::Clamp(static_cast<float>(axis) / static_cast<float>(axis_max_abs), -1.0f, 1.0f);
}

namespace {

void RampToTarget(float target, float &current) {
  const bool magnitude_increasing = std::fabs(target) > std::fabs(current);
  const float step = magnitude_increasing ? kKeyboardAccelRampStep : kKeyboardBrakeRampStep;
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
}

float ResolveKeyboardAxis(uint16_t keys, uint16_t key_pos, uint16_t key_neg, float &ramp_state) {
  float target = 0.0f;
  if ((keys & key_pos) != 0U) {
    target = 1.0f;
  } else if ((keys & key_neg) != 0U) {
    target = -1.0f;
  }
  RampToTarget(target, ramp_state);
  return ramp_state;
}

/// 判断 combat_profile 是否为自瞄模式
bool IsAutoAimProfile(const CombatProfile p) {
  return p == CombatProfile::kAutoAimAmmo || p == CombatProfile::kAutoAimFuSmall || p == CombatProfile::kAutoAimFuBig;
}

/// 将 aim_mode 映射为 CombatProfile
wheel_legged::CombatProfile ResolveAutoAimProfile(const TcSemanticState::AimMode aim_mode) {
  switch (aim_mode) {
    case TcSemanticState::AimMode::kFuSmall:
      return wheel_legged::CombatProfile::kAutoAimFuSmall;
    case TcSemanticState::AimMode::kFuBig:
      return wheel_legged::CombatProfile::kAutoAimFuBig;
    case TcSemanticState::AimMode::kAmmo:
    default:
      return wheel_legged::CombatProfile::kAutoAimAmmo;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// 子函数 1：TC 键盘上升沿检测（原 ResolveInputSemantics:164-283）
// ═════════════════════════════════════════════════════════════════════════════

void ResolveTcKeyboardEdges(const TcRemoteInput &tc_remote, TcSemanticState &tc_state,
                            Dr16SemanticState &semantic_state, wheel_legged::StairTaskRequest &stair_task_request,
                            bool &r_yaw_reset_edge, bool &r_flip_180_edge) {
  const bool ctrl_pressed = (tc_remote.keyboard_value & kRcKeyCtrl) != 0U;

  // C 键（无 Ctrl）：对齐正方向后切换中/低腿长、取消台阶
  const bool c_pressed = (tc_remote.keyboard_value & kRcKeyC) != 0U;
  if (c_pressed && !ctrl_pressed && tc_state.mid_leg_c_armed) {
    r_yaw_reset_edge = true;
    tc_state.pending_action = TcSemanticState::PendingAction::kC;
    tc_state.mid_leg_c_armed = false;
  }
  if (!c_pressed) tc_state.mid_leg_c_armed = true;

  // G 键（上升沿）：循环切换 aim_mode
  const bool g_pressed = (tc_remote.keyboard_value & kRcKeyG) != 0U;
  if (g_pressed && tc_state.g_aim_armed) {
    tc_state.aim_mode = static_cast<TcSemanticState::AimMode>((static_cast<uint8_t>(tc_state.aim_mode) + 1) % 3);
    tc_state.g_aim_armed = false;
  }
  if (!g_pressed) tc_state.g_aim_armed = true;

  // Q: disabled <-> enabled
  const bool q_pressed = (tc_remote.keyboard_value & kRcKeyQ) != 0U;
  if (q_pressed && !ctrl_pressed && tc_state.q_domain_armed) {
    if (tc_state.domain_state == 1U) {
      tc_state.domain_state = 2U;
    } else {
      tc_state.domain_state = (tc_state.domain_state == 0U) ? 2U : 0U;
    }
    tc_state.q_domain_armed = false;
  }
  if (!q_pressed) tc_state.q_domain_armed = true;

  // Ctrl+Q: 切换 standby / enabled
  if (ctrl_pressed && q_pressed && tc_state.ctrl_q_standby_armed) {
    tc_state.domain_state = (tc_state.domain_state == 1U) ? 2U : 1U;
    tc_state.ctrl_q_standby_armed = false;
  }
  if (!ctrl_pressed || !q_pressed) tc_state.ctrl_q_standby_armed = true;

  // V 键：对齐正方向后启动单台阶任务
  const bool v_pressed = (tc_remote.keyboard_value & kRcKeyV) != 0U;
  if (v_pressed && tc_state.v_high_leg_armed) {
    r_yaw_reset_edge = true;
    tc_state.pending_action = TcSemanticState::PendingAction::kV;
    tc_state.v_high_leg_armed = false;
  }
  if (!v_pressed) tc_state.v_high_leg_armed = true;

  // B 键：对齐正方向后启动双台阶任务
  const bool b_pressed = (tc_remote.keyboard_value & kRcKeyB) != 0U;
  if (b_pressed && tc_state.b_high_leg_armed) {
    r_yaw_reset_edge = true;
    tc_state.pending_action = TcSemanticState::PendingAction::kB;
    tc_state.b_high_leg_armed = false;
  }
  if (!b_pressed) tc_state.b_high_leg_armed = true;

  // F 键（上升沿）：切换 mid_leg_f 中腿长模式
  const bool f_pressed = (tc_remote.keyboard_value & kRcKeyF) != 0U;
  if (f_pressed && tc_state.f_slow_armed) {
    tc_state.mid_leg_f = !tc_state.mid_leg_f;
    tc_state.mid_leg_hold = tc_state.mid_leg_f;
    tc_state.f_slow_armed = false;
  }
  if (!f_pressed) tc_state.f_slow_armed = true;

  // R 键（上升沿）：云台转 180°
  const bool r_pressed = (tc_remote.keyboard_value & kRcKeyR) != 0U;
  if (r_pressed && tc_state.r_flip_armed) {
    semantic_state.rc_target.yaw_rad = rm::modules::Wrap(semantic_state.rc_target.yaw_rad + params::active::kPi,
                                                         -params::active::kPi, params::active::kPi);
    r_flip_180_edge = true;
    tc_state.r_flip_armed = false;
  }
  if (!r_pressed) tc_state.r_flip_armed = true;

  const bool z_pressed = (tc_remote.keyboard_value & kRcKeyZ) != 0U;
  const bool x_pressed = (tc_remote.keyboard_value & kRcKeyX) != 0U;

  // E 键：UI 刷新使能（电平有效）
  tc_state.e_ui_refresh = (tc_remote.keyboard_value & kRcKeyE) != 0U;

  // X 键上升沿：只切换工作的 ToF 硬件对。进入下台阶模式时取消尚未触发的自动跳跃。
  if (x_pressed && tc_state.x_tof_mode_armed) {
    tc_state.x_tof_mode_armed = false;
    if (!ctrl_pressed && !tc_state.auto_jump_in_progress) {
      tc_state.requested_tof_mode = tc_state.requested_tof_mode == wheel_legged::TofMode::kAutoJump
                                        ? wheel_legged::TofMode::kStairDescend
                                        : wheel_legged::TofMode::kAutoJump;
      if (tc_state.requested_tof_mode == wheel_legged::TofMode::kStairDescend) {
        tc_state.auto_jump_enabled = false;
        tc_state.auto_jump_tof_armed = true;
      }
    }
  }
  if (!x_pressed) tc_state.x_tof_mode_armed = true;

  // Z 键上升沿：在前向 ToF 模式下启动一次自动跳跃；下台阶模式中屏蔽。
  if (z_pressed && !ctrl_pressed && tc_state.z_auto_jump_armed) {
    if (tc_state.requested_tof_mode == wheel_legged::TofMode::kAutoJump && !tc_state.auto_jump_enabled) {
      tc_state.auto_jump_enabled = true;
      tc_state.auto_jump_tof_armed = true;
    }
    tc_state.z_auto_jump_armed = false;
  }
  if (!z_pressed) tc_state.z_auto_jump_armed = true;
}

// ═════════════════════════════════════════════════════════════════════════════
// 子函数 2：DR16 拨杆→模式映射（原 ResolveInputSemantics:308-385）
// ═════════════════════════════════════════════════════════════════════════════

Dr16ModeResult ResolveDr16Mode(const Dr16RawInput &dr16, TcSemanticState &tc_state) {
  Dr16ModeResult r{};

  // 左拨杆 kDown + 右拨杆的组合用于云台辨识/验证
  if (dr16.switch_l == rm::device::DR16::SwitchPosition::kDown) {
    if (dr16.switch_r == rm::device::DR16::SwitchPosition::kUp) {
      r.domain = wheel_legged::DomainRequest::kService;
      r.gimbal_test = wheel_legged::GimbalTestProfile::kIdent;
    } else if (dr16.switch_r == rm::device::DR16::SwitchPosition::kMid) {
      r.domain = wheel_legged::DomainRequest::kService;
      r.gimbal_test = wheel_legged::GimbalTestProfile::kFfVerify;
    } else {
      r.domain = wheel_legged::DomainRequest::kDisabled;
    }
  } else {
    r.domain = ResolveDomainRequest(dr16.switch_l);
  }

  // 腿长 + 战斗子模式：右拨杆
  if (r.domain == wheel_legged::DomainRequest::kCombat) {
    switch (dr16.switch_r) {
      case rm::device::DR16::SwitchPosition::kDown:
        r.combat = wheel_legged::CombatProfile::kNormal;
        r.leg = wheel_legged::LegProfile::kLow;
        break;
      case rm::device::DR16::SwitchPosition::kMid:
        r.combat = wheel_legged::CombatProfile::kAutoAimAmmo;
        r.leg = wheel_legged::LegProfile::kLow;
        break;
      case rm::device::DR16::SwitchPosition::kUp:
        r.combat = wheel_legged::CombatProfile::kAutoAimFuSmall;
        r.leg = wheel_legged::LegProfile::kLow;
        break;
      default:
        break;
    }
  } else if (r.gimbal_test == wheel_legged::GimbalTestProfile::kNormal) {
    r.leg = ResolveLegProfile(dr16.switch_r);
  }

  // 拨轮：上拨跳跃，下拨小陀螺（仅左拨杆 kMid 时生效）
  if (dr16.switch_l == rm::device::DR16::SwitchPosition::kMid) {
    if (dr16.dial >= kWheelSpinThreshold && tc_state.dial_jump_armed) {
      r.jump_trigger = true;
      tc_state.dial_jump_armed = false;
    }
    if (dr16.dial < kWheelSpinThreshold) {
      tc_state.dial_jump_armed = true;
    }
    if (dr16.dial <= -kWheelActionThreshold) {
      r.spin_hold = true;
      r.spin_dir = -1.0f;
    }
  }

  return r;
}

// ═════════════════════════════════════════════════════════════════════════════
// 子函数 3：TC 键鼠→模式映射（原 ResolveInputSemantics:387-489）
// ═════════════════════════════════════════════════════════════════════════════

TcModeResult ResolveTcMode(const TcRemoteInput &tc_remote, TcSemanticState &tc_state) {
  TcModeResult r{};

  // 工作域：Q 键切换 disabled/enabled，Ctrl+Q 进入 standby
  switch (tc_state.domain_state) {
    case 0:
      r.domain = wheel_legged::DomainRequest::kDisabled;
      break;
    case 1:
      r.domain = wheel_legged::DomainRequest::kCombat;
      break;
    case 2:
    default:
      r.domain = wheel_legged::DomainRequest::kCombat;
      break;
  }
  const bool standby = (tc_state.domain_state == 1U);

  // 腿长
  r.leg = tc_state.mid_leg_hold ? wheel_legged::LegProfile::kMid : wheel_legged::LegProfile::kLow;

  // 小陀螺：Shift 键
  r.spin_hold = (tc_remote.keyboard_value & kRcKeyShift) != 0U;

  // 跳跃：鼠标滚轮下滚 < -70（低腿）
  {
    const bool mouse_z_trigger = (r.leg == wheel_legged::LegProfile::kLow && tc_remote.mouse_z < -70);
    const bool mouse_z_edge = mouse_z_trigger && tc_state.mouse_z_jump_armed;
    if (mouse_z_trigger) tc_state.mouse_z_jump_armed = false;
    if (tc_remote.mouse_z >= 0) tc_state.mouse_z_jump_armed = true;
    r.jump_trigger = mouse_z_edge;
  }

  // 将 standby 状态暂存到 domain 字段的注释里，由调用方处理
  // 实际 standby 由 ResolveInputSemantics 设置到 request.standby
  if (standby) {
    r.domain = wheel_legged::DomainRequest::kCombat;
  }
  // 注意：standby 信息通过 domain_state==1 传递，由调用方检查
  // 这里用一个技巧：当 domain_state==1 时 domain 仍为 kCombat，
  // 但 ResolveInputSemantics 需要知道 standby 状态。
  // 我们在返回结果中不直接编码 standby，而是在 ResolveInputSemantics 中处理。

  return r;
}

// ═════════════════════════════════════════════════════════════════════════════
// 子函数 4：云台 yaw/pitch 目标积分（原 ResolveInputSemantics:529-580）
// ═════════════════════════════════════════════════════════════════════════════

void IntegrateGimbalTarget(Dr16SemanticState &semantic_state, const Dr16RawInput &dr16, const TcRemoteInput &tc_remote,
                           bool tc_remote_active, float gimbal_imu_yaw_rad, float gimbal_imu_pitch_rad,
                           bool host_controls_gimbal) {
  if (!semantic_state.gimbal_target_initialized) {
    semantic_state.rc_target.yaw_rad = gimbal_imu_yaw_rad;
    semantic_state.rc_target.pitch_rad = gimbal_imu_pitch_rad;
    semantic_state.gimbal_target_initialized = true;
  }
  if (host_controls_gimbal) return;

  {
    float yaw_delta = 0.0f;
    float pitch_delta = 0.0f;
    if (dr16.online) {
      const bool dr16_mouse_active = (dr16.mouse_x != 0 || dr16.mouse_y != 0);
      if (dr16_mouse_active) {
        yaw_delta += static_cast<float>(dr16.mouse_x) / kDr16MouseMax * kDr16MouseYawRateMaxRadS * kControlLoopDtS;
        pitch_delta += static_cast<float>(dr16.mouse_y) / kDr16MouseMax * kDr16MousePitchRateMaxRadS * kControlLoopDtS;
      } else {
        yaw_delta += static_cast<float>(dr16.left_x) / kRcStickMax * kRcYawRateMaxRadS * kControlLoopDtS;
        pitch_delta += static_cast<float>(dr16.left_y) / kRcStickMax * kRcPitchRateMaxRadS * kControlLoopDtS;
      }
    } else if (tc_remote_active) {
      const float mouse_yaw =
          static_cast<float>(tc_remote.mouse_x) / kTcMouseMax * kTcMouseYawRateMaxRadS * kControlLoopDtS;
      const float mouse_pitch =
          static_cast<float>(tc_remote.mouse_y) / kTcMouseMax * kTcMousePitchRateMaxRadS * kControlLoopDtS;
      const bool mouse_active = (tc_remote.mouse_x != 0 || tc_remote.mouse_y != 0);
      if (mouse_active) {
        yaw_delta += mouse_yaw;
        pitch_delta += mouse_pitch;
      }
    }
    semantic_state.rc_target.yaw_rad =
        rm::modules::Wrap(semantic_state.rc_target.yaw_rad + yaw_delta, -params::active::kPi, params::active::kPi);
    semantic_state.rc_target.pitch_rad =
        std::clamp(semantic_state.rc_target.pitch_rad + pitch_delta, kPitchTargetMinRad, kPitchTargetMaxRad);
  }
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// 公开接口
// ═════════════════════════════════════════════════════════════════════════════

DriveInputNorm ResolveDriveInput(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote, DriveInputRampState &ramp) {
  DriveInputNorm out{};

  // DR16 优先 > VT03 键鼠
  if (dr16.online) {
    const uint16_t keys = dr16.keyboard;
    out.forward = ResolveKeyboardAxis(keys, kRcKeyW, kRcKeyS, ramp.forward);
    out.side = ResolveKeyboardAxis(keys, kRcKeyD, kRcKeyA, ramp.side);

    const bool keyboard_active = (keys & (kRcKeyW | kRcKeyS | kRcKeyA | kRcKeyD)) != 0U;
    if (!keyboard_active) {
      if (std::fabs(ramp.forward) < kKeyboardBrakeRampStep && std::fabs(ramp.side) < kKeyboardBrakeRampStep) {
        ramp.forward = 0.0f;
        ramp.side = 0.0f;
        out.forward = NormalizeDr16Axis(dr16.right_y, kDr16AxisMaxAbs);
        out.side = NormalizeDr16Axis(dr16.right_x, kDr16AxisMaxAbs);
      }
    }
  } else if (tc_remote.valid) {
    const uint16_t keys = tc_remote.keyboard_value;
    out.forward = ResolveKeyboardAxis(keys, kRcKeyW, kRcKeyS, ramp.forward);
    out.side = ResolveKeyboardAxis(keys, kRcKeyD, kRcKeyA, ramp.side);
  } else {
    ramp.forward = 0.0f;
    ramp.side = 0.0f;
  }
  return out;
}

wheel_legged::LegProfile ResolveLegProfile(const rm::device::DR16::SwitchPosition switch_r) {
  switch (switch_r) {
    case rm::device::DR16::SwitchPosition::kMid:
      return wheel_legged::LegProfile::kMid;
    case rm::device::DR16::SwitchPosition::kUp:
      return wheel_legged::LegProfile::kHigh;
    case rm::device::DR16::SwitchPosition::kDown:
    default:
      return wheel_legged::LegProfile::kLow;
  }
}

wheel_legged::DomainRequest ResolveDomainRequest(const rm::device::DR16::SwitchPosition switch_l) {
  switch (switch_l) {
    case rm::device::DR16::SwitchPosition::kUp:
      return wheel_legged::DomainRequest::kCombat;
    case rm::device::DR16::SwitchPosition::kMid:
      return wheel_legged::DomainRequest::kService;
    case rm::device::DR16::SwitchPosition::kDown:
    default:
      return wheel_legged::DomainRequest::kDisabled;
  }
}

void ResolveInputSemantics(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote, Dr16SemanticState &semantic_state,
                           TcSemanticState &tc_state, InputSnapshot &input) {
  const bool tc_remote_active = tc_remote.valid && !tc_remote.tc_from_dr16;
  const bool has_any_input = dr16.online || tc_remote.valid;

  // ── 1. TC 键盘边沿检测 ──
  bool r_yaw_reset_edge = false;
  bool r_flip_180_edge = false;
  wheel_legged::StairTaskRequest stair_task_request = wheel_legged::StairTaskRequest::kNone;
  if (tc_remote.valid) {
    ResolveTcKeyboardEdges(tc_remote, tc_state, semantic_state, stair_task_request, r_yaw_reset_edge, r_flip_180_edge);
  }
  tc_state.auto_aim_hold = tc_remote.right_button;

  input.input_valid = has_any_input;
  input.dr16 = dr16;
  input.tc_remote = tc_remote;

  // ── 2. 按优先级解析模式 ──
  wheel_legged::ModeRequest request{};
  request.input_valid = has_any_input;
  request.service_profile = wheel_legged::ServiceProfile::kChassisAndGimbalSafe;
  request.reset_yaw_request = r_yaw_reset_edge;
  request.flip_180_request = r_flip_180_edge;
  request.stair_task_request = stair_task_request;

  wheel_legged::CombatProfile combat_profile = wheel_legged::CombatProfile::kNormal;
  wheel_legged::LegProfile leg_request = wheel_legged::LegProfile::kLow;

  if (dr16.online) {
    auto r = ResolveDr16Mode(dr16, tc_state);
    request.domain_request = r.domain;
    request.gimbal_test_profile = r.gimbal_test;
    request.jump_trigger = r.jump_trigger;
    request.spin_hold = r.spin_hold;
    request.spin_dir = r.spin_dir;
    combat_profile = r.combat;
    leg_request = r.leg;

    // 鼠标右键按住时覆盖为自瞄模式
    if (tc_state.auto_aim_hold && request.domain_request == wheel_legged::DomainRequest::kCombat) {
      combat_profile = ResolveAutoAimProfile(tc_state.aim_mode);
    }
  } else if (tc_remote_active) {
    auto r = ResolveTcMode(tc_remote, tc_state);
    request.domain_request = r.domain;
    request.jump_trigger = r.jump_trigger;
    request.spin_hold = r.spin_hold;
    combat_profile = r.combat;
    leg_request = r.leg;

    // TC 路径的 standby 处理
    if (tc_state.domain_state == 1U) {
      request.standby = true;
    }

    // 战斗子模式：自瞄覆盖
    if (tc_state.auto_aim_hold) {
      combat_profile = ResolveAutoAimProfile(tc_state.aim_mode);
    }
  } else {
    request.domain_request = wheel_legged::DomainRequest::kDisabled;
    request.jump_trigger = false;
  }

  // ── 3. 组装剩余字段 ──
  request.leg_request = leg_request;
  if (leg_request == wheel_legged::LegProfile::kHigh &&
      request.stair_task_request == wheel_legged::StairTaskRequest::kNone) {
    request.stair_task_request = wheel_legged::StairTaskRequest::kArmContinuous;
  }
  request.mid_leg_f = tc_state.mid_leg_f;

  const bool is_auto_aim = IsAutoAimProfile(combat_profile);
  if (combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
      combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig) {
    request.standby = true;
  }
  if (request.domain_request == wheel_legged::DomainRequest::kDisabled) request.standby = false;

  request.combat_profile = combat_profile;

  // 目标来源
  request.target_source =
      is_auto_aim ? (request.host_target_valid ? wheel_legged::TargetSource::kHost : wheel_legged::TargetSource::kRc)
                  : wheel_legged::TargetSource::kRc;

  // 自瞄切出时重置积分
  if (!is_auto_aim && semantic_state.last_auto_aim) {
    semantic_state.gimbal_target_initialized = false;
  }
  semantic_state.last_auto_aim = is_auto_aim;

  // 云台目标积分
  const bool host_controls_gimbal = is_auto_aim && request.target_source == wheel_legged::TargetSource::kHost;
  const float yaw_motor_pos_rad = input.estimator_input.yaw_motor_rad;
  if (!has_any_input || request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    semantic_state.rc_target.yaw_rad = yaw_motor_pos_rad;
    semantic_state.rc_target.pitch_rad = 0.0f;
    semantic_state.gimbal_target_initialized = false;
  } else {
    IntegrateGimbalTarget(semantic_state, dr16, tc_remote, tc_remote_active, input.gimbal_imu_yaw_rad,
                          input.gimbal_imu_pitch_rad, host_controls_gimbal);
  }
  request.rc_target = semantic_state.rc_target;

  request.tick_ms = 0U;

  input.mode_request = request;
}

void UpdateRawFeedbackAndInputSnapshot(SharedResources &g, chassis_runtime::Actuators &actuators, InputSnapshot &input,
                                       Dr16SemanticState &semantic_state, TcSemanticState &tc_state,
                                       const uint32_t now_ms) {
  const bool previous_host_target_active =
      input.mode_request.target_source == wheel_legged::TargetSource::kHost && input.mode_request.host_target_valid;

  // 1. 从执行器采集关节/轮毂/IMU 反馈
  input.auto_jump_triggered = false;
  input.stair_descend_triggered = false;
  actuators.FillEstimatorInput(g, input.estimator_input);

  // 2. 读取 DR16 原始值
  const bool gimbal_rx_ready = g.gimbal_rx.has_value();
  const bool gimbal_rx_valid = gimbal_rx_ready && g.gimbal_rx->frame_count() > 0;
  const bool keyboard_rx_valid = gimbal_rx_ready && g.gimbal_rx->vt03_online();

  Dr16RawInput dr16{
      .online = (g.dr16.online_status() == rm::device::Device::kOk),
      .switch_l = g.dr16.switch_l(),
      .switch_r = g.dr16.switch_r(),
      .right_y = g.dr16.right_y(),
      .right_x = g.dr16.right_x(),
      .left_x = g.dr16.left_x(),
      .left_y = g.dr16.left_y(),
      .dial = g.dr16.dial(),
      .mouse_x = g.dr16.mouse_x(),
      .mouse_y = g.dr16.mouse_y(),
      .mouse_left = g.dr16.mouse_button_left(),
      .mouse_right = g.dr16.mouse_button_right(),
  };

  // DR16 键盘位掩码（使用命名常量）
  {
    uint16_t keys = 0;
    if (g.dr16.key(rm::device::DR16::Key::kW)) keys |= kRcKeyW;
    if (g.dr16.key(rm::device::DR16::Key::kS)) keys |= kRcKeyS;
    if (g.dr16.key(rm::device::DR16::Key::kA)) keys |= kRcKeyA;
    if (g.dr16.key(rm::device::DR16::Key::kD)) keys |= kRcKeyD;
    if (g.dr16.key(rm::device::DR16::Key::kShift)) keys |= kRcKeyShift;
    if (g.dr16.key(rm::device::DR16::Key::kCtrl)) keys |= kRcKeyCtrl;
    if (g.dr16.key(rm::device::DR16::Key::kQ)) keys |= kRcKeyQ;
    if (g.dr16.key(rm::device::DR16::Key::kE)) keys |= kRcKeyE;
    if (g.dr16.key(rm::device::DR16::Key::kR)) keys |= kRcKeyR;
    if (g.dr16.key(rm::device::DR16::Key::kF)) keys |= kRcKeyF;
    if (g.dr16.key(rm::device::DR16::Key::kG)) keys |= kRcKeyG;
    if (g.dr16.key(rm::device::DR16::Key::kZ)) keys |= kRcKeyZ;
    if (g.dr16.key(rm::device::DR16::Key::kX)) keys |= kRcKeyX;
    if (g.dr16.key(rm::device::DR16::Key::kC)) keys |= kRcKeyC;
    if (g.dr16.key(rm::device::DR16::Key::kV)) keys |= kRcKeyV;
    if (g.dr16.key(rm::device::DR16::Key::kB)) keys |= kRcKeyB;
    dr16.keyboard = keys;
  }

  // 键鼠数据：VT03 优先，否则 DR16
  TcRemoteInput tc_remote{};
  if (keyboard_rx_valid) {
    tc_remote.valid = true;
    tc_remote.mouse_x = g.gimbal_rx->mouse_x();
    tc_remote.mouse_y = g.gimbal_rx->mouse_y();
    tc_remote.mouse_z = g.gimbal_rx->mouse_z();
    tc_remote.left_button = g.gimbal_rx->left_button();
    tc_remote.right_button = g.gimbal_rx->right_button();
    tc_remote.keyboard_value = g.gimbal_rx->keyboard_value();
  } else if (dr16.online) {
    tc_remote.valid = true;
    tc_remote.tc_from_dr16 = true;
    tc_remote.mouse_x = dr16.mouse_x;
    tc_remote.mouse_y = dr16.mouse_y;
    tc_remote.left_button = dr16.mouse_left;
    tc_remote.right_button = dr16.mouse_right;
    tc_remote.keyboard_value = dr16.keyboard;
  }

  // 3a. 使能边沿检测：从 kDisabled 进入使能域时，腿长状态全部归零
  {
    const bool tc_active = tc_remote.valid && !tc_remote.tc_from_dr16;
    wheel_legged::DomainRequest predicted_domain = wheel_legged::DomainRequest::kDisabled;
    if (dr16.online) {
      predicted_domain = ResolveDomainRequest(dr16.switch_l);
    } else if (tc_active) {
      predicted_domain =
          (tc_state.domain_state != 0) ? wheel_legged::DomainRequest::kCombat : wheel_legged::DomainRequest::kDisabled;
    }
    if (tc_state.prev_domain == wheel_legged::DomainRequest::kDisabled &&
        predicted_domain != wheel_legged::DomainRequest::kDisabled) {
      tc_state.mid_leg_hold = false;
      tc_state.mid_leg_f = false;
      tc_state.aim_mode = TcSemanticState::AimMode::kAmmo;
    }
    tc_state.prev_domain = predicted_domain;
  }

  // 3b. 语义折叠
  ResolveInputSemantics(dr16, tc_remote, semantic_state, tc_state, input);

  // 预留给未来下台阶控制：动作完成后置位即可自动退回默认前向 ToF。
  if (tc_state.stair_descend_completed) {
    tc_state.requested_tof_mode = wheel_legged::TofMode::kAutoJump;
    tc_state.stair_descend_in_progress = false;
    tc_state.stair_descend_completed = false;
  }
  g.requested_tof_mode = tc_state.requested_tof_mode;

  // 下台阶：向下 ToF 切换完成后才允许进入 0.25 m 接近阶段；单帧双侧小于阈值即触发。
  input.mode_request.stair_descend_request = tc_state.requested_tof_mode == wheel_legged::TofMode::kStairDescend;
  input.mode_request.stair_descend_ready = false;
  input.mode_request.stair_descend_trigger = false;
  if (input.mode_request.stair_descend_request && g.active_tof_mode == wheel_legged::TofMode::kStairDescend &&
      g.tof_mode_ready && g.left_down_tof.has_value() && g.right_down_tof.has_value()) {
    const auto &left = *g.left_down_tof;
    const auto &right = *g.right_down_tof;
    const uint32_t now_ms = HAL_GetTick();
    const bool measurements_fresh =
        left.sample_count() > 0U && right.sample_count() > 0U &&
        now_ms - left.last_sample_tick_ms() <= params::active::tof::kStairDescendFreshTimeoutMs &&
        now_ms - right.last_sample_tick_ms() <= params::active::tof::kStairDescendFreshTimeoutMs;
    const bool measurements_valid = left.ranging() && right.ranging() && left.data_valid() && right.data_valid();
    input.mode_request.stair_descend_ready = measurements_valid && measurements_fresh;
    input.mode_request.stair_descend_trigger =
        input.mode_request.stair_descend_ready &&
        left.measurement().distance_mm < params::active::tof::kStairDescendTriggerDistanceMm &&
        right.measurement().distance_mm < params::active::tof::kStairDescendTriggerDistanceMm;
    input.stair_descend_triggered = input.mode_request.stair_descend_trigger;
  }

  // 3c. 一次性自动跳跃：仅使用已就绪且数据有效、新鲜的两个前向 ToF。
  if (tc_state.requested_tof_mode == wheel_legged::TofMode::kAutoJump &&
      g.active_tof_mode == wheel_legged::TofMode::kAutoJump && g.tof_mode_ready) {
    const bool tof_ready = g.left_front_tof.has_value() && g.right_front_tof.has_value();
    if (tof_ready) {
      const auto &left = *g.left_front_tof;
      const auto &right = *g.right_front_tof;
      const bool measurements_valid = left.ranging() && right.ranging() && left.data_valid() && right.data_valid();
      const bool both_close = measurements_valid &&
                              (left.measurement().distance_mm + right.measurement().distance_mm) / 2 <
                                  params::active::tof::kAutoJumpTriggerDistanceMm &&
                              (left.measurement().distance_mm + right.measurement().distance_mm) / 2 >
                                  params::active::tof::kAutoJumpMinDistanceMm;
      const bool both_range_ok = left.measurement().range_status == 0 && right.measurement().range_status == 0;
      if (both_range_ok) {
        if (tc_state.both_active_start_ms == 0) tc_state.both_active_start_ms = now_ms;
      } else {
        tc_state.both_active_start_ms = 0;
      }
      const bool both_active = both_range_ok && (now_ms - tc_state.both_active_start_ms >=
                                                 params::active::tof::kAutoJumpBothActiveDurationMs);
      input.auto_jump_both_close = both_close;
      input.auto_jump_tof_armed_debug = tc_state.auto_jump_tof_armed;
      input.auto_jump_both_active = both_active;
      input.auto_jump_trigger_ready = both_close && tc_state.auto_jump_tof_armed && both_active;
      if (tc_state.auto_jump_enabled) {
        if (both_close && tc_state.auto_jump_tof_armed && both_active) {
          input.mode_request.jump_trigger = true;
          input.auto_jump_triggered = true;
          tc_state.auto_jump_tof_armed = false;
          tc_state.both_active_start_ms = 0;
        }
        const bool both_clear = measurements_valid &&
                                left.measurement().distance_mm > params::active::tof::kAutoJumpRearmDistanceMm &&
                                right.measurement().distance_mm > params::active::tof::kAutoJumpRearmDistanceMm;
        if (both_clear) tc_state.auto_jump_tof_armed = true;
      }
    }
  }

  input.auto_jump_enabled = tc_state.auto_jump_enabled;

  // 3d. 自瞄上位机目标（NUC 反馈 → host_target，仅在自瞄模式下生效）
  const bool auto_aim_active = IsAutoAimProfile(input.mode_request.combat_profile);
  const bool host_target_available = g.aimbot.has_value() && g.aimbot->online_status() == rm::device::Device::kOk &&
                                     g.aimbot->nuc_start_flag() != 0 && auto_aim_active &&
                                     (g.aimbot->aimbot_state() & 0x01U) != 0U;
  if (previous_host_target_active && auto_aim_active && !host_target_available && gimbal_rx_valid) {
    semantic_state.rc_target.yaw_rad = g.gimbal_rx->yaw_rad();
    semantic_state.rc_target.pitch_rad = g.gimbal_rx->pitch_rad();
    semantic_state.gimbal_target_initialized = true;
    input.mode_request.rc_target = semantic_state.rc_target;
  }
  if (host_target_available) {
    constexpr float kDegToRad = params::active::kPi / 180.0f;
    input.mode_request.host_target.yaw_rad = g.aimbot->yaw() * kDegToRad;
    input.mode_request.host_target.pitch_rad = -g.aimbot->pitch() * kDegToRad;
    input.mode_request.host_target_valid = true;
    input.mode_request.target_source = wheel_legged::TargetSource::kHost;
  }

  // 4. 云台惯导（CAN 桥，独立于底盘 IMU）
  input.gimbal_imu_yaw_rad = gimbal_rx_valid ? g.gimbal_rx->yaw_rad() : 0.0f;
  input.gimbal_imu_pitch_rad = gimbal_rx_valid ? g.gimbal_rx->pitch_rad() : 0.0f;
  input.gimbal_imu_gyro_z_rad_s = gimbal_rx_valid ? g.gimbal_rx->gyro_z_rad_s() : 0.0f;
  input.gimbal_imu_gyro_x_rad_s = gimbal_rx_valid ? g.gimbal_rx->gyro_x_rad_s() : 0.0f;
}

chassis::Fsm::Input BuildChassisFsmInput(const InputSnapshot &input, const uint32_t tick_ms,
                                         const chassis::Chassis::UpdateOutput &chassis_output, uint32_t &fall_start_ms,
                                         bool &was_posture_invalid) {
  // 倒地检测：基于上周期底盘姿态（posture_valid 在 Chassis::Update 中计算，FSM 之前运行，因此用上一周期值）

  const bool fall_detected = !chassis_output.posture_valid;
  uint32_t fall_detected_hold_ms = 0;
  bool upright_stable = false;

  if (fall_detected) {
    if (!was_posture_invalid) {
      fall_start_ms = tick_ms;
    }
    fall_detected_hold_ms = tick_ms - fall_start_ms;
  } else if (was_posture_invalid) {
    upright_stable = true;
  }
  was_posture_invalid = fall_detected;

  chassis::Fsm::Input fsm_input{};
  const auto &m = input.mode_request;
  fsm_input.request = {
      .input_valid = m.input_valid,
      .domain_request = m.domain_request,
      .leg_request = m.leg_request,
      .combat_profile = m.combat_profile,
      .standby = m.standby,
      .spin_hold = m.spin_hold,
      .spin_dir = m.spin_dir,
      .jump_trigger = m.jump_trigger,
      .stair_descend_request = m.stair_descend_request,
      .stair_descend_ready = m.stair_descend_ready,
      .stair_descend_trigger = m.stair_descend_trigger,
      .current_leg_length_m = chassis_output.mean_leg_length_m,
      .theta_ll_rad = chassis_output.current_state.theta_ll,
      .theta_lr_rad = chassis_output.current_state.theta_lr,
      .fall_detected = fall_detected,
      .fall_detected_hold_ms = fall_detected_hold_ms,
      .upright_stable = upright_stable,
      .tick_ms = tick_ms,
  };
  fsm_input.request.current_s_dot = chassis_output.current_state.s_dot;
  fsm_input.request.stair_step2 = m.stair_step2;
  return fsm_input;
}

gimbal::Fsm::Input BuildGimbalFsmInput(const InputSnapshot &input, const chassis::Fsm::Output &chassis_output,
                                       const bool startup_align_complete) {
  gimbal::Fsm::Input fsm_input{};
  const auto &m = input.mode_request;
  fsm_input.request = {
      .input_valid = m.input_valid,
      .domain_request = m.domain_request,
      .service_profile = m.service_profile,
      .combat_profile = m.combat_profile,
      .target_source = m.target_source,
      .rc_target = m.rc_target,
      .host_target = m.host_target,
      .host_target_valid = m.host_target_valid,
      .gimbal_test_profile = m.gimbal_test_profile,
      .chassis_recovery_active = chassis_output.mode == chassis::Fsm::State::kRecoveryFallCheck ||
                                 chassis_output.mode == chassis::Fsm::State::kRecoverySelfRight,
      .startup_align_complete = startup_align_complete,
  };
  return fsm_input;
}

}  // namespace wheel_legged::control_loop
