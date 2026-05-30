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
f32 flag2, flag3, f4, f5;
namespace wheel_legged::control_loop {

namespace {

constexpr int16_t kDr16AxisMaxAbs = params::active::control_loop::kDr16AxisMaxAbs;
constexpr int16_t kWheelSpinThreshold = params::active::control_loop::kWheelSpinThreshold;
constexpr int16_t kWheelActionThreshold = params::active::control_loop::kWheelActionThreshold;
constexpr int16_t kWheelCenterThreshold = params::active::control_loop::kWheelCenterThreshold;
constexpr uint16_t kAutoJumpDistanceThresholdMm = params::active::control_loop::kAutoJumpDistanceThresholdMm;
constexpr float kAutoJumpHoldTimeS = params::active::control_loop::kAutoJumpHoldTimeS;
constexpr float kAutoJumpDistanceHoldTimeS = params::active::control_loop::kAutoJumpDistanceHoldTimeS;
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

constexpr float kFricSpeedStepRpm = params::active::shoot::kFricSpeedStepRpm;

}  // namespace

float NormalizeDr16Axis(const int16_t axis, const int16_t axis_max_abs) {
  return rm::modules::Clamp(static_cast<float>(axis) / static_cast<float>(axis_max_abs), -1.0f, 1.0f);
}

// 键盘输入斜坡：加速/减速使用不同步进
constexpr float kKeyboardAccelRampStep = params::active::control_loop::kKeyboardAccelRampStep;
constexpr float kKeyboardBrakeRampStep = params::active::control_loop::kKeyboardBrakeRampStep;

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

DriveInputNorm ResolveDriveInput(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote, bool dr16_parallel) {
  DriveInputNorm out{};
  static float keyboard_forward = 0.0f;
  static float keyboard_side = 0.0f;

  // DR16 优先 > VT03 键鼠 > DR16 摇杆
  if (dr16.online) {
    // DR16 键盘 WASD
    const uint16_t keys = dr16.keyboard;
    out.forward = ResolveKeyboardAxis(keys, kRcKeyW, kRcKeyS, keyboard_forward);
    out.side = ResolveKeyboardAxis(keys, kRcKeyD, kRcKeyA, keyboard_side);

    // 键盘无输入且斜坡归零后降级到摇杆
    const bool keyboard_active = (keys & (kRcKeyW | kRcKeyS | kRcKeyA | kRcKeyD)) != 0U;
    if (!keyboard_active) {
      if (std::fabs(keyboard_forward) < kKeyboardBrakeRampStep && std::fabs(keyboard_side) < kKeyboardBrakeRampStep) {
        keyboard_forward = 0.0f;
        keyboard_side = 0.0f;
        out.forward = NormalizeDr16Axis(dr16.right_y, kDr16AxisMaxAbs);
        out.side = NormalizeDr16Axis(dr16.right_x, kDr16AxisMaxAbs);
      }
    }
  } else if (tc_remote.valid) {
    // VT03 键鼠（DR16 离线时的兜底）
    const uint16_t keys = tc_remote.keyboard_value;
    out.forward = ResolveKeyboardAxis(keys, kRcKeyW, kRcKeyS, keyboard_forward);
    out.side = ResolveKeyboardAxis(keys, kRcKeyD, kRcKeyA, keyboard_side);

    // 键盘有输入时，斜坡生效；无输入时斜坡归零后才降级到摇杆
    const bool keyboard_active = (keys & (kRcKeyW | kRcKeyS | kRcKeyA | kRcKeyD)) != 0U;
    if (!keyboard_active) {
      // 等斜坡归零后再降级到摇杆，避免跳变
      if (std::fabs(keyboard_forward) < kKeyboardBrakeRampStep && std::fabs(keyboard_side) < kKeyboardBrakeRampStep) {
        keyboard_forward = 0.0f;
        keyboard_side = 0.0f;
        if (((dr16_parallel && tc_remote.keyboard_value == 0) || tc_remote.tc_from_dr16) && dr16.online) {
          out.forward = NormalizeDr16Axis(dr16.right_y, kDr16AxisMaxAbs);
          out.side = NormalizeDr16Axis(dr16.right_x, kDr16AxisMaxAbs);
        }
      }
    }
  } else {
    // 无输入，重置斜坡状态
    keyboard_forward = 0.0f;
    keyboard_side = 0.0f;
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
  // 严格优先级：DR16 > CAN 桥/裁判系统（tc_remote）
  // DR16 在线时，tc_remote 所有通道均被忽略，不混合来源。
  const bool tc_remote_active = tc_remote.valid && !tc_remote.tc_from_dr16;
  const bool has_any_input = dr16.online || tc_remote.valid;

  // ── 摩擦轮目标转速初始化（首次从参数表加载，后续由 Z/X 键调整）──
  if (tc_state.fric_speed_target_rpm == 0.0f) {
    tc_state.fric_speed_target_rpm = params::active::shoot::kFricSpeedTargetRpm;
  }

  // ── 图传键上升沿检测（图传在线时生效）──
  bool r_yaw_reset_edge = false;
  bool r_flip_180_edge = false;
  wheel_legged::StairTaskRequest stair_task_request = wheel_legged::StairTaskRequest::kNone;
  if (tc_remote_active) {
    // Ctrl 键状态（用于 Ctrl+Z/X 摩擦轮调速组合键）
    const bool ctrl_pressed = (tc_remote.keyboard_value & kRcKeyCtrl) != 0U;

    // C 键：任意状态按 C → 中腿长；已在中腿长则回低腿长（同时重置底盘正方向）
    const bool c_pressed = (tc_remote.keyboard_value & kRcKeyC) != 0U;
    if (c_pressed && tc_state.mid_leg_c_armed) {
      r_yaw_reset_edge = true;
      const bool already_mid = tc_state.mid_leg_hold;
      tc_state.mid_leg_hold = !already_mid;
      tc_state.mid_leg_f = false;
      tc_state.stair_descend_hold = false;
      tc_state.auto_small_jump_enabled = false;
      stair_task_request = wheel_legged::StairTaskRequest::kCancel;
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

    // Q: disabled -> standby -> enabled -> disabled.
    const bool q_pressed = (tc_remote.keyboard_value & kRcKeyQ) != 0U;
    if (q_pressed && tc_state.q_domain_armed) {
      tc_state.domain_state = (tc_state.domain_state + 1) % 3;
      tc_state.q_domain_armed = false;
    }
    if (!q_pressed) tc_state.q_domain_armed = true;

    // V 键：启动/取消单台阶任务；待命期间由任务协调器保持高腿长。
    const bool v_pressed = (tc_remote.keyboard_value & kRcKeyV) != 0U;
    if (v_pressed && tc_state.v_high_leg_armed) {
      r_yaw_reset_edge = true;
      tc_state.mid_leg_hold = false;
      tc_state.mid_leg_f = false;
      tc_state.auto_small_jump_enabled = false;
      stair_task_request = wheel_legged::StairTaskRequest::kArmSingle;
      tc_state.stair_descend_hold = false;
      tc_state.v_high_leg_armed = false;
    }
    if (!v_pressed) tc_state.v_high_leg_armed = true;

    // B 键：启动/取消双台阶任务；第一次成功后自动回到高腿待命。
    const bool b_pressed = (tc_remote.keyboard_value & kRcKeyB) != 0U;
    if (b_pressed && tc_state.b_high_leg_armed) {
      r_yaw_reset_edge = true;
      tc_state.mid_leg_hold = false;
      tc_state.mid_leg_f = false;
      tc_state.auto_small_jump_enabled = false;
      stair_task_request = wheel_legged::StairTaskRequest::kArmDouble;
      tc_state.stair_descend_hold = false;
      tc_state.b_high_leg_armed = false;
    }
    if (!b_pressed) tc_state.b_high_leg_armed = true;

    // F 键（上升沿）：切换 mid_leg_f 中腿长模式（慢速中腿长）
    const bool f_pressed = (tc_remote.keyboard_value & kRcKeyF) != 0U;
    if (f_pressed && tc_state.f_slow_armed) {
      tc_state.mid_leg_f = !tc_state.mid_leg_f;
      tc_state.mid_leg_hold = tc_state.mid_leg_f;
      tc_state.stair_descend_hold = false;
      tc_state.auto_small_jump_enabled = false;
      tc_state.f_slow_armed = false;
    }
    if (!f_pressed) tc_state.f_slow_armed = true;

    // R 键（上升沿）：云台转 180° + 底盘正方向切换
    const bool r_pressed = (tc_remote.keyboard_value & kRcKeyR) != 0U;
    if (r_pressed && tc_state.r_flip_armed) {
      semantic_state.rc_target.yaw_rad = rm::modules::Wrap(semantic_state.rc_target.yaw_rad + params::active::kPi,
                                                           -params::active::kPi, params::active::kPi);
      r_flip_180_edge = true;
      tc_state.r_flip_armed = false;
    }
    if (!r_pressed) tc_state.r_flip_armed = true;

    // Ctrl+Z 组合键：摩擦轮目标转速 -20 rpm（上升沿）
    const bool z_pressed = (tc_remote.keyboard_value & kRcKeyZ) != 0U;
    if (ctrl_pressed && z_pressed && tc_state.z_fric_dec_armed) {
      tc_state.fric_speed_target_rpm -= kFricSpeedStepRpm;
      tc_state.z_fric_dec_armed = false;
    }
    if (!ctrl_pressed || !z_pressed) tc_state.z_fric_dec_armed = true;

    // Ctrl+X 组合键：摩擦轮目标转速 +20 rpm（上升沿）
    const bool x_pressed = (tc_remote.keyboard_value & kRcKeyX) != 0U;
    if (ctrl_pressed && x_pressed && tc_state.x_fric_inc_armed) {
      tc_state.fric_speed_target_rpm += kFricSpeedStepRpm;
      tc_state.x_fric_inc_armed = false;
    }
    if (!ctrl_pressed || !x_pressed) tc_state.x_fric_inc_armed = true;

    // E 键：UI 刷新使能（电平有效，按住时为 true）
    tc_state.e_ui_refresh = (tc_remote.keyboard_value & kRcKeyE) != 0U;

    // Z 键长按 1s：切换倒地自启自动/手动模式
    if (z_pressed) {
      tc_state.z_hold_ms += kControlLoopDtS * 1000.0f;
      if (tc_state.z_hold_ms >= 1000.0f && tc_state.z_recovery_armed) {
        tc_state.recovery_manual_mode = !tc_state.recovery_manual_mode;
        tc_state.z_recovery_armed = false;
      }
    } else {
      tc_state.z_hold_ms = 0.0f;
      tc_state.z_recovery_armed = true;
    }

    // Z 键短按（无 Ctrl）：切换 AD 功能开关
    if (z_pressed && !ctrl_pressed && tc_state.z_ad_armed) {
      tc_state.ad_enabled = !tc_state.ad_enabled;
      tc_state.z_ad_armed = false;
    }
    if (!z_pressed) tc_state.z_ad_armed = true;
  }

  // 鼠标右键：按住时进入自瞄模式（电平有效，VT03 / DR16 均生效）
  tc_state.auto_aim_hold = tc_remote.right_button;
  f5 = tc_remote.right_button;

  input.input_valid = has_any_input;
  input.dr16 = dr16;
  input.tc_remote = tc_remote;
  const float yaw_motor_pos_rad = input.estimator_input.yaw_motor_rad;

  // ── 手动模式腿摆角速度（局部变量，每周期从键盘解析）──
  float manual_left_leg_speed = 0.0f;
  float manual_right_leg_speed = 0.0f;

  // ── 构建整车语义请求 ──
  wheel_legged::ModeRequest request{};
  request.input_valid = has_any_input;
  request.service_profile = wheel_legged::ServiceProfile::kChassisAndGimbalSafe;
  request.reset_yaw_request = r_yaw_reset_edge;
  request.flip_180_request = r_flip_180_edge;
  request.stair_task_request = stair_task_request;

  wheel_legged::CombatProfile combat_profile = wheel_legged::CombatProfile::kNormal;
  wheel_legged::LegProfile leg_request = wheel_legged::LegProfile::kLow;

  if (dr16.online) {
    // ═══ DR16 优先 ═══

    // 左拨杆 kDown + 右拨杆的组合用于云台辨识/验证
    if (dr16.switch_l == rm::device::DR16::SwitchPosition::kDown) {
      if (dr16.switch_r == rm::device::DR16::SwitchPosition::kUp) {
        request.domain_request = wheel_legged::DomainRequest::kService;
        request.gimbal_test_profile = wheel_legged::GimbalTestProfile::kIdent;
        leg_request = wheel_legged::LegProfile::kLow;
      } else if (dr16.switch_r == rm::device::DR16::SwitchPosition::kMid) {
        request.domain_request = wheel_legged::DomainRequest::kService;
        request.gimbal_test_profile = wheel_legged::GimbalTestProfile::kFfVerify;
        leg_request = wheel_legged::LegProfile::kLow;
      } else {
        request.domain_request = wheel_legged::DomainRequest::kDisabled;
        leg_request = wheel_legged::LegProfile::kLow;
      }
    } else {
      request.domain_request = ResolveDomainRequest(dr16.switch_l);
    }

    // 腿长 + 战斗子模式：右拨杆
    if (request.domain_request == wheel_legged::DomainRequest::kCombat) {
      switch (dr16.switch_r) {
        case rm::device::DR16::SwitchPosition::kDown:
          combat_profile = wheel_legged::CombatProfile::kNormal;
          leg_request = wheel_legged::LegProfile::kLow;
          break;
        case rm::device::DR16::SwitchPosition::kMid:
          combat_profile = wheel_legged::CombatProfile::kAutoAimAmmo;
          leg_request = wheel_legged::LegProfile::kLow;
          break;
        case rm::device::DR16::SwitchPosition::kUp:
          combat_profile = wheel_legged::CombatProfile::kAutoAimFuSmall;
          leg_request = wheel_legged::LegProfile::kLow;
          break;
        default:
          break;
      }
    } else if (request.gimbal_test_profile == wheel_legged::GimbalTestProfile::kNormal) {
      leg_request = ResolveLegProfile(dr16.switch_r);
    }

    // 鼠标右键按住时覆盖为自瞄模式（Ctrl+F/G 切换子模式）
    if (tc_state.auto_aim_hold && request.domain_request == wheel_legged::DomainRequest::kCombat) {
      switch (tc_state.aim_mode) {
        case TcSemanticState::AimMode::kFuSmall:
          combat_profile = wheel_legged::CombatProfile::kAutoAimFuSmall;
          break;
        case TcSemanticState::AimMode::kFuBig:
          combat_profile = wheel_legged::CombatProfile::kAutoAimFuBig;
          break;
        case TcSemanticState::AimMode::kAmmo:
        default:
          combat_profile = wheel_legged::CombatProfile::kAutoAimAmmo;
          break;
      }
    }

    // 小陀螺正/反转：辨识/验证模式下不响应拨轮；战斗模式下禁用反转
    if (request.gimbal_test_profile == wheel_legged::GimbalTestProfile::kNormal) {
      if (dr16.dial >= kWheelSpinThreshold) {
        request.spin_hold = true;
      } else if (dr16.dial <= -kWheelActionThreshold &&
                 request.domain_request != wheel_legged::DomainRequest::kCombat) {
        request.spin_hold = true;
        request.spin_dir = -1.0f;
      }
    }

  } else if (tc_remote_active) {
    // ═══ 图传链路兜底（CAN 桥 VT03 > 裁判系统 0x304）═══

    // 战斗子模式：鼠标右键按住进入自瞄（Ctrl+F/G 切换子模式）
    if (tc_state.auto_aim_hold) {
      switch (tc_state.aim_mode) {
        case TcSemanticState::AimMode::kFuSmall:
          combat_profile = wheel_legged::CombatProfile::kAutoAimFuSmall;
          break;
        case TcSemanticState::AimMode::kFuBig:
          combat_profile = wheel_legged::CombatProfile::kAutoAimFuBig;
          break;
        case TcSemanticState::AimMode::kAmmo:
        default:
          combat_profile = wheel_legged::CombatProfile::kAutoAimAmmo;
          break;
      }
    }

    // 工作域：Q 键循环
    switch (tc_state.domain_state) {
      case 0:
        request.domain_request = wheel_legged::DomainRequest::kDisabled;
        break;
      case 1:
        request.domain_request = wheel_legged::DomainRequest::kCombat;
        request.standby = true;
        break;
      case 2:
      default:
        request.domain_request = wheel_legged::DomainRequest::kCombat;
        break;
    }

    // 台阶任务的高腿待命由协调器输出覆盖；输入层仅保留普通中/低腿请求。
    if (tc_state.mid_leg_hold) {
      leg_request = wheel_legged::LegProfile::kMid;
    } else {
      leg_request = wheel_legged::LegProfile::kLow;
    }

    // 战斗子模式：图传无拨杆，默认 Normal

    // 小陀螺：Shift 键
    request.spin_hold = (tc_remote.keyboard_value & kRcKeyShift) != 0U;

    // Mouse wheel up toggles stair-descend mode. Entry uses the existing forward-reset path.
    {
      const bool mouse_z_trigger = tc_remote.mouse_z > 70;
      const bool mouse_z_edge = mouse_z_trigger && tc_state.stair_descend_armed;
      if (mouse_z_edge) {
        const bool entering = !tc_state.stair_descend_hold;
        tc_state.stair_descend_hold = entering;
        if (entering) {
          tc_state.mid_leg_hold = false;
          tc_state.mid_leg_f = false;
          tc_state.auto_small_jump_enabled = false;
          request.stair_task_request = wheel_legged::StairTaskRequest::kCancel;
          request.reset_yaw_request = true;
        }
      }
      if (mouse_z_trigger) tc_state.stair_descend_armed = false;
      if (tc_remote.mouse_z <= 0) tc_state.stair_descend_armed = true;
    }

    // 跳跃：F 键（中腿）或鼠标滚轮下滚 < -70（低腿）
    // 鼠标滚轮边沿检测：下滚<-70触发后，须回到>=0才重新就绪，防止同一轮滚动重复触发
    {
      static bool s_mouse_z_armed = true;
      const bool mouse_z_trigger = (leg_request == wheel_legged::LegProfile::kLow && tc_remote.mouse_z < -70);
      const bool mouse_z_edge = mouse_z_trigger && s_mouse_z_armed;
      if (mouse_z_trigger) s_mouse_z_armed = false;
      if (tc_remote.mouse_z >= 0) s_mouse_z_armed = true;
      request.jump_trigger = mouse_z_edge && !tc_state.stair_descend_hold;
    }

    // 手动倒地自启模式：A/D 控制左右腿正转，Ctrl+A/D 控制反转
    {
      const uint16_t keys = tc_remote.keyboard_value;
      const bool ctrl_held = (keys & kRcKeyCtrl) != 0U;
      const bool a_held = (keys & kRcKeyA) != 0U;
      const bool d_held = (keys & kRcKeyD) != 0U;
      constexpr float kSpeed = params::active::chassis::kManualRecoveryLegSpeedRadS;

      if (tc_state.recovery_manual_mode) {
        if (a_held && ctrl_held) {
          manual_left_leg_speed = -kSpeed;
        } else if (a_held) {
          manual_left_leg_speed = kSpeed;
        }
        if (d_held && ctrl_held) {
          manual_right_leg_speed = -kSpeed;
        } else if (d_held) {
          manual_right_leg_speed = kSpeed;
        }
      }
    }

  } else {
    // ═══ 无输入 ═══
    request.domain_request = wheel_legged::DomainRequest::kDisabled;
    request.jump_trigger = false;
  }

  request.stair_descend_active = tc_state.stair_descend_hold;
  request.leg_request = leg_request;
  if (leg_request == wheel_legged::LegProfile::kHigh &&
      request.stair_task_request == wheel_legged::StairTaskRequest::kNone) {
    // DR16 高腿档保持自动台阶待命，松开档位后协调器会自动取消。
    request.stair_task_request = wheel_legged::StairTaskRequest::kArmContinuous;
  }
  request.mid_leg_f = tc_state.mid_leg_f;
  if (combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
      combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig) {
    request.standby = true;
  }
  if (request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    request.standby = false;
  }

  // ── 目标来源：自瞄模式优先上位机 ──
  if (combat_profile == wheel_legged::CombatProfile::kAutoAimAmmo ||
      combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
      combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig) {
    request.target_source =
        request.host_target_valid ? wheel_legged::TargetSource::kHost : wheel_legged::TargetSource::kRc;
  } else {
    request.target_source = wheel_legged::TargetSource::kRc;
  }
  request.combat_profile = combat_profile;

  // ── 自瞄 → 正常模式切出时，重置积分初始标记，避免云台甩到切入自瞄时的角度 ──
  {
    const bool is_auto_aim = (combat_profile == wheel_legged::CombatProfile::kAutoAimAmmo ||
                              combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
                              combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig);
    if (!is_auto_aim && semantic_state.last_auto_aim) {
      semantic_state.gimbal_target_initialized = false;
    }
    semantic_state.last_auto_aim = is_auto_aim;
  }

  // ── 云台目标积分（优先级同上：DR16 > tc_remote）──
  const bool is_auto_aim = (combat_profile == wheel_legged::CombatProfile::kAutoAimAmmo ||
                            combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
                            combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig);
  const bool host_controls_gimbal = is_auto_aim && request.target_source == wheel_legged::TargetSource::kHost;
  if (!has_any_input || request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    semantic_state.rc_target.yaw_rad = yaw_motor_pos_rad;
    semantic_state.rc_target.pitch_rad = 0.0f;
    semantic_state.gimbal_target_initialized = false;
  } else if (host_controls_gimbal) {
    // 自瞄 + NUC 有目标：仅在切入时锁定当前云台 IMU 位置；后续不更新，由 PID 保持
    if (!semantic_state.gimbal_target_initialized) {
      semantic_state.rc_target.yaw_rad = input.gimbal_imu_yaw_rad;
      semantic_state.rc_target.pitch_rad = -input.gimbal_imu_pitch_rad;
      semantic_state.gimbal_target_initialized = true;
    }
  } else {
    if (!semantic_state.gimbal_target_initialized) {
      semantic_state.rc_target.yaw_rad = input.gimbal_imu_yaw_rad;
      semantic_state.rc_target.pitch_rad = -input.gimbal_imu_pitch_rad;
      semantic_state.gimbal_target_initialized = true;
    }
    float yaw_delta = 0.0f;
    float pitch_delta = 0.0f;
    if (dr16.online) {
      // DR16 鼠标（优先）
      const bool dr16_mouse_active = (dr16.mouse_x != 0 || dr16.mouse_y != 0);
      if (dr16_mouse_active) {
        yaw_delta += static_cast<float>(dr16.mouse_x) / kDr16MouseMax * kDr16MouseYawRateMaxRadS * kControlLoopDtS;
        pitch_delta += static_cast<float>(dr16.mouse_y) / kDr16MouseMax * kDr16MousePitchRateMaxRadS * kControlLoopDtS;
      } else {
        // DR16 左摇杆
        yaw_delta += static_cast<float>(dr16.left_x) / kRcStickMax * kRcYawRateMaxRadS * kControlLoopDtS;
        pitch_delta += static_cast<float>(dr16.left_y) / kRcStickMax * kRcPitchRateMaxRadS * kControlLoopDtS;
      }
    } else if (tc_remote_active) {
      // 图传鼠标（DR16 离线时的兜底）
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
  request.rc_target = semantic_state.rc_target;

  request.fall_detected = false;
  request.fall_detected_hold_ms = 0U;
  request.upright_stable = true;
  request.recovery_manual_mode = tc_state.recovery_manual_mode;
  request.manual_left_leg_speed = manual_left_leg_speed;
  request.manual_right_leg_speed = manual_right_leg_speed;
  request.tick_ms = 0U;

  input.mode_request = request;
}

void UpdateRawFeedbackAndInputSnapshot(SharedResources &g, chassis_runtime::Actuators &actuators, InputSnapshot &input,
                                       Dr16SemanticState &semantic_state, TcSemanticState &tc_state) {
  const bool previous_host_target_active =
      input.mode_request.target_source == wheel_legged::TargetSource::kHost && input.mode_request.host_target_valid;

  // 1. 从执行器采集关节/轮毂/IMU 反馈
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

  // DR16 键盘位掩码
  {
    uint16_t keys = 0;
    if (g.dr16.key(rm::device::DR16::Key::kW)) keys |= 0x0001;
    if (g.dr16.key(rm::device::DR16::Key::kS)) keys |= 0x0002;
    if (g.dr16.key(rm::device::DR16::Key::kA)) keys |= 0x0004;
    if (g.dr16.key(rm::device::DR16::Key::kD)) keys |= 0x0008;
    if (g.dr16.key(rm::device::DR16::Key::kShift)) keys |= 0x0010;
    if (g.dr16.key(rm::device::DR16::Key::kCtrl)) keys |= 0x0020;
    if (g.dr16.key(rm::device::DR16::Key::kQ)) keys |= 0x0040;
    if (g.dr16.key(rm::device::DR16::Key::kE)) keys |= 0x0080;
    if (g.dr16.key(rm::device::DR16::Key::kR)) keys |= 0x0100;
    if (g.dr16.key(rm::device::DR16::Key::kF)) keys |= 0x0200;
    if (g.dr16.key(rm::device::DR16::Key::kG)) keys |= 0x0400;
    if (g.dr16.key(rm::device::DR16::Key::kZ)) keys |= 0x0800;
    if (g.dr16.key(rm::device::DR16::Key::kX)) keys |= 0x1000;
    if (g.dr16.key(rm::device::DR16::Key::kC)) keys |= 0x2000;
    if (g.dr16.key(rm::device::DR16::Key::kV)) keys |= 0x4000;
    if (g.dr16.key(rm::device::DR16::Key::kB)) keys |= 0x8000;
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

  // 3a. 使能边沿检测：从 kDisabled 进入使能域时，腿长状态全部归零（低腿长起立）
  {
    const bool tc_active = tc_remote.valid && !tc_remote.tc_from_dr16;
    wheel_legged::DomainRequest predicted_domain = wheel_legged::DomainRequest::kDisabled;
    if (dr16.online) {
      predicted_domain = ResolveDomainRequest(dr16.switch_l);
    } else if (tc_active) {
      predicted_domain =
          (tc_state.domain_state != 0) ? wheel_legged::DomainRequest::kCombat : wheel_legged::DomainRequest::kDisabled;
    }
    static wheel_legged::DomainRequest prev_domain = wheel_legged::DomainRequest::kDisabled;
    if (prev_domain == wheel_legged::DomainRequest::kDisabled &&
        predicted_domain != wheel_legged::DomainRequest::kDisabled) {
      tc_state.mid_leg_hold = false;
      tc_state.mid_leg_f = false;
      tc_state.auto_small_jump_enabled = false;
      tc_state.aim_mode = TcSemanticState::AimMode::kAmmo;
    }
    prev_domain = predicted_domain;
  }

  // 3b. 语义折叠
  ResolveInputSemantics(dr16, tc_remote, semantic_state, tc_state, input);

  // 3c. 自动小跳（G 键模式 + 超声波测距触发）
  {
    // 模式重新开启时清除触发锁存
    if (tc_state.auto_small_jump_enabled && !input.auto_jump_enabled) {
      input.auto_small_jump_triggered = false;
    }
    const bool in_low_leg = (input.mode_request.leg_request == wheel_legged::LegProfile::kLow);
    static float s_distance_below_timer = 0.0f;
    const auto dyp_left = g.dyp_left.has_value() ? g.dyp_left->distance_raw() : 0U;
    const auto dyp_right = g.dyp_right.has_value() ? g.dyp_right->distance_raw() : 0U;
    const auto dyp_avg = static_cast<uint16_t>((static_cast<uint32_t>(dyp_left) + dyp_right) / 2U);
    const bool distance_below_threshold = (dyp_avg <= kAutoJumpDistanceThresholdMm && dyp_avg > 0U);
    if (tc_state.auto_small_jump_enabled && in_low_leg && distance_below_threshold) {
      s_distance_below_timer += kControlLoopDtS;
    } else {
      s_distance_below_timer = 0.0f;
    }
    if (s_distance_below_timer >= kAutoJumpDistanceHoldTimeS) {
      // input.mode_request.jump_trigger = true;
      input.mode_request.auto_jump_triggered = true;
      input.auto_small_jump_triggered = true;  // 锁存，供调试观察
      tc_state.auto_small_jump_enabled = false;
      s_distance_below_timer = 0.0f;
    }
    input.auto_jump_enabled = tc_state.auto_small_jump_enabled;
  }

  // 3d. 自瞄上位机目标（NUC 反馈 → host_target，仅在自瞄模式下生效）
  const bool auto_aim_active = input.mode_request.combat_profile == wheel_legged::CombatProfile::kAutoAimAmmo ||
                               input.mode_request.combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
                               input.mode_request.combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig;
  const bool host_target_available = g.aimbot.has_value() && g.aimbot->online_status() == rm::device::Device::kOk &&
                                     g.aimbot->nuc_start_flag() != 0 && auto_aim_active &&
                                     g.aimbot->aimbot_state() != 0;
  if (previous_host_target_active && auto_aim_active && !host_target_available && gimbal_rx_valid) {
    // Host -> RC handover: take over from the current pose instead of returning to a stale manual target.
    semantic_state.rc_target.yaw_rad = g.gimbal_rx->yaw_rad();
    semantic_state.rc_target.pitch_rad = -g.gimbal_rx->pitch_rad();
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
                                         const chassis::Chassis::UpdateOutput &chassis_output) {
  // 倒地检测：基于上周期底盘姿态（posture_valid 在 Chassis::Update 中计算，FSM 之前运行，因此用上一周期值）
  static uint32_t fall_start_ms = 0;
  static bool was_posture_invalid = false;

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
      .stair_descend_active = m.stair_descend_active,
      .theta_b_rad = chassis_output.current_state.theta_b,
      .input_valid = m.input_valid,
      .domain_request = m.domain_request,
      .leg_request = m.leg_request,
      .combat_profile = m.combat_profile,
      .standby = m.standby,
      .spin_hold = m.spin_hold,
      .spin_dir = m.spin_dir,
      .jump_trigger = m.jump_trigger,
      .auto_jump_triggered = m.auto_jump_triggered,
      .current_leg_length_m = chassis_output.mean_leg_length_m,
      .theta_ll_rad = chassis_output.current_state.theta_ll,
      .theta_lr_rad = chassis_output.current_state.theta_lr,
      .fall_detected = fall_detected,
      .fall_detected_hold_ms = fall_detected_hold_ms,
      .upright_stable = upright_stable,
      .recovery_manual_mode = m.recovery_manual_mode,
      .manual_left_leg_speed = m.manual_left_leg_speed,
      .manual_right_leg_speed = m.manual_right_leg_speed,
      .tick_ms = tick_ms,
  };
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
