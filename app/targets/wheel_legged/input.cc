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
constexpr float kPitchTargetMinRad = params::active::control_loop::kPitchTargetMinRad;
constexpr float kPitchTargetMaxRad = params::active::control_loop::kPitchTargetMaxRad;

constexpr uint16_t kRcKeyW = 0x0001;
constexpr uint16_t kRcKeyS = 0x0002;
constexpr uint16_t kRcKeyA = 0x0004;
constexpr uint16_t kRcKeyD = 0x0008;
constexpr uint16_t kRcKeyShift = 0x0010;
constexpr uint16_t kRcKeyQ = 0x0040;
constexpr uint16_t kRcKeyR = 0x0100;
constexpr uint16_t kRcKeyF = 0x0200;
constexpr uint16_t kRcKeyV = 0x4000;
constexpr uint16_t kRcKeyC = 0x2000;
constexpr uint16_t kRcKeyB = 0x8000;
constexpr uint16_t kRcKeyG = 0x0400;

}  // namespace

float NormalizeDr16Axis(const int16_t axis, const int16_t axis_max_abs) {
  return rm::modules::Clamp(static_cast<float>(axis) / static_cast<float>(axis_max_abs), -1.0f, 1.0f);
}

DriveInputNorm ResolveDriveInput(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote, bool dr16_parallel) {
  DriveInputNorm out{};

  // 图传链路在线时优先使用 WASD；图传离线时降级为 DR16 摇杆
  if (tc_remote.valid) {
    const uint16_t keys = tc_remote.keyboard_value;
    if ((keys & kRcKeyW) != 0U) {
      out.forward = 1.0f;
    } else if ((keys & kRcKeyS) != 0U) {
      out.forward = -1.0f;
    }
    if ((keys & kRcKeyD) != 0U) {
      out.side = 1.0f;
    } else if ((keys & kRcKeyA) != 0U) {
      out.side = -1.0f;
    }
    // 并行模式：键鼠无输入时降级到 DR16 右摇杆
    if (dr16_parallel && dr16.online && out.forward == 0.0f && out.side == 0.0f) {
      out.forward = NormalizeDr16Axis(dr16.right_y, kDr16AxisMaxAbs);
      out.side = NormalizeDr16Axis(dr16.right_x, kDr16AxisMaxAbs);
    }
  } else if (dr16.online) {
    out.forward = NormalizeDr16Axis(dr16.right_y, kDr16AxisMaxAbs);
    out.side = NormalizeDr16Axis(dr16.right_x, kDr16AxisMaxAbs);
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
  // 严格优先级：CAN 桥/裁判系统（tc_remote）> DR16
  // tc_remote 在线时，DR16 所有通道均被忽略，不混合来源。
  const bool tc_remote_active = tc_remote.valid;
  const bool has_any_input = dr16.online || tc_remote_active;

  // ── 图传键上升沿检测（图传在线时生效）──
  bool r_yaw_reset_edge = false;
  bool f_jump_edge = false;
  if (tc_remote_active) {
    // C 键：任意状态按 C → 中腿长；已在中腿长则回低腿长
    const bool c_pressed = (tc_remote.keyboard_value & kRcKeyC) != 0U;
    if (c_pressed && tc_state.mid_leg_c_armed) {
      const bool already_mid = tc_state.mid_leg_hold && !tc_state.high_leg_hold && !tc_state.stair_climb_done;
      tc_state.mid_leg_hold = !already_mid;
      tc_state.high_leg_hold = false;
      tc_state.stair_climb_done = false;
      tc_state.b_double_mode = false;
      tc_state.b_attempt = 0;
      tc_state.mid_leg_c_armed = false;
    }
    if (!c_pressed) tc_state.mid_leg_c_armed = true;

    // Q 键：工作域循环（kDisabled → kService → kDisabled …）
    const bool q_pressed = (tc_remote.keyboard_value & kRcKeyQ) != 0U;
    if (q_pressed && tc_state.q_domain_armed) {
      tc_state.domain_state = (tc_state.domain_state + 1) % 2;
      tc_state.q_domain_armed = false;
    }
    if (!q_pressed) tc_state.q_domain_armed = true;

    // V 键：任意状态按 V → 高腿长（1 次上台阶）；已在高腿长则回低腿长
    const bool v_pressed = (tc_remote.keyboard_value & kRcKeyV) != 0U;
    if (v_pressed && tc_state.v_high_leg_armed) {
      const bool already_high = tc_state.high_leg_hold && !tc_state.stair_climb_done;
      tc_state.high_leg_hold = !already_high;
      tc_state.mid_leg_hold = false;
      tc_state.stair_climb_done = false;
      tc_state.b_double_mode = false;
      tc_state.b_attempt = 0;
      tc_state.v_high_leg_armed = false;
    }
    if (!v_pressed) tc_state.v_high_leg_armed = true;

    // B 键：任意状态按 B → 高腿长（2 次上台阶）；已在高腿长则回低腿长
    const bool b_pressed = (tc_remote.keyboard_value & kRcKeyB) != 0U;
    if (b_pressed && tc_state.b_high_leg_armed) {
      const bool already_high = tc_state.high_leg_hold && !tc_state.stair_climb_done;
      tc_state.high_leg_hold = !already_high;
      tc_state.mid_leg_hold = false;
      tc_state.stair_climb_done = false;
      tc_state.b_double_mode = !already_high;
      tc_state.b_attempt = 0;
      tc_state.b_high_leg_armed = false;
    }
    if (!b_pressed) tc_state.b_high_leg_armed = true;

    // R 键：重置底盘正方向（上升沿，一次性请求）
    const bool r_pressed = (tc_remote.keyboard_value & kRcKeyR) != 0U;
    if (r_pressed && tc_state.r_yaw_reset_armed) {
      r_yaw_reset_edge = true;
      tc_state.r_yaw_reset_armed = false;
    }
    if (!r_pressed) tc_state.r_yaw_reset_armed = true;

    // F 键：跳跃（上升沿，一次性请求）
    const bool f_pressed = (tc_remote.keyboard_value & kRcKeyF) != 0U;
    if (f_pressed && tc_state.f_jump_armed) {
      f_jump_edge = true;
      tc_state.f_jump_armed = false;
    }
    if (!f_pressed) tc_state.f_jump_armed = true;

    // G 键长按 1s：切换 DR16 并行模式
    const bool g_pressed = (tc_remote.keyboard_value & kRcKeyG) != 0U;
    if (g_pressed) {
      tc_state.g_hold_ms += kControlLoopDtS * 1000.0f;
      if (tc_state.g_hold_ms >= 1000.0f && tc_state.dr16_parallel_armed) {
        tc_state.dr16_parallel = !tc_state.dr16_parallel;
        tc_state.dr16_parallel_armed = false;
      }
    } else {
      tc_state.g_hold_ms = 0.0f;
      tc_state.dr16_parallel_armed = true;
    }
  }

  input.input_valid = has_any_input;
  input.dr16 = dr16;
  input.tc_remote = tc_remote;
  const float yaw_motor_pos_rad = input.estimator_input.yaw_motor_rad;

  // ── 构建整车语义请求 ──
  wheel_legged::ModeRequest request{};
  request.input_valid = has_any_input;
  request.service_profile = wheel_legged::ServiceProfile::kChassisAndGimbalSafe;
  request.reset_yaw_request = r_yaw_reset_edge;

  wheel_legged::CombatProfile combat_profile = wheel_legged::CombatProfile::kNormal;
  wheel_legged::LegProfile leg_request = wheel_legged::LegProfile::kLow;

  if (tc_remote_active) {
    // ═══ 图传链路优先（CAN 桥 VT03 > 裁判系统 0x304）═══

    // 工作域：Q 键循环
    switch (tc_state.domain_state) {
      case 0:
        request.domain_request = wheel_legged::DomainRequest::kDisabled;
        break;
      case 1:
      default:
        request.domain_request = wheel_legged::DomainRequest::kService;
        break;
    }

    // 腿长：上台阶完成锁定低腿长 > V 键高腿长 > C 键中腿长 > 低腿长
    if (tc_state.stair_climb_done) {
      leg_request = wheel_legged::LegProfile::kLow;
    } else if (tc_state.high_leg_hold) {
      leg_request = wheel_legged::LegProfile::kHigh;
    } else if (tc_state.mid_leg_hold) {
      leg_request = wheel_legged::LegProfile::kMid;
    } else {
      leg_request = wheel_legged::LegProfile::kLow;
    }

    // 战斗子模式：图传无拨杆，默认 Normal

    // 小陀螺：Shift 键
    request.spin_hold = (tc_remote.keyboard_value & kRcKeyShift) != 0U;

    // 跳跃：F 键
    request.jump_trigger = f_jump_edge;

    // 并行模式：DR16 拨杆/拨轮也响应（键鼠优先，DR16 补充）
    if (tc_state.dr16_parallel && dr16.online) {
      if (request.domain_request == wheel_legged::DomainRequest::kDisabled) {
        request.domain_request = ResolveDomainRequest(dr16.switch_l);
      }
      if (leg_request == wheel_legged::LegProfile::kLow) {
        leg_request = ResolveLegProfile(dr16.switch_r);
      }
      if (!request.spin_hold) {
        request.spin_hold = dr16.dial >= kWheelSpinThreshold;
      }
      if (!request.jump_trigger) {
        if (std::abs(dr16.dial) <= kWheelCenterThreshold) {
          semantic_state.wheel_action_armed = true;
        }
        const bool wheel_action_trigger =
            semantic_state.wheel_action_armed && (dr16.dial <= -kWheelActionThreshold);
        if (wheel_action_trigger) {
          semantic_state.wheel_action_armed = false;
        }
        request.jump_trigger = wheel_action_trigger &&
                               request.domain_request != wheel_legged::DomainRequest::kCombat;
      }
    }

  } else if (dr16.online) {
    // ═══ DR16 兜底 ═══

    // 工作域：左拨杆
    request.domain_request = ResolveDomainRequest(dr16.switch_l);

    // 腿长 + 战斗子模式：右拨杆
    if (request.domain_request == wheel_legged::DomainRequest::kCombat) {
      switch (dr16.switch_r) {
        case rm::device::DR16::SwitchPosition::kDown:
          combat_profile = wheel_legged::CombatProfile::kNormal;
          leg_request = wheel_legged::LegProfile::kLow;
          break;
        case rm::device::DR16::SwitchPosition::kMid:
          combat_profile = wheel_legged::CombatProfile::kAutoAimNoMove;
          leg_request = wheel_legged::LegProfile::kLow;
          break;
        case rm::device::DR16::SwitchPosition::kUp:
          combat_profile = wheel_legged::CombatProfile::kAutoAimWithMove;
          leg_request = wheel_legged::LegProfile::kLow;
          break;
        default:
          break;
      }
    } else {
      leg_request = ResolveLegProfile(dr16.switch_r);
    }

    // 小陀螺：拨轮
    request.spin_hold = dr16.dial >= kWheelSpinThreshold;

    // 跳跃：拨轮回中后快速负推（非战斗域）
    if (std::abs(dr16.dial) <= kWheelCenterThreshold) {
      semantic_state.wheel_action_armed = true;
    }
    const bool wheel_action_trigger = semantic_state.wheel_action_armed && (dr16.dial <= -kWheelActionThreshold);
    if (wheel_action_trigger) {
      semantic_state.wheel_action_armed = false;
    }
    request.jump_trigger = wheel_action_trigger && request.domain_request != wheel_legged::DomainRequest::kCombat;

  } else {
    // ═══ 无输入 ═══
    request.domain_request = wheel_legged::DomainRequest::kDisabled;
    request.jump_trigger = false;
  }

  request.leg_request = leg_request;

  // ── 目标来源：自瞄模式优先上位机 ──
  if (combat_profile == wheel_legged::CombatProfile::kAutoAimNoMove ||
      combat_profile == wheel_legged::CombatProfile::kAutoAimWithMove) {
    request.target_source =
        request.host_target_valid ? wheel_legged::TargetSource::kHost : wheel_legged::TargetSource::kRc;
  } else {
    request.target_source = wheel_legged::TargetSource::kRc;
  }
  request.combat_profile = combat_profile;

  // ── 云台目标积分（优先级同上：tc_remote > DR16）──
  if (!has_any_input || request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    semantic_state.rc_target.yaw_rad = yaw_motor_pos_rad;
    semantic_state.rc_target.pitch_rad = 0.0f;
    semantic_state.gimbal_target_initialized = false;
  } else {
    if (!semantic_state.gimbal_target_initialized) {
      semantic_state.rc_target.yaw_rad = yaw_motor_pos_rad;
      semantic_state.rc_target.pitch_rad = 0.0f;
      semantic_state.gimbal_target_initialized = true;
    }
    float yaw_delta = 0.0f;
    float pitch_delta = 0.0f;
    if (tc_remote_active) {
      // 图传鼠标（优先）
      const float mouse_yaw = static_cast<float>(tc_remote.mouse_x) / kTcMouseMax * kTcMouseYawRateMaxRadS * kControlLoopDtS;
      const float mouse_pitch = static_cast<float>(tc_remote.mouse_y) / kTcMouseMax * kTcMousePitchRateMaxRadS * kControlLoopDtS;
      const bool mouse_active = (tc_remote.mouse_x != 0 || tc_remote.mouse_y != 0);
      if (mouse_active) {
        yaw_delta += mouse_yaw;
        pitch_delta += mouse_pitch;
      } else if (tc_state.dr16_parallel && dr16.online) {
        // 鼠标静止 + 并行模式 → 降级到 DR16 左摇杆
        yaw_delta += static_cast<float>(dr16.left_x) / kRcStickMax * kRcYawRateMaxRadS * kControlLoopDtS;
        pitch_delta += static_cast<float>(dr16.left_y) / kRcStickMax * kRcPitchRateMaxRadS * kControlLoopDtS;
      }
    } else {
      // DR16 左摇杆
      yaw_delta += static_cast<float>(dr16.left_x) / kRcStickMax * kRcYawRateMaxRadS * kControlLoopDtS;
      pitch_delta += static_cast<float>(dr16.left_y) / kRcStickMax * kRcPitchRateMaxRadS * kControlLoopDtS;
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
  request.tick_ms = 0U;

  input.mode_request = request;
}

void UpdateRawFeedbackAndInputSnapshot(SharedResources &g, chassis_runtime::Actuators &actuators, InputSnapshot &input,
                                       Dr16SemanticState &semantic_state, TcSemanticState &tc_state) {
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
    tc_remote.left_button = g.gimbal_rx->left_button();
    tc_remote.right_button = g.gimbal_rx->right_button();
    tc_remote.keyboard_value = g.gimbal_rx->keyboard_value();
  } else if (dr16.online) {
    tc_remote.valid = true;
    tc_remote.mouse_x = dr16.mouse_x;
    tc_remote.mouse_y = dr16.mouse_y;
    tc_remote.left_button = dr16.mouse_left;
    tc_remote.right_button = dr16.mouse_right;
    tc_remote.keyboard_value = dr16.keyboard;
  }

  // 3a. 使能边沿检测：从 kDisabled 进入使能域时，腿长状态全部归零（低腿长起立）
  {
    const bool tc_active = tc_remote.valid;
    wheel_legged::DomainRequest predicted_domain = wheel_legged::DomainRequest::kDisabled;
    if (tc_active) {
      predicted_domain =
          (tc_state.domain_state == 1) ? wheel_legged::DomainRequest::kService : wheel_legged::DomainRequest::kDisabled;
    } else if (dr16.online) {
      predicted_domain = ResolveDomainRequest(dr16.switch_l);
    }
    static wheel_legged::DomainRequest prev_domain = wheel_legged::DomainRequest::kDisabled;
    if (prev_domain == wheel_legged::DomainRequest::kDisabled &&
        predicted_domain != wheel_legged::DomainRequest::kDisabled) {
      tc_state.mid_leg_hold = false;
      tc_state.high_leg_hold = false;
      tc_state.stair_climb_done = false;
      tc_state.b_double_mode = false;
      tc_state.b_attempt = 0;
    }
    prev_domain = predicted_domain;
  }

  // 3b. 语义折叠
  ResolveInputSemantics(dr16, tc_remote, semantic_state, tc_state, input);

  // 3c. 自瞄上位机目标（NUC 反馈 → host_target，覆盖语义折叠中的 rc_target）
  // 3a2. 自动跳跃（暂时注释：先调原地跳跃，调完再启用）
  // static bool s_auto_jump_enabled = false;
  // static float s_dial_hold_timer = 0.0f;
  // static bool s_dial_hold_fired = false;
  //
  // const bool in_low_leg = (input.mode_request.leg_request == wheel_legged::LegProfile::kLow);
  // const bool dial_at_threshold = (dr16.dial <= -kWheelActionThreshold);
  //
  // if (dial_at_threshold && in_low_leg) {
  //   if (!s_dial_hold_fired) {
  //     s_dial_hold_timer += kControlLoopDtS;
  //     if (s_dial_hold_timer >= kAutoJumpHoldTimeS) {
  //       s_auto_jump_enabled = !s_auto_jump_enabled;
  //       s_dial_hold_fired = true;
  //     }
  //   }
  // } else {
  //   s_dial_hold_timer = 0.0f;
  //   s_dial_hold_fired = false;
  // }
  //
  // static float s_distance_below_timer = 0.0f;
  // const bool distance_below_threshold =
  //     g.dyp_rx.has_value() && g.dyp_rx->distance_filtered_avg() <= kAutoJumpDistanceThresholdMm &&
  //     g.dyp_rx->distance_filtered_avg() > 0U;
  // if (s_auto_jump_enabled && in_low_leg && distance_below_threshold) {
  //   s_distance_below_timer += kControlLoopDtS;
  // } else {
  //   s_distance_below_timer = 0.0f;
  // }
  // if (s_distance_below_timer >= kAutoJumpDistanceHoldTimeS) {
  //   input.mode_request.jump_trigger = true;
  //   input.mode_request.auto_jump_triggered = true;
  //   s_auto_jump_enabled = false;
  //   s_distance_below_timer = 0.0f;
  // }
  // input.auto_jump_enabled = s_auto_jump_enabled;

  // 3b. 自瞄上位机目标（NUC 反馈 → host_target，覆盖语义折叠中的 rc_target）
  if (g.aimbot.has_value() && g.aimbot->nuc_start_flag() != 0) {
    constexpr float kDegToRad = params::active::kPi / 180.0f;
    input.mode_request.host_target.yaw_rad = -g.aimbot->yaw() * kDegToRad;
    input.mode_request.host_target.pitch_rad = g.aimbot->pitch() * kDegToRad;
    input.mode_request.host_target_valid = true;
    // 修正 target_source：ResolveInputSemantics 计算时 host_target_valid 尚为 false，需重设
    if (input.mode_request.combat_profile == wheel_legged::CombatProfile::kAutoAimNoMove ||
        input.mode_request.combat_profile == wheel_legged::CombatProfile::kAutoAimWithMove) {
      input.mode_request.target_source = wheel_legged::TargetSource::kHost;
    }
  }

  // 4. 云台惯导（CAN 桥，独立于底盘 IMU）
  input.gimbal_imu_yaw_rad = gimbal_rx_valid ? g.gimbal_rx->yaw_rad() : 0.0f;
  input.gimbal_imu_pitch_rad = gimbal_rx_valid ? g.gimbal_rx->pitch_rad() : 0.0f;
  input.gimbal_imu_gyro_z_rad_s = gimbal_rx_valid ? g.gimbal_rx->gyro_z_rad_s() : 0.0f;
  input.gimbal_imu_gyro_x_rad_s = gimbal_rx_valid ? g.gimbal_rx->gyro_x_rad_s() : 0.0f;
}

chassis::Fsm::Input BuildChassisFsmInput(const InputSnapshot &input, const uint32_t tick_ms,
                                         const chassis::Chassis::UpdateOutput &chassis_output) {
  chassis::Fsm::Input fsm_input{};
  const auto &m = input.mode_request;
  fsm_input.request = {
      .input_valid = m.input_valid,
      .domain_request = m.domain_request,
      .leg_request = m.leg_request,
      .combat_profile = m.combat_profile,
      .spin_hold = m.spin_hold,
      .jump_trigger = m.jump_trigger,
      .current_leg_length_m = chassis_output.mean_leg_length_m,
      .theta_ll_rad = chassis_output.current_state.theta_ll,
      .theta_lr_rad = chassis_output.current_state.theta_lr,
      .fall_detected = m.fall_detected,
      .fall_detected_hold_ms = m.fall_detected_hold_ms,
      .upright_stable = m.upright_stable,
      .stair_climb_ready_for_done = chassis_output.stair_climb_ready_for_done,
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
      .chassis_recovery_active = chassis_output.mode == chassis::Fsm::State::kRecoveryFallCheck ||
                                 chassis_output.mode == chassis::Fsm::State::kRecoverySelfRight,
      .startup_align_complete = startup_align_complete,
  };
  return fsm_input;
}

}  // namespace wheel_legged::control_loop
