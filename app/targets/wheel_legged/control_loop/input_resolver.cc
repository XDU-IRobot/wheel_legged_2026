/**
 * @file  targets/wheel_legged/control_loop/input_resolver.cc
 * @brief 硬件输入采集与语义折叠实现
 */

#include "input_resolver.hpp"

#include <algorithm>
#include <cmath>

#include "../include/actuators.hpp"
#include "../include/globals.hpp"
#include "../include/wheel_legged_params.hpp"

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
constexpr float kPitchTargetMinRad = params::active::control_loop::kPitchTargetMinRad;
constexpr float kPitchTargetMaxRad = params::active::control_loop::kPitchTargetMaxRad;

constexpr uint16_t kRcKeyW = 0x0001;
constexpr uint16_t kRcKeyS = 0x0002;
constexpr uint16_t kRcKeyA = 0x0004;
constexpr uint16_t kRcKeyD = 0x0008;
constexpr uint16_t kRcKeyShift = 0x0010;
constexpr uint16_t kRcKeyC = 0x2000;

}  // namespace

float NormalizeDr16Axis(const int16_t axis, const int16_t axis_max_abs) {
  return rm::modules::Clamp(static_cast<float>(axis) / static_cast<float>(axis_max_abs), -1.0f, 1.0f);
}

DriveInputNorm ResolveDriveInput(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote) {
  DriveInputNorm out{};

  // DR16 摇杆优先；仅在 DR16 离线时降级为图传键盘 WASD
  if (dr16.online) {
    out.forward = NormalizeDr16Axis(dr16.right_y, kDr16AxisMaxAbs);
    out.side = NormalizeDr16Axis(dr16.right_x, kDr16AxisMaxAbs);
  } else if (tc_remote.valid) {
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

void ResolveInputSemantics(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote,
                           Dr16SemanticState &semantic_state, TcSemanticState &tc_state, InputSnapshot &input) {
  const bool tc_remote_active = tc_remote.valid;
  const bool has_any_input = dr16.online || tc_remote_active;

  // ── 图传 C 键：上升沿切换中腿长保持（仅在 DR16 离线时生效）──
  if (!dr16.online && tc_remote_active) {
    const bool c_pressed = (tc_remote.keyboard_value & kRcKeyC) != 0U;
    if (c_pressed && tc_state.mid_leg_c_armed) {
      tc_state.mid_leg_hold = !tc_state.mid_leg_hold;
      tc_state.mid_leg_c_armed = false;
    }
    if (!c_pressed) tc_state.mid_leg_c_armed = true;
  }

  input.input_valid = has_any_input;
  input.dr16 = dr16;
  input.tc_remote = tc_remote;
  const float yaw_motor_pos_rad = input.estimator_input.yaw_motor_rad;

  // ── 构建整车语义请求 ──
  wheel_legged::ModeRequest request{};
  request.input_valid = has_any_input;
  request.service_profile = wheel_legged::ServiceProfile::kChassisAndGimbalSafe;
  // @todo: 目前仅使用 kChassisAndGimbalSafe 单一 Service 策略；未来如需区分 Service 子模式再接入 DR16 输入。

  // 工作域：DR16 左拨杆优先；仅 DR16 离线时由 TC 键鼠链路接管
  if (dr16.online) {
    request.domain_request = ResolveDomainRequest(dr16.switch_l);
  } else if (tc_remote_active) {
    request.domain_request = wheel_legged::DomainRequest::kCombat;
  } else {
    request.domain_request = wheel_legged::DomainRequest::kDisabled;
  }

  // ── 战斗域子模式与腿长档位 ──
  // DR16 在线时由右拨杆决定，C 键不参与；DR16 离线时 TC 键鼠接管，默认中腿。
  wheel_legged::CombatProfile combat_profile = wheel_legged::CombatProfile::kNormal;
  wheel_legged::LegProfile leg_request = wheel_legged::LegProfile::kLow;

  if (request.domain_request == wheel_legged::DomainRequest::kCombat) {
    if (dr16.online) {
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
          leg_request = wheel_legged::LegProfile::kMid;
          break;
        default:
          break;
      }
    } else {
      // TC 键鼠链路：战斗域固定 Normal + 中腿
      combat_profile = wheel_legged::CombatProfile::kNormal;
      leg_request = wheel_legged::LegProfile::kMid;
    }
  } else {
    // 非战斗域仅 DR16 在线可达（TC 强制 Combat），由右拨杆决定
    leg_request = ResolveLegProfile(dr16.switch_r);
  }
  request.leg_request = leg_request;

  // ── 目标来源：自瞄模式优先上位机，上位机无效时降级为 RC ──
  // @todo: 上位机自瞄目标尚未接入，host_target_valid 始终为 false，host_target 始终为零。
  // 接入后需从 CAN/串口读取上位机目标并填充 host_target / host_target_valid。
  const bool host_target_valid = false;
  request.host_target_valid = host_target_valid;
  if (combat_profile == wheel_legged::CombatProfile::kAutoAimNoMove ||
      combat_profile == wheel_legged::CombatProfile::kAutoAimWithMove) {
    request.target_source = host_target_valid ? wheel_legged::TargetSource::kHost : wheel_legged::TargetSource::kRc;
  } else {
    request.target_source = wheel_legged::TargetSource::kRc;
  }
  request.combat_profile = combat_profile;

  // ── 小陀螺触发：DR16 拨轮优先；仅 DR16 离线时由 Shift 键接管 ──
  request.spin_hold = (dr16.online && dr16.dial >= kWheelSpinThreshold) ||
                      (!dr16.online && tc_remote_active && (tc_remote.keyboard_value & kRcKeyShift) != 0U);

  // ── 跳跃触发：拨轮回中后快速负推（非 combat 域下）──
  if (std::abs(dr16.dial) <= kWheelCenterThreshold) {
    semantic_state.wheel_action_armed = true;
  }
  const bool wheel_action_trigger = semantic_state.wheel_action_armed && (dr16.dial <= -kWheelActionThreshold);
  if (wheel_action_trigger) {
    semantic_state.wheel_action_armed = false;
  }
  request.jump_trigger =
      dr16.online && wheel_action_trigger && request.domain_request != wheel_legged::DomainRequest::kCombat;

  // ── RC 积分目标更新 ──
  if ((!dr16.online && !tc_remote_active) || request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    // 无输入或关闭时，目标归位到当前偏航电机角
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
    if (dr16.online) {
      yaw_delta += static_cast<float>(dr16.left_x) / kRcStickMax * kRcYawRateMaxRadS * kControlLoopDtS;
      pitch_delta += static_cast<float>(dr16.left_y) / kRcStickMax * kRcPitchRateMaxRadS * kControlLoopDtS;
    } else if (tc_remote_active) {
      // DR16 离线时降级为图传鼠标
      yaw_delta += static_cast<float>(tc_remote.mouse_x) / kTcMouseMax * kTcMouseYawRateMaxRadS * kControlLoopDtS;
      pitch_delta += static_cast<float>(tc_remote.mouse_y) / kTcMouseMax * kTcMousePitchRateMaxRadS * kControlLoopDtS;
    }
    semantic_state.rc_target.yaw_rad =
        rm::modules::Wrap(semantic_state.rc_target.yaw_rad + yaw_delta, -params::active::kPi, params::active::kPi);
    semantic_state.rc_target.pitch_rad =
        std::clamp(semantic_state.rc_target.pitch_rad + pitch_delta, kPitchTargetMinRad, kPitchTargetMaxRad);
  }
  request.rc_target = semantic_state.rc_target;

  // @todo: 倒地检测未接入 IMU 姿态判断，目前始终无倒地。
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
  const bool keyboard_rx_valid = gimbal_rx_ready && g.gimbal_rx->tc_link_activated();

  Dr16RawInput dr16{
      .online = (g.dr16.online_status() == rm::device::Device::kOk),
      .switch_l = g.dr16.switch_l(),
      .switch_r = g.dr16.switch_r(),
      .right_y = g.dr16.right_y(),
      .right_x = g.dr16.right_x(),
      .left_x = g.dr16.left_x(),
      .left_y = g.dr16.left_y(),
      .dial = g.dr16.dial(),
  };
  TcRemoteInput tc_remote{};
  if (keyboard_rx_valid) {
    tc_remote.valid = true;
    tc_remote.mouse_x = g.gimbal_rx->mouse_x();
    tc_remote.mouse_y = g.gimbal_rx->mouse_y();
    tc_remote.left_button = g.gimbal_rx->left_button();
    tc_remote.right_button = g.gimbal_rx->right_button();
    tc_remote.keyboard_value = g.gimbal_rx->keyboard_value();
  }

  // 3. 语义折叠
  ResolveInputSemantics(dr16, tc_remote, semantic_state, tc_state, input);

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
