#include "include/gimbal/fsm.hpp"

/**
 * @file  targets/wheel_legged/gimbal_fsm.cc
 * @brief 云台状态机实现
 */

namespace {

/**
 * @brief 判断统一语义请求是否要求云台关闭
 */
bool IsInputDisabled(const wheel_legged::GimbalFsmInput &request) {
  return !request.input_valid || request.domain_request == wheel_legged::DomainRequest::kDisabled;
}

/**
 * @brief 在输入有效时解析云台常规工作模式
 */
gimbal::Fsm::State ResolveNormalMode(const gimbal::Fsm::Input &input) {
  const wheel_legged::GimbalFsmInput &request = input.request;

  if (IsInputDisabled(request)) {
    return gimbal::Fsm::State::kDisabled;
  }
  if (request.yaw_centering_for_recovery) {
    return gimbal::Fsm::State::kRecoveryYawCentering;
  }
  if (request.chassis_recovery_active) {
    return gimbal::Fsm::State::kRecoveryAlign;
  }
  if (request.domain_request == wheel_legged::DomainRequest::kCombat) {
    return gimbal::Fsm::State::kCombat;
  }
  return request.service_profile == wheel_legged::ServiceProfile::kChassisAndGimbalSafe
             ? gimbal::Fsm::State::kServiceSafe
             : gimbal::Fsm::State::kServiceWithFire;
}

/**
 * @brief 解析含启动归中约束的云台目标模式
 */
gimbal::Fsm::State ResolveMode(const gimbal::Fsm::Input &input, const gimbal::Fsm::State current_mode) {
  const wheel_legged::GimbalFsmInput &request = input.request;
  if (IsInputDisabled(request)) {
    return gimbal::Fsm::State::kDisabled;
  }

  // 辨识/前馈验证模式：直接由 GimbalTestProfile 决定，不走普通模式路由
  if (request.gimbal_test_profile == wheel_legged::GimbalTestProfile::kIdent) {
    return gimbal::Fsm::State::kIdent;
  }
  if (request.gimbal_test_profile == wheel_legged::GimbalTestProfile::kFfVerify) {
    return gimbal::Fsm::State::kFfVerify;
  }

  const gimbal::Fsm::State normal_mode = ResolveNormalMode(input);
  // 恢复归中优先：绕过 startup_align_complete 限制，从任意状态立即切入
  // 避免 chassis_posture_invalid 重置 startup_align_complete 后云台卡在 kStartupAlign
  // 导致云台被禁用、yaw 无法归中、底盘 theta 恢复死锁
  if (normal_mode == gimbal::Fsm::State::kRecoveryYawCentering) {
    return normal_mode;
  }
  if (current_mode == gimbal::Fsm::State::kDisabled) {
    return gimbal::Fsm::State::kStartupAlign;
  }
  if (current_mode == gimbal::Fsm::State::kStartupAlign) {
    return request.startup_align_complete ? normal_mode : gimbal::Fsm::State::kStartupAlign;
  }
  return request.startup_align_complete ? normal_mode : gimbal::Fsm::State::kStartupAlign;
}

/**
 * @brief 根据云台模式生成控制动作
 */
gimbal::Fsm::Output::ControlOutput BuildControlOutput(const gimbal::Fsm::Input &input, const gimbal::Fsm::State mode) {
  gimbal::Fsm::Output::ControlOutput control{};
  const wheel_legged::GimbalFsmInput &request = input.request;

  const bool use_host_target = request.target_source == wheel_legged::TargetSource::kHost && request.host_target_valid;
  control.active_target_source = use_host_target ? wheel_legged::TargetSource::kHost : wheel_legged::TargetSource::kRc;
  control.gimbal_target = use_host_target ? request.host_target : request.rc_target;

  switch (mode) {
    case gimbal::Fsm::State::kDisabled:
      control.gimbal_enable = false;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kStartupAlign:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kServiceWithFire:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kServiceSafe:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kCombat:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kRecoveryAlign:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = true;
      break;

    case gimbal::Fsm::State::kRecoveryYawCentering:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kIdent:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      control.gimbal_test_profile = request.gimbal_test_profile;
      break;

    case gimbal::Fsm::State::kFfVerify:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      control.gimbal_test_profile = request.gimbal_test_profile;
      break;
  }

  return control;
}

}  // namespace

void gimbal::Fsm::Init() {
  mode_ = State::kDisabled;
  output_ = {};
  output_.mode = mode_;
  output_.control = BuildControlOutput(Input{}, mode_);
}

void gimbal::Fsm::Transit(const State new_mode) {
  output_.state_changed = (new_mode != mode_);
  mode_ = new_mode;
  output_.mode = mode_;
}

gimbal::Fsm::Output gimbal::Fsm::Update(const Input &input) {
  output_.state_changed = false;

  const State requested = ResolveMode(input, mode_);
  if (requested != mode_) {
    Transit(requested);
  }

  output_.control = BuildControlOutput(input, mode_);
  return output_;
}
