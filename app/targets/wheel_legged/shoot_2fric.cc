#if WHEEL_LEGGED_ROBOT_VARIANT != 1

#include "include/gimbal/shoot_2fric.hpp"

#include <algorithm>

#include "common/controllers/shoot_2firc.hpp"
#include "include/params.hpp"

namespace ns = wheel_legged::params::active::shoot;

void Shoot::Init() {
  auto &spid = controller_.pid();
  const auto init_pid = [](rm::modules::PID &pid, const wheel_legged::params::PidGains &gains) {
    pid.SetKp(gains.kp).SetKi(gains.ki).SetKd(gains.kd).SetMaxOut(gains.max_out).SetMaxIout(gains.max_iout);
    pid.Clear();
  };
  init_pid(spid.fric_1_speed, ns::kFricSpeedPid);
  init_pid(spid.fric_2_speed, ns::kFricSpeedPid);
  init_pid(spid.loader_speed, ns::kDialSpeedPid);
  init_pid(spid.loader_position, ns::kDialPositionPid);
}

void Shoot::Enable() { enabled_ = true; }

void Shoot::Disable() { enabled_ = false; }

ShootOutput Shoot::Update(float fric_left_rpm, float fric_right_rpm, float dial_encoder, float dial_rpm, float dt,
                          bool fire_flag, bool shoot_enabled, float fric_speed_target_rpm, bool auto_aim) {
  ShootOutput out{};

  if (shoot_enabled) {
    controller_.Enable(true);
    controller_.Arm(true);
    controller_.SetArmSpeed(fric_speed_target_rpm);

    if (auto_aim) {
      // 自瞄模式：单发，拨轮上升沿触发
      controller_.SetMode(Shoot2Fric::kSingleShot);
      const bool fire_rising_edge = fire_flag && !prev_fire_flag_;
      if (fire_rising_edge) {
        controller_.Fire();
      }
    } else {
      // 普通模式：连发
      if (fire_flag) {
        controller_.SetMode(Shoot2Fric::kFullAuto);
        controller_.SetShootFrequency(ns::kShootFrequencyHz);
      } else {
        controller_.SetMode(Shoot2Fric::kStop);
      }
      controller_.Fire();
    }
    prev_fire_flag_ = fire_flag;
  } else {
    controller_.Enable(true);
    controller_.Arm(true);
    controller_.SetArmSpeed(0);
    prev_fire_flag_ = false;
  }

  controller_.Update(-fric_left_rpm, -fric_right_rpm, dial_encoder, dial_rpm, dt);

  // 打弹检测：摩擦轮达速后降速超过 200 rpm 记为一发
  {
    constexpr float kReadyThresholdRpm = 50.0f;
    constexpr float kDropThresholdRpm = 200.0f;
    const float target_abs = std::fabs(fric_speed_target_rpm);
    const float left_abs = std::fabs(fric_left_rpm);
    const float right_abs = std::fabs(fric_right_rpm);

    if (target_abs > 0.0f && left_abs >= target_abs - kReadyThresholdRpm &&
        right_abs >= target_abs - kReadyThresholdRpm) {
      fric_ready_ = true;
    }

    if (fric_ready_ && (target_abs - left_abs > kDropThresholdRpm || target_abs - right_abs > kDropThresholdRpm)) {
      ++shot_count_;
      fric_ready_ = false;
    }
  }

  out.fric_left_current = std::clamp(controller_.output().fric_1, -16000.0f, 16000.0f);
  out.fric_right_current = std::clamp(controller_.output().fric_2, -16000.0f, 16000.0f);
  out.dial_current = std::clamp(-controller_.output().loader, -16000.0f, 16000.0f);
  return out;
}

#endif  // WHEEL_LEGGED_ROBOT_VARIANT != 1
