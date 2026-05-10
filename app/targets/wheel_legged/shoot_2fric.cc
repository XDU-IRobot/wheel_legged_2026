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
                          int16_t dr16_dial, bool mouse_left, bool shoot_enabled) {
  ShootOutput out{};

  if (shoot_enabled) {
    controller_.Enable(true);
    controller_.Arm(true);
    controller_.SetArmSpeed(ns::kFricSpeedTargetRpm);

    if (dr16_dial < ns::kDialFireThreshold || mouse_left) {
      controller_.SetMode(Shoot2Fric::kFullAuto);
      controller_.SetShootFrequency(ns::kShootFrequencyHz);
    } else {
      controller_.SetMode(Shoot2Fric::kStop);
    }
    controller_.Fire();
  } else {
    controller_.Enable(false);
  }

  controller_.Update(fric_left_rpm, fric_right_rpm, dial_encoder, dial_rpm, dt);

  out.fric_left_current = std::clamp(controller_.output().fric_1, -16000.0f, 16000.0f);
  out.fric_right_current = std::clamp(controller_.output().fric_2, -16000.0f, 16000.0f);
  out.dial_current = std::clamp(-controller_.output().loader, -16000.0f, 16000.0f);
  return out;
}

#endif  // WHEEL_LEGGED_ROBOT_VARIANT != 1
