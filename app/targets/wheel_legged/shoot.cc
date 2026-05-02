#include "include/shoot.hpp"
#include "include/globals.hpp"

#include <algorithm>

namespace ns = wheel_legged::params::active::shoot;

void Shoot::Init(SharedResources &g) {
  auto &spid = g.shoot_controller.pid();
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

void Shoot::Update(SharedResources &g, float dt, int16_t dr16_dial, bool mouse_left) {
  auto &sc = g.shoot_controller;

  if (enabled_) {
    sc.Enable(true);
    sc.Arm(true);
    sc.SetArmSpeed(ns::kFricSpeedTargetRpm);

    // DR16 拨轮或图传鼠标左键控制拨盘连发
    if (dr16_dial < ns::kDialFireThreshold || mouse_left) {
      sc.SetMode(Shoot2Fric::kFullAuto);
      sc.SetShootFrequency(ns::kShootFrequencyHz);
    } else {
      sc.SetMode(Shoot2Fric::kStop);
    }
    sc.Fire();
  } else {
    sc.Enable(false);
  }

  const float fric_left_rpm = g.fric_left.has_value() ? static_cast<float>(g.fric_left->rpm()) : 0.0f;
  const float fric_right_rpm = g.fric_right.has_value() ? static_cast<float>(g.fric_right->rpm()) : 0.0f;
  const float dial_encoder = g.dial.has_value() ? -static_cast<float>(g.dial->encoder()) : 0.0f;
  const float dial_rpm = g.dial.has_value() ? -static_cast<float>(g.dial->rpm()) : 0.0f;

  sc.Update(fric_left_rpm, fric_right_rpm, dial_encoder, dial_rpm, dt);

  if (g.fric_left.has_value()) {
    g.fric_left->SetCurrent(static_cast<int16_t>(std::clamp(sc.output().fric_1, -16000.0f, 16000.0f)));
  }
  if (g.fric_right.has_value()) {
    g.fric_right->SetCurrent(static_cast<int16_t>(std::clamp(sc.output().fric_2, -16000.0f, 16000.0f)));
  }
  if (g.dial.has_value()) {
    g.dial->SetCurrent(static_cast<int16_t>(std::clamp(-sc.output().loader, -16000.0f, 16000.0f)));
  }
  if (g.gimbal_can.has_value()) {
    rm::device::DjiMotorBase::SendCommand(*g.gimbal_can);
  }
}
