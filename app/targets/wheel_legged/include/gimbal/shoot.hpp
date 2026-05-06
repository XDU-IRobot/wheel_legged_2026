#pragma once

#include <cstdint>

#include "common/controllers/shoot_2firc.hpp"

/**
 * @brief 发射机构控制输出
 */
struct ShootOutput {
  float fric_left_current{0.0f};
  float fric_right_current{0.0f};
  float dial_current{0.0f};
};

/**
 * @brief 发射机构控制器（双摩擦轮 + M3508 拨盘，Infantry3/4）
 * @note  仅做控制计算，返回电流值；电机 IO 由 Actuators::ApplyShootOutput 统一下发。
 */
class Shoot {
 public:
  void Init();
  void Enable();
  void Disable();
  ShootOutput Update(float fric_left_rpm, float fric_right_rpm, float dial_encoder, float dial_rpm, float dt,
                     int16_t dr16_dial, bool mouse_left, bool shoot_enabled);

  bool enabled() const { return enabled_; }

 private:
  bool enabled_{false};
  Shoot2Fric controller_{9, 42.75f};
};
