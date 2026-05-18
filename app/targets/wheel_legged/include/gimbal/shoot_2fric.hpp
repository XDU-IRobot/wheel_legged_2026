#pragma once

#include <cstdint>

#include "common/controllers/shoot_2firc.hpp"
#include "common/encoder_counter.hpp"

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
                     bool fire_flag, bool shoot_enabled, float fric_speed_target_rpm, bool auto_aim);

  bool enabled() const { return enabled_; }
  uint32_t shot_count() const { return shot_count_; }
  void ResetShotCount() { shot_count_ = 0; }
  int32_t dial_linear_pos() const { return dial_encoder_counter_.linear_ticks(); }
  auto &dial_encoder_counter() { return dial_encoder_counter_; }

 private:
  bool enabled_{false};
  bool fric_ready_{false};
  bool prev_fire_flag_{false};
  uint32_t shot_count_{0};
  EncoderCounter dial_encoder_counter_;
  Shoot2Fric controller_{9, 37.58f};
};
