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
                     bool fire_flag, bool shoot_enabled, float fric_speed_target_rpm, bool single_shot = false);

  bool enabled() const { return enabled_; }
  uint32_t shot_count() const { return shot_count_; }
  void ResetShotCount() { shot_count_ = 0; }
  int32_t dial_linear_pos() const { return dial_encoder_counter_.linear_ticks(); }
  auto &dial_encoder_counter() { return dial_encoder_counter_; }

  void SetHeatParams(uint16_t heat_limit, uint16_t cooling_rate) {
    if (heat_limit > 0) {
      heat_limit_ = heat_limit;
    }
    if (cooling_rate > 0) {
      cooling_rate_ = cooling_rate;
    }
  }
  float current_heat() const { return current_heat_; }
  uint16_t heat_limit() const { return heat_limit_; }
  uint16_t cooling_rate() const { return cooling_rate_; }
  bool heat_over_limit() const { return heat_suppressed_; }
  uint8_t shoot_mode() const { return static_cast<uint8_t>(controller_.mode()); }
  bool single_complete() const { return controller_.shoot_flag(); }
  float loader_pos_error() const { return controller_.position(); }
  float loader_pos_target() const { return controller_.target().loader_position; }
  float loader_pos_pid_out() const { return controller_.pid().loader_position.out(); }
  float loader_spd_target() const { return controller_.target().loader_speed; }
  float loader_spd_pid_out() const { return controller_.pid().loader_speed.out(); }

 private:
  bool enabled_{false};
  bool fric_ready_{false};
  bool prev_fire_flag_{false};
  uint32_t shot_count_{0};
  EncoderCounter dial_encoder_counter_;
  Shoot2Fric controller_{9, 42.75f};

  float current_heat_{0.0f};
  uint16_t heat_limit_{240};
  uint16_t cooling_rate_{40};
  bool heat_suppressed_{false};

  // 单发间隔计数器：每周期自增，达到阈值后允许下一发
  static constexpr uint32_t kSingleShotCycleThreshold = 200;  // 0.4s @ 500Hz
  uint32_t shoot_cycle_counter_{0};
};
