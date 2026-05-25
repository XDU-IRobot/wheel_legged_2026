#pragma once

#include <cstdint>

struct UiSnapshot {
  bool referee_online{false};
  uint8_t referee_robot_id{0};

  float gimbal_pitch_rad{0.0f};
  float gimbal_yaw_rad{0.0f};
  float yaw_motor_raw_pos_rad{0.0f};

  float left_leg_length_m{0.0f};
  float right_leg_length_m{0.0f};
  float left_leg_theta_rad{0.0f};
  float right_leg_theta_rad{0.0f};
  bool leg_view_flip{false};

  uint8_t chassis_fsm_state{0};
  uint8_t domain_request{0};
  uint8_t combat_profile{0};
  uint8_t aim_mode{0};        ///< 0=kAmmo, 1=kFuSmall, 2=kFuBig
  bool auto_aim_hold{false};  ///< 右键是否按下
  bool standby{false};
  bool spin_active{false};
  bool cross_active{false};

  float supercap_cap_energy{0.0f};
  float fric_left_rpm{0.0f};
  float fric_right_rpm{0.0f};
  float fw_raw_rpm_1{0.0f};
  float fw_raw_rpm_2{0.0f};
  float fw_raw_rpm_3{0.0f};
  float bullet_speed_mps{0.0f};
  uint16_t projectile_allowance{0};
};

inline UiSnapshot ui_snapshot{};
