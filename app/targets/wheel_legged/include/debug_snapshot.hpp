#pragma once

#include <cstdint>

/**
 * @brief 调试快照，放置于 SRAM4 供调试器/DMA 直接读取
 */
struct __attribute__((packed, aligned(4))) DebugSnapshot {
  // ── 时间戳与状态机 ──
  uint32_t tick_ms;                    // 系统 tick
  uint8_t  chassis_fsm_state;          // 底盘状态机当前状态
  uint8_t  gimbal_fsm_state;           // 云台状态机当前状态
  uint8_t  chassis_fsm_state_changed;  // 底盘状态机本周期是否切换
  uint8_t  gimbal_fsm_state_changed;   // 云台状态机本周期是否切换

  // ── 遥控器原始输入 ──
  uint8_t  dr16_online;               // DR16 在线
  int32_t  dr16_switch_l_raw;         // 左拨杆原始位置
  int32_t  dr16_switch_r_raw;         // 右拨杆原始位置
  int16_t  dr16_dial_raw;             // 拨轮原始值
  int16_t  dr16_left_x_raw;           // 左摇杆 X
  int16_t  dr16_left_y_raw;           // 左摇杆 Y
  int16_t  dr16_right_x_raw;          // 右摇杆 X
  int16_t  dr16_right_y_raw;          // 右摇杆 Y

  // ── 遥控器/图传语义请求 ──
  uint8_t  input_domain_request;      // 解析后的工作域
  uint8_t  input_leg_profile;         // 腿长档位
  uint8_t  input_combat_profile;      // 战斗域子模式
  uint8_t  input_target_source;       // 目标来源
  uint8_t  input_host_target_valid;   // 上位机目标有效
  uint8_t  dr16_enable_request;       // 使能请求
  uint8_t  dr16_spin_request;         // 小陀螺请求
  uint8_t  dr16_jump_trigger_edge;    // 跳跃触发边沿
  uint8_t  tc_remote_valid;           // 图传键鼠链路活跃（收到键盘帧）
  uint8_t  tc_mid_leg_hold;           // 图传C键中腿长锁定
  uint16_t tc_keyboard_value;         // 图传键盘按键位掩码
  int16_t  tc_mouse_x;                // 图传鼠标 X 增量
  int16_t  tc_mouse_y;                // 图传鼠标 Y 增量
  uint8_t  tc_left_button;            // 图传鼠标左键
  uint8_t  tc_right_button;           // 图传鼠标右键
  float    gimbal_target_yaw_rad;     // 云台偏航目标
  float    gimbal_target_pitch_rad;   // 云台俯仰目标

  // ── 关节电机原始反馈 ──
  float    motor_raw_lf_pos_rad;      // 左前腿位置
  float    motor_raw_lf_vel_rad_s;    // 左前腿速度
  float    motor_raw_lf_tau_nm;       // 左前腿反馈力矩
  float    motor_raw_lb_pos_rad;      // 左后腿位置
  float    motor_raw_lb_vel_rad_s;    // 左后腿速度
  float    motor_raw_lb_tau_nm;       // 左后腿反馈力矩
  float    motor_raw_rf_pos_rad;      // 右前腿位置
  float    motor_raw_rf_vel_rad_s;    // 右前腿速度
  float    motor_raw_rf_tau_nm;       // 右前腿反馈力矩
  float    motor_raw_rb_pos_rad;      // 右后腿位置
  float    motor_raw_rb_vel_rad_s;    // 右后腿速度
  float    motor_raw_rb_tau_nm;       // 右后腿反馈力矩

  // ── 关节电机控制输出 ──
  float    motor_cmd_lf_tau_nm;       // 左前腿输出力矩
  float    motor_cmd_lb_tau_nm;       // 左后腿输出力矩
  float    motor_cmd_rf_tau_nm;       // 右前腿输出力矩
  float    motor_cmd_rb_tau_nm;       // 右后腿输出力矩

  // ── 轮毂电机 ──
  float    wheel_raw_left_rad_s;      // 左轮原始转速
  float    wheel_raw_right_rad_s;     // 右轮原始转速
  float    wheel_cmd_left_tau_nm;     // 左轮输出力矩
  float    wheel_cmd_right_tau_nm;    // 右轮输出力矩

  // ── 底盘 IMU 原始反馈 ──
  float    imu_raw_roll_rad;          // 横滚角
  float    imu_raw_pitch_rad;         // 俯仰角
  float    imu_raw_yaw_rad;           // 偏航角
  float    imu_raw_gyro_x_rad_s;      // 陀螺 X
  float    imu_raw_gyro_y_rad_s;      // 陀螺 Y
  float    imu_raw_gyro_z_rad_s;      // 陀螺 Z
  float    imu_raw_acc_x_mps2;        // 加速度 X
  float    imu_raw_acc_y_mps2;        // 加速度 Y
  float    imu_raw_acc_z_mps2;        // 加速度 Z

  // ── 云台 IMU ──
  float    gimbal_imu_pitch_rad;      // 云台 IMU 俯仰
  float    gimbal_imu_yaw_rad;        // 云台 IMU 偏航
  float    gimbal_imu_gyro_x_rad_s;   // 云台 IMU 陀螺 X（俯仰轴角速度）
  float    gimbal_imu_gyro_z_rad_s;   // 云台 IMU 陀螺 Z（偏航轴角速度）

  // ── 云台反馈与电机 ──
  float    yaw_motor_raw_pos_rad;           // 偏航 DM 电机编码器（仅归中模式用作位置反馈）
  float    gimbal_yaw_pos_feedback_rad;     // 偏航角度反馈值（来源：云台 IMU yaw 或电机编码器）
  float    gimbal_yaw_vel_feedback_rad_s;   // 偏航角速度反馈值（来源：云台 IMU 陀螺 Z）
  float    yaw_cmd_target_rad;              // 偏航目标角
  float    yaw_cmd_torque_nm;               // 偏航输出力矩
  uint8_t  yaw_motor_status;                // 偏航 DM 状态
  float    gimbal_pitch_pos_feedback_rad;   // 俯仰角度反馈值（来源：-云台 IMU pitch）
  float    gimbal_pitch_vel_feedback_rad_s; // 俯仰角速度反馈值（来源：云台 IMU 陀螺 X）
  float    pitch_cmd_target_rad;            // 俯仰目标角
  float    pitch_cmd_torque_nm;             // 俯仰输出力矩
  uint8_t  pitch_motor_status;              // 俯仰 DM 状态

  // ── 底盘模型状态向量 ──
  float    state_s_m;                 // 纵向位置
  float    state_s_dot_mps;           // 纵向速度
  float    expected_s_m;              // 期望纵向位置
  float    expected_s_dot_mps;        // 期望纵向速度
  float    state_phi_rad;             // 偏航角
  float    state_phi_dot_rad_s;       // 偏航角速度
  float    state_theta_ll_rad;        // 左腿摆角
  float    state_theta_ll_dot_rad_s;  // 左腿摆角速度
  float    state_theta_lr_rad;        // 右腿摆角
  float    state_theta_lr_dot_rad_s;  // 右腿摆角速度
  float    state_theta_b_rad;         // 车体俯仰角
  float    state_theta_b_dot_rad_s;   // 车体俯仰角速度
  float    state_l_l_m;               // 左腿等效长度
  float    state_l_r_m;               // 右腿等效长度

  // ── 底盘状态 ──
  float    chassis_mean_leg_length_m; // 平均腿长
  float    chassis_left_leg_length_m; // 左腿长度
  float    chassis_right_leg_length_m;// 右腿长度
  float    chassis_speed_mps;         // 车体融合速度
  float    chassis_left_force_n;          // 左腿竖直力
  float    chassis_right_force_n;         // 右腿竖直力
  float    chassis_left_support_force_n;  // 左腿支撑力
  float    chassis_right_support_force_n; // 右腿支撑力
  uint8_t  chassis_posture_valid;     // 姿态有效
  uint8_t  chassis_off_ground;        // 离地

  // ── 定点锁定 ──
  uint8_t  lock_point_enabled;        // 定点锁定使能
  uint8_t  lock_point_request;        // 锁点请求
  uint8_t  lock_point_captured;       // 锁点已捕获
  uint8_t  lock_point_rising_edge;    // 锁点上升沿
  uint8_t  lock_point_speed_below_threshold;  // filtered_s_dot 已斜坡归零

  // ── DYP 超声波 ──
  uint16_t dyp_distance_raw_left;     // 左超声波读数
  uint16_t dyp_distance_raw_right;    // 右超声波读数
  uint8_t  dyp_result_left;           // 左测量状态
  uint8_t  dyp_result_right;          // 右测量状态
  uint32_t dyp_frame_count;           // 接收帧计数

  // ── 发射机构（双摩擦变体）──
  uint8_t  shoot_enabled;             // 发射使能
  // ── 发射机构（三摩擦变体 hero）──
  float    booster_raw_pos_rad;       // DM 拨盘位置 (hero)
  float    fw_raw_rpm_1;              // 摩擦轮1 RPM (hero)
  float    fw_raw_rpm_2;              // 摩擦轮2 RPM (hero)
  float    fw_raw_rpm_3;              // 摩擦轮3 RPM (hero)

  // ── 裁判系统 ──
  uint8_t  referee_online;             // 裁判系统是否在线（收到过有效包）
  uint8_t  referee_robot_id;           // 裁判系统上报的机器人 ID
  float    referee_bullet_speed_mps;   // 最近一发弹丸初速度 [m/s]

  // ── 云台 IMU 欧拉角（Frame C: 0x112）──
  float    gimbal_euler_yaw_rad;       // 云台 IMU 偏航角
  float    gimbal_euler_pitch_rad;     // 云台 IMU 俯仰角
  float    gimbal_euler_roll_rad;      // 云台 IMU 横滚角

  // ── 自瞄通信 TX（发给 NUC）──
  uint8_t  aimbot_tx_mode;            // 发送的模式 (0=Normal,1=AutoAimNoMove,2=AutoAimWithMove)
  uint8_t  aimbot_tx_robot_id;        // 发送的机器人 ID

  // ── 自瞄通信 RX（NUC 反馈）──
  uint8_t  aimbot_rx_state;           // NUC 反馈的自瞄状态
  uint8_t  aimbot_rx_target;          // NUC 反馈的目标信息
  uint8_t  aimbot_rx_nuc_start_flag;  // NUC 启动标志
  float    aimbot_rx_yaw_rad;         // NUC 下发的偏航目标
  float    aimbot_rx_pitch_rad;       // NUC 下发的俯仰目标

};
static_assert(sizeof(DebugSnapshot) <= 512, "DebugSnapshot must fit in 512 bytes for efficient DMA");

extern DebugSnapshot wl_debug;
