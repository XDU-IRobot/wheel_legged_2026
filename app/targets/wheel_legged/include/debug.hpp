#pragma once

#include <cstdint>

#include "chassis/chassis.hpp"
#include "chassis/stair_climb_sequence.hpp"
#include "chassis/stair_task_coordinator.hpp"
#include "gimbal/gimbal.hpp"
#include "input.hpp"

/**
 * @file  targets/wheel_legged/include/debug.hpp
 * @brief 调试快照结构体（SRAM4）与填充函数声明
 */

/**
 * @brief 调试快照，放置于 SRAM4 供调试器/DMA 直接读取
 */
struct __attribute__((packed, aligned(4))) DebugSnapshot {
  // ── 时间戳与状态机 ──
  uint32_t tick_ms;                   // 系统 tick
  uint8_t chassis_fsm_state;          // 底盘状态机当前状态
  uint8_t gimbal_fsm_state;           // 云台状态机当前状态
  uint8_t chassis_fsm_state_changed;  // 底盘状态机本周期是否切换
  uint8_t gimbal_fsm_state_changed;   // 云台状态机本周期是否切换

  // Compact 500 Hz control-loop timing (kept small to preserve the 1024-byte snapshot limit)
  uint16_t control_dt_us;
  uint16_t control_dt_min_us;
  uint16_t control_dt_max_us;
  uint16_t control_jitter_max_us;
  uint16_t control_exec_last_us;
  uint16_t control_exec_max_us;
  uint32_t control_jitter_over_100us_count;
  uint32_t control_overrun_count;

  // ── 遥控器原始输入 ──
  uint8_t dr16_online;        // DR16 在线
  int32_t dr16_switch_l_raw;  // 左拨杆原始位置
  int32_t dr16_switch_r_raw;  // 右拨杆原始位置
  int16_t dr16_dial_raw;      // 拨轮原始值
  int16_t dr16_left_x_raw;    // 左摇杆 X
  int16_t dr16_left_y_raw;    // 左摇杆 Y
  int16_t dr16_right_x_raw;   // 右摇杆 X
  int16_t dr16_right_y_raw;   // 右摇杆 Y
  int16_t dr16_mouse_x;       // DR16 鼠标 X
  int16_t dr16_mouse_y;       // DR16 鼠标 Y
  uint8_t dr16_mouse_left;    // DR16 鼠标左键
  uint8_t dr16_mouse_right;   // DR16 鼠标右键
  uint16_t dr16_keyboard;     // DR16 键盘按键位掩码

  // ── 遥控器/图传语义请求 ──
  uint8_t input_domain_request;     // 解析后的工作域
  uint8_t input_leg_profile;        // 腿长档位
  uint8_t input_combat_profile;     // 战斗域子模式
  uint8_t input_target_source;      // 目标来源
  uint8_t input_host_target_valid;  // 上位机目标有效
  uint8_t dr16_enable_request;      // 使能请求
  uint8_t dr16_spin_request;        // 小陀螺请求
  uint8_t dr16_jump_trigger_edge;   // 跳跃触发边沿
  uint8_t auto_jump_triggered;      // 自动跳跃触发（TOF 触发）
  uint8_t auto_jump_enabled;        // 自动跳跃模式是否开启
  uint8_t auto_jump_both_close;     // 调试：both_close 条件
  uint8_t auto_jump_tof_armed;      // 调试：auto_jump_tof_armed 条件
  uint8_t auto_jump_both_active;    // 调试：both_active 条件（含200ms消抖）
  uint8_t auto_jump_trigger_ready;  // 调试：both_close && tof_armed && both_active
  uint8_t stair_descend_request;    // 下台阶模式请求
  uint8_t stair_descend_ready;      // 向下 ToF 数据已准备完成
  uint8_t stair_descend_triggered;  // 双侧向下 ToF 单帧触发
  uint8_t tc_remote_valid;          // 图传键鼠链路活跃（收到键盘帧）
  uint8_t stair_high_leg_request;   // Stair coordinator requests high-leg standby
  uint8_t stair_task_request;       // Stair command parsed this cycle
  uint8_t stair_task_mode;          // Stair coordinator state
  uint8_t stair_requested_attempts;
  uint8_t stair_completed_attempts;
  uint8_t stair_phase;  // Active sequence phase
  uint8_t stair_abort_reason;
  float stair_target_leg_length_m;
  float stair_target_theta_ll_rad;
  float stair_target_theta_lr_rad;
  float stair_theta_ll_error_rad;
  float stair_theta_lr_error_rad;
  float stair_leg_length_error_m;
  float stair_t_bl_cmd;  ///< 上台阶左腿摆角控制输出
  float stair_t_br_cmd;  ///< 上台阶右腿摆角控制输出
  uint32_t stair_phase_elapsed_ms;
  uint32_t stair_stable_elapsed_ms;
  uint8_t reset_yaw_request;      // R键重置正方向请求
  uint16_t tc_keyboard_value;     // 图传键盘按键位掩码
  int16_t tc_mouse_x;             // 图传鼠标 X 增量
  int16_t tc_mouse_y;             // 图传鼠标 Y 增量
  uint8_t tc_left_button;         // 图传鼠标左键
  uint8_t tc_right_button;        // 图传鼠标右键
  uint8_t tc_ui_refresh_key;      // E 键按下（UI 刷新使能）
  float gimbal_target_yaw_rad;    // 云台偏航目标
  float gimbal_target_pitch_rad;  // 云台俯仰目标

  // ── 关节电机原始反馈 ──
  float motor_raw_lf_pos_rad;    // 左前腿位置
  float motor_raw_lf_vel_rad_s;  // 左前腿速度
  float motor_raw_lf_tau_nm;     // 左前腿反馈力矩
  float motor_raw_lb_pos_rad;    // 左后腿位置
  float motor_raw_lb_vel_rad_s;  // 左后腿速度
  float motor_raw_lb_tau_nm;     // 左后腿反馈力矩
  float motor_raw_rf_pos_rad;    // 右前腿位置
  float motor_raw_rf_vel_rad_s;  // 右前腿速度
  float motor_raw_rf_tau_nm;     // 右前腿反馈力矩
  float motor_raw_rb_pos_rad;    // 右后腿位置
  float motor_raw_rb_vel_rad_s;  // 右后腿速度
  float motor_raw_rb_tau_nm;     // 右后腿反馈力矩

  // ── 关节电机控制输出 ──
  float motor_cmd_lf_tau_nm;  // 左前腿输出力矩
  float motor_cmd_lb_tau_nm;  // 左后腿输出力矩
  float motor_cmd_rf_tau_nm;  // 右前腿输出力矩
  float motor_cmd_rb_tau_nm;  // 右后腿输出力矩

  // ── 轮毂电机 ──
  float wheel_raw_left_rad_s;    // 左轮原始转速
  float wheel_raw_right_rad_s;   // 右轮原始转速
  float wheel_cmd_left_tau_nm;   // 左轮输出力矩
  float wheel_cmd_right_tau_nm;  // 右轮输出力矩

  // ── 底盘 IMU 原始反馈 ──
  float imu_raw_roll_rad;      // 横滚角
  float imu_raw_pitch_rad;     // 俯仰角
  float imu_raw_yaw_rad;       // 偏航角
  float imu_raw_gyro_x_rad_s;  // 陀螺 X
  float imu_raw_gyro_y_rad_s;  // 陀螺 Y
  float imu_raw_gyro_z_rad_s;  // 陀螺 Z
  float imu_raw_acc_x_mps2;    // 加速度 X
  float imu_raw_acc_y_mps2;    // 加速度 Y
  float imu_raw_acc_z_mps2;    // 加速度 Z

  // ── 云台 IMU ──
  float gimbal_imu_pitch_rad;     // 云台 IMU 俯仰
  float gimbal_imu_yaw_rad;       // 云台 IMU 偏航
  float gimbal_imu_gyro_x_rad_s;  // 云台 IMU 陀螺 X（俯仰轴角速度）
  float gimbal_imu_gyro_z_rad_s;  // 云台 IMU 陀螺 Z（偏航轴角速度）

  // ── 云台反馈与电机 ──
  float yaw_motor_raw_pos_rad;            // 偏航 DM 电机编码器（仅归中模式用作位置反馈）
  float pitch_motor_raw_pos_rad;          // 俯仰 DM 电机编码器
  float gimbal_yaw_pos_feedback_rad;      // 偏航角度反馈值（来源：云台 IMU yaw 或电机编码器）
  float gimbal_yaw_vel_feedback_rad_s;    // 偏航角速度反馈值（来源：云台 IMU 陀螺 Z）
  float yaw_cmd_target_rad;               // 偏航目标角
  float yaw_cmd_torque_nm;                // 偏航输出力矩
  uint8_t yaw_motor_status;               // 偏航 DM 状态
  float gimbal_pitch_pos_feedback_rad;    // 俯仰角度反馈值（来源：-云台 IMU pitch）
  float gimbal_pitch_vel_feedback_rad_s;  // 俯仰角速度反馈值（来源：云台 IMU 陀螺 X）
  float pitch_cmd_target_rad;             // 俯仰目标角
  float pitch_cmd_torque_nm;              // 俯仰输出力矩
  uint8_t pitch_motor_status;             // 俯仰 DM 状态

  // ── 云台动力学前馈 ──
  float gimbal_yaw_dq_rad_s;      // 偏航目标角速度
  float gimbal_pitch_dq_rad_s;    // 俯仰目标角速度
  float gimbal_yaw_ddq_rad_s2;    // 偏航目标角加速度
  float gimbal_pitch_ddq_rad_s2;  // 俯仰目标角加速度
  float ff_yaw_inertia;           // 偏航惯性力矩
  float ff_yaw_gravity;           // 偏航重力项
  float ff_yaw_friction;          // 偏航摩擦项
  float ff_pitch_coupling;        // 俯仰耦合项
  float ff_pitch_inertia;         // 俯仰惯性项
  float ff_pitch_gravity;         // 俯仰重力项
  float ff_pitch_friction;        // 俯仰摩擦项

  // ── 底盘模型状态向量 ──
  float state_s_m;                 // 纵向位置
  float state_s_dot_mps;           // 纵向速度
  float expected_s_m;              // 期望纵向位置
  float expected_s_dot_mps;        // 期望纵向速度（= filtered_s_dot）
  float filtered_s_dot_mps;        // 斜坡滤波后的纵向速度（连接减速斜坡与 I 项积分）
  float state_phi_rad;             // 偏航角
  float state_phi_dot_rad_s;       // 偏航角速度
  float state_theta_ll_rad;        // 左腿摆角
  float state_theta_ll_dot_rad_s;  // 左腿摆角速度
  float state_theta_lr_rad;        // 右腿摆角
  float state_theta_lr_dot_rad_s;  // 右腿摆角速度
  float state_theta_b_rad;         // 车体俯仰角
  float state_theta_b_dot_rad_s;   // 车体俯仰角速度
  float state_l_l_m;               // 左腿等效长度
  float state_l_r_m;               // 右腿等效长度

  // ── LQR 状态误差 ──
  float lqr_err_s;             // s 误差
  float lqr_err_s_dot;         // s_dot 误差
  float lqr_err_phi;           // phi 误差（已 wrap）
  float lqr_err_phi_dot;       // phi_dot 误差
  float lqr_err_theta_ll;      // theta_ll 误差
  float lqr_err_theta_ll_dot;  // theta_ll_dot 误差
  float lqr_err_theta_lr;      // theta_lr 误差
  float lqr_err_theta_lr_dot;  // theta_lr_dot 误差
  float lqr_err_theta_b;       // theta_b 误差
  float lqr_err_theta_b_dot;   // theta_b_dot 误差

  // ── 底盘状态 ──
  float chassis_leg_target_length_m;       // 斜坡平滑后的腿长目标
  float chassis_mean_leg_length_m;         // 平均腿长
  float chassis_left_leg_length_m;         // 左腿长度
  float chassis_right_leg_length_m;        // 右腿长度
  float chassis_left_l0_dot_mps;           // 左腿腿长变化率
  float chassis_right_l0_dot_mps;          // 右腿腿长变化率
  float chassis_left_l0_ddot_mps2;         // 左腿腿长加速度
  float chassis_right_l0_ddot_mps2;        // 右腿腿长加速度
  float chassis_left_l0_pid_out;           // 左腿腿长 PID 输出
  float chassis_right_l0_pid_out;          // 右腿腿长 PID 输出
  float chassis_speed_mps;                 // 车体融合速度
  float chassis_raw_wheel_speed_mps;       // 原始轮速观测
  float chassis_filtered_wheel_speed_mps;  // 低通滤波后轮速
  float chassis_raw_accel_speed_mps;       // 原始加速度积分速度
  float chassis_imu_acc_x_integral_mps;    // IMU X轴加速度直接积分速度
  float chassis_left_force_n;              // 左腿竖直力
  float chassis_right_force_n;             // 右腿竖直力
  float chassis_left_support_force_n;      // 左腿支撑力
  float chassis_right_support_force_n;     // 右腿支撑力
  float chassis_left_F_bh_n;               // 左腿雅可比反力
  float chassis_right_F_bh_n;              // 右腿雅可比反力
  float chassis_left_gravity_support_n;    // 左腿重力支撑
  float chassis_right_gravity_support_n;   // 右腿重力支撑
  float chassis_left_dyn_support_n;        // 左腿动力学补偿
  float chassis_right_dyn_support_n;       // 右腿动力学补偿
  uint8_t chassis_posture_valid;           // 姿态有效
  uint8_t chassis_off_ground;              // 离地
  uint8_t chassis_standup_complete;        // 起立完成
  uint8_t chassis_standup_phase;           // 起立阶段 (0=收腿, 1=摆角收敛, 2=完成)
  uint8_t dm_enabled_latched;              // DM 电机使能锁存
  uint8_t gimbal_motors_enabled_latched;   // 云台电机使能锁存
  uint8_t position_frozen_by_timeout;      // 位置锚定原因: 0=速度低于阈值, 1=超时强冻
  uint8_t motor_reenable_chassis_trig;     // 底盘电机重使能触发（心跳恢复脉冲，单周期）
  uint8_t motor_reenable_gimbal_trig;      // 云台电机重使能触发（心跳恢复脉冲，单周期）
  uint8_t dm_lf_online;                    // 左前 DM 电机在线状态 (Device::online_status)
  uint8_t dm_lb_online;                    // 左后 DM 电机在线状态
  uint8_t dm_rf_online;                    // 右前 DM 电机在线状态
  uint8_t dm_rb_online;                    // 右后 DM 电机在线状态
  uint8_t yaw_motor_online;                // 偏航电机在线状态
  uint8_t pitch_motor_online;              // 俯仰电机在线状态
  float expected_theta_ll_rad;             // LQR 期望左腿摆角
  float expected_theta_lr_rad;             // LQR 期望右腿摆角
  float filtered_theta_ll_dot_rad_s;       // 滤波后左腿摆角速度
  float filtered_theta_lr_dot_rad_s;       // 滤波后右腿摆角速度

  // ── 四路 ToF 与硬件模式 ──
  uint8_t tof_runtime_enabled;  // 0 for the current no-ToF baseline build
  uint8_t tof_requested_mode;   // 0=auto-jump(front), 1=stair-descend(down)
  uint8_t tof_active_mode;      // 实际已切换的硬件模式
  uint8_t tof_mode_ready;       // 两个当前传感器都 Begin 成功
  uint8_t tof_init_error_mask;  // bit0=left, bit1=right
  uint32_t tof_switch_count;
  uint32_t tof_poll_request_count;    // TIM13 产生的 100 Hz 请求数
  uint32_t tof_poll_process_count;    // 主循环实际处理次数
  uint32_t tof_poll_coalesced_count;  // 主循环来不及处理而被合并的请求数
  uint32_t tof_poll_last_us;          // 最近一次左右两颗 Poll 总耗时
  uint32_t tof_poll_max_us;           // 启动以来最大总耗时

  float active_tof_pair_raw_mean_mm;  // active pair: (left raw + right raw) / 2 (auto_jump or stair_descend)
  float left_front_tof_low_pass_mm;   // first-order LPF of new valid samples
  float right_front_tof_low_pass_mm;
  float left_front_tof_moving_average_mm;  // 5-point mean of LPF output
  float right_front_tof_moving_average_mm;

  uint8_t left_front_tof_driver_status;
  uint8_t left_front_tof_range_status;
  uint8_t left_front_tof_data_valid;
  uint8_t left_front_tof_ranging;
  uint16_t left_front_tof_model_id;
  uint16_t left_front_tof_distance_mm;
  uint32_t left_front_tof_sample_count;
  uint32_t left_front_tof_last_sample_tick_ms;
  uint32_t left_front_tof_poll_count;
  uint32_t left_front_tof_i2c_error_count;

  uint8_t right_front_tof_driver_status;
  uint8_t right_front_tof_range_status;
  uint8_t right_front_tof_data_valid;
  uint8_t right_front_tof_ranging;
  uint16_t right_front_tof_model_id;
  uint16_t right_front_tof_distance_mm;
  uint32_t right_front_tof_sample_count;
  uint32_t right_front_tof_last_sample_tick_ms;
  uint32_t right_front_tof_poll_count;
  uint32_t right_front_tof_i2c_error_count;

  // 下方 TOF 精简调试字段（仅保留关键数据，完整字段集同上方的 front TOF）
  uint8_t left_down_tof_driver_status;
  uint8_t left_down_tof_range_status;
  uint8_t left_down_tof_data_valid;
  uint8_t left_down_tof_ranging;
  uint16_t left_down_tof_distance_mm;
  uint32_t left_down_tof_sample_count;
  uint32_t left_down_tof_i2c_error_count;

  uint8_t right_down_tof_driver_status;
  uint8_t right_down_tof_range_status;
  uint8_t right_down_tof_data_valid;
  uint8_t right_down_tof_ranging;
  uint16_t right_down_tof_distance_mm;
  uint32_t right_down_tof_sample_count;
  uint32_t right_down_tof_i2c_error_count;

  // ── 发射机构（双摩擦变体）──
  uint8_t shoot_enabled;            // 发射使能（云台处于 Combat）
  uint8_t shoot_fric_ready;         // 摩擦轮达速标志（转速达到目标±阈值）
  uint8_t shoot_single_shot_mode;   // 单发模式（打符：仅上升沿触发一发）
  uint8_t shoot_manual_fire;        // 手动开火触发（拨轮超阈值 或 左键按下）
  float fric_speed_target_rpm;      // 摩擦轮目标转速 [rpm]（运行时可调）
  float fric_left_rpm;              // 左摩擦轮实际转速 [rpm]
  float fric_right_rpm;             // 右摩擦轮实际转速 [rpm]
  float dial_encoder_raw;           // 拨盘编码器原始值
  float shoot_dial_current;         // 拨盘电机输出电流
  uint32_t shot_count;              // 打弹成功计数
  uint8_t shoot_mode;               // Shoot2Fric 模式 (0=kStop,1=kFullAuto,2=kSingleShot)
  uint8_t shoot_single_complete;    // 单发完成标志（拨盘已走完一颗弹）
  uint8_t shoot_fire_flag;          // 开火标志（手动触发 或 NUC允许，热量抑制前）
  uint8_t shoot_effective_fire;     // 有效开火（fire_flag && 热量未超限）
  float shoot_loader_pos_error;     // 拨盘位置误差（目标 - 当前）[编码器单位]
  float shoot_loader_pos_target;    // 拨盘位置环目标 [编码器单位]
  float shoot_loader_pos_feedback;  // 拨盘位置环反馈 [编码器单位]（传给控制器的 encoder）
  float shoot_loader_pos_pid_out;   // 拨盘位置环 PID 输出（串级给速度环）
  float shoot_loader_spd_target;    // 拨盘速度环目标 [rpm]
  float shoot_loader_spd_feedback;  // 拨盘速度环反馈 [rpm]
  float shoot_loader_spd_pid_out;   // 拨盘速度环 PID 输出（最终给电机）
  // ── 发射机构（三摩擦变体 hero）──
  float booster_raw_pos_rad;        // DM 拨盘当前位置 (hero)
  float booster_target_rad;         // DM 拨盘目标角度 (hero)
  float fw_raw_rpm_1;               // 摩擦轮1 RPM (hero)
  float fw_raw_rpm_2;               // 摩擦轮2 RPM (hero)
  float fw_raw_rpm_3;               // 摩擦轮3 RPM (hero)
  uint8_t shoot_hero_state;         // Hero ShootController 状态机 (0=kStop,1=kInit,2=kReady,3=kShooting,4=kCooling)
  uint8_t shoot_hero_fire_trigger;  // Hero 发射触发标志
  uint8_t shoot_hero_enter;         // Hero 进入射击模式
  int32_t shoot_hero_heat_delta;    // Hero 热量余量（heat_limit - current_heat）
  int32_t hero_remaining_ammo;      // Hero 剩余弹量（本地跟踪）
  float hero_displacement_bias;     // Hero 动态位移偏置 [m]

  // ── 本地热量闭环 ──
  float shoot_local_heat;         // 本地估算枪口热量
  uint16_t shoot_heat_limit;      // 当前热量上限
  uint16_t shoot_cooling_rate;    // 当前冷却速率 [单位/秒]
  uint8_t shoot_heat_suppressed;  // 热量超限抑制中

  // ── 裁判系统 ──
  uint8_t referee_online;          // 裁判系统是否在线（收到过有效包）
  uint8_t referee_robot_id;        // 裁判系统上报的机器人 ID
  float referee_bullet_speed_mps;  // 最近一发弹丸初速度 [m/s]
  uint16_t referee_barrel_heat;    // 裁判系统上报的枪口热量（可对比 shoot_local_heat）
  uint8_t referee_power_chassis;   // 裁判系统电源管理：底盘供电状态
  uint8_t referee_power_gimbal;    // 裁判系统电源管理：云台供电状态

  // ── 超级电容 ──
  uint8_t supercap_enable_dcdc;           // 电容开启标志
  uint8_t supercap_error_code;            // 错误码
  float supercap_chassis_power;           // 底盘功率 [W]
  uint16_t supercap_chassis_power_limit;  // 底盘功率上限
  uint8_t supercap_cap_energy;            // 电容能量百分比

  // ── 云台 IMU 欧拉角（Frame C: 0x112）──
  float gimbal_euler_yaw_rad;    // 云台 IMU 偏航角
  float gimbal_euler_pitch_rad;  // 云台 IMU 俯仰角

  // ── 自瞄通信 TX（发给 NUC）──
  uint8_t aimbot_tx_mode;      // 发送的模式 (0=Normal,1=AutoAimNoMove,2=AutoAimWithMove)
  uint8_t aimbot_tx_robot_id;  // 发送的机器人 ID

  // ── 自瞄通信 RX（NUC 反馈）──
  uint8_t aimbot_rx_state;           // NUC 反馈的自瞄状态
  uint8_t aimbot_rx_target;          // NUC 反馈的目标信息
  uint8_t aimbot_rx_nuc_start_flag;  // NUC 启动标志
  float aimbot_rx_yaw_rad;           // NUC 下发的偏航目标
  float aimbot_rx_pitch_rad;         // NUC 下发的俯仰目标

  // ── AI Policy 网络观测输入 (27维，均为训练缩放后的值) ──
  // base_ang_vel * 0.25 [rad/s]
  float policy_obs_gyro_x;  // 陀螺仪 X (roll轴)
  float policy_obs_gyro_y;  // 陀螺仪 Y (pitch轴)
  float policy_obs_gyro_z;  // 陀螺仪 Z (yaw轴)
  // projected_gravity [1]
  float policy_obs_gravity_x;  // 重力投影 X
  float policy_obs_gravity_y;  // 重力投影 Y
  float policy_obs_gravity_z;  // 重力投影 Z
  // command
  float policy_obs_cmd_vx;      // 纵向速度指令 * 2.0 [m/s]
  float policy_obs_cmd_yaw;     // 偏航角速度指令 * 0.25 [rad/s]
  float policy_obs_cmd_height;  // 高度指令 * 5.0 [m]
  // leg angle [rad]
  float policy_obs_theta_ll;  // 左腿摆角 theta0_L
  float policy_obs_theta_lr;  // 右腿摆角 theta0_R
  // leg angle dot * 0.05 [rad/s]
  float policy_obs_theta_dot_ll;  // 左腿摆角速度
  float policy_obs_theta_dot_lr;  // 右腿摆角速度
  // leg length * 5.0 [m]
  float policy_obs_l_l;  // 左腿等效长度 L0_L
  float policy_obs_l_r;  // 右腿等效长度 L0_R
  // leg length dot * 0.25 [m/s]
  float policy_obs_l_dot_l;  // 左腿腿长变化率
  float policy_obs_l_dot_r;  // 右腿腿长变化率
  // wheel pos [rad] (左轮取负)
  float policy_obs_wheel_pos_l;  // -q_l_wheel
  float policy_obs_wheel_pos_r;  //  q_r_wheel
  // wheel vel * 0.05 [rad/s] (左轮取负)
  float policy_obs_wheel_vel_l;  // -dq_l_wheel * 0.05
  float policy_obs_wheel_vel_r;  //  dq_r_wheel * 0.05
  // last action (上一帧 VMC action)
  float policy_obs_prev_a_theta_l;  // 上帧 a_theta_L
  float policy_obs_prev_a_l0_l;     // 上帧 a_L0_L
  float policy_obs_prev_a_wheel_l;  // 上帧 a_wheel_L
  float policy_obs_prev_a_theta_r;  // 上帧 a_theta_R
  float policy_obs_prev_a_l0_r;     // 上帧 a_L0_R
  float policy_obs_prev_a_wheel_r;  // 上帧 a_wheel_R

  // ── AI Policy 网络动作输出 (6维 VMC action，原始无量纲值) ──
  float policy_act_theta_l;  // a_theta_L, clamp ±3.0
  float policy_act_l0_l;     // a_L0_L,   clamp ±3.0
  float policy_act_wheel_l;  // a_wheel_L, clamp ±1.326...
  float policy_act_theta_r;  // a_theta_R, clamp ±3.0
  float policy_act_l0_r;     // a_L0_R,   clamp ±3.0
  float policy_act_wheel_r;  // a_wheel_R, clamp ±1.326...

  // ── AI Policy 网络动作输出 (物理单位) ──
  float policy_act_theta_l_rad;    // a_theta_L * 0.5 [rad]
  float policy_act_theta_r_rad;    // a_theta_R * 0.5 [rad]
  float policy_act_l0_l_m;         // clamp(a_L0_L * 0.1 + 0.17, 0.122, 0.301) [m]
  float policy_act_l0_r_m;         // clamp(a_L0_R * 0.1 + 0.17, 0.122, 0.301) [m]
  float policy_act_wheel_l_rad_s;  // a_wheel_L * 52.0 [rad/s]
  float policy_act_wheel_r_rad_s;  // a_wheel_R * 52.0 [rad/s]

  // ── AI Policy 推理状态 ──
  uint32_t policy_infer_us;    // 最近一次推理耗时 [us]
  uint32_t policy_step_count;  // 推理步数累计
  uint8_t policy_ok;           // 最近一次推理成功标志
};
static_assert(sizeof(DebugSnapshot) <= 1024, "DebugSnapshot must fit in 1024 bytes for efficient DMA");

extern DebugSnapshot wl_debug;

/**
 * @brief 填充 DebugSnapshot
 * @param tick_ms                当前系统 tick
 * @param input                  本周期输入快照
 * @param chassis_output         底盘 FSM 输出
 * @param gimbal_output          云台 FSM 输出
 * @param chassis_control_output 底盘控制器输出
 * @param gimbal_control_output  云台控制器输出
 */
void UpdateDebugSnapshot(uint32_t tick_ms, const wheel_legged::control_loop::InputSnapshot &input,
                         const chassis::Fsm::Output &chassis_output, const gimbal::Fsm::Output &gimbal_output,
                         const chassis::Chassis::UpdateOutput &chassis_control_output,
                         const gimbal::Gimbal::UpdateOutput &gimbal_control_output,
                         const chassis::StairTaskCoordinator::Output &stair_task_output,
                         const chassis::StairClimbSequence::Output &stair_sequence_output);
