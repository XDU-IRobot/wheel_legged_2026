#pragma once

#include "state.hpp"
#include "fsm.hpp"
#include "lqr.hpp"
#include "leso.hpp"
#include "../params.hpp"
#include "../fsm_common.hpp"

/**
 * @file  targets/wheel_legged/include/chassis/chassis.hpp
 * @brief 轮腿底盘控制主类
 */

namespace chassis {

/**
 * @brief 底盘控制器
 */
class Chassis {
 public:
  enum class LesoDisableReason : uint8_t {
    kActive = 0,
    kObserverDisabled,
    kOutputDisabled,
    kStandupIncomplete,
    kInvalidPosture,
    kUnsupportedMode,
    kMidLegDip,
    kOffGround,
    kInvalidInput,
  };

  /**
   * @brief 单次控制更新输入
   */
  struct UpdateInput {
    ChassisStateEstimatorInput estimator_input{};       ///< 传感器反馈
    wbr::ExpectedState expected{};                      ///< 期望状态
    Fsm::State fsm_mode{Fsm::State::kDisabled};         ///< 当前状态机模式
    bool enable_output{false};                          ///< 是否允许输出电机命令
    bool run_chassis_update{false};                     ///< 是否执行底盘控制计算
    bool spin_enable{false};                            ///< 是否开启小陀螺
    bool keyboard_active{false};                        ///< 图传键鼠是否在线
    wheel_legged::ChassisMotionTarget motion_target{};  ///< 本周期解析后的唯一运动目标
    bool yaw_centering_complete{false};                 ///< 云台恢复归中是否完成
    bool position_hold_active{false};                   ///< 位置锚定激活中，触发 LQR 误差缩放
    bool stair_sequence_controls_motion{false};         ///< 台阶动作序列是否已经接管运动控制
    rm::f32 displacement_bias{
        wheel_legged::params::active::control_loop::kExpectedDisplacementBiasMLowLeg};  ///< 低腿长期望位移偏置 [m]
  };

  /**
   * @brief 单次控制更新输出
   */
  struct UpdateOutput {
    rm::f32 lqr_scaled_err_s{0.0f};
    rm::f32 lqr_scaled_err_s_dot{0.0f};
    bool position_hold_active{false};
    std::array<rm::f32, 5> leso_generalized_disturbance{};
    std::array<rm::f32, 4> leso_virtual_disturbance{};
    std::array<rm::f32, 4> leso_previous_virtual_command{};
    std::array<rm::f32, 5> leso_momentum_error{};
    std::array<rm::f32, 4> leso_applied_compensation{};
    bool leso_enabled{false};
    bool leso_initialized{false};
    LesoDisableReason leso_disable_reason{LesoDisableReason::kObserverDisabled};

    rm::f32 lf_tau{0.0f};  ///< 左前关节电机力矩
    rm::f32 lb_tau{0.0f};  ///< 左后关节电机力矩
    rm::f32 rf_tau{0.0f};  ///< 右前关节电机力矩
    rm::f32 rb_tau{0.0f};  ///< 右后关节电机力矩
    rm::f32 lw_tau{0.0f};  ///< 左轮电机力矩
    rm::f32 rw_tau{0.0f};  ///< 右轮电机力矩

    rm::f32 left_l0_pid_out{0.0f};           ///< 左腿腿长 PID 输出
    rm::f32 right_l0_pid_out{0.0f};          ///< 右腿腿长 PID 输出
    rm::f32 left_force_n{0.0f};              ///< 左腿竖直力
    rm::f32 right_force_n{0.0f};             ///< 右腿竖直力
    rm::f32 left_support_force_n{0.0f};      ///< 左腿支撑力估计
    rm::f32 right_support_force_n{0.0f};     ///< 右腿支撑力估计
    rm::f32 left_F_bh_n{0.0f};               ///< 左腿雅可比反力（竖直分量）
    rm::f32 right_F_bh_n{0.0f};              ///< 右腿雅可比反力（竖直分量）
    rm::f32 left_gravity_support_n{0.0f};    ///< 左腿重力支撑分量
    rm::f32 right_gravity_support_n{0.0f};   ///< 右腿重力支撑分量
    rm::f32 left_dyn_support_n{0.0f};        ///< 左腿动力学补偿分量
    rm::f32 right_dyn_support_n{0.0f};       ///< 右腿动力学补偿分量
    rm::f32 mean_leg_length_m{0.0f};         ///< 平均腿长
    rm::f32 leg_target_length_m{0.0f};       ///< 斜坡平滑后的腿长目标
    rm::f32 left_l0_dot_mps{0.0f};           ///< 左腿腿长变化率
    rm::f32 right_l0_dot_mps{0.0f};          ///< 右腿腿长变化率
    rm::f32 left_l0_ddot_mps2{0.0f};         ///< 左腿腿长加速度
    rm::f32 right_l0_ddot_mps2{0.0f};        ///< 右腿腿长加速度
    rm::f32 filtered_theta_ll_dot{0.0f};     ///< 滤波后左腿摆角速度
    rm::f32 filtered_theta_lr_dot{0.0f};     ///< 滤波后右腿摆角速度
    rm::f32 speed_mps{0.0f};                 ///< 融合车速
    rm::f32 wheel_speed_mps{0.0f};           ///< 轮系解算车速
    rm::f32 filtered_wheel_speed_mps{0.0f};  ///< 低通滤波后轮速
    rm::f32 raw_wheel_speed_mps{0.0f};       ///< 原始轮速观测
    rm::f32 raw_accel_speed_mps{0.0f};       ///< 原始加速度积分速度
    rm::f32 imu_acc_x_integral_mps{0.0f};    ///< IMU X轴加速度直接积分速度
    rm::f32 current_speed_mps{0.0f};         ///< 速度融合当前估计
    bool off_ground_in_mid_high_leg{false};
    bool off_ground_gravity_off{false};          ///< 离地 > 0.1s 重力补偿已关闭
    bool posture_valid{true};                    ///< 底盘姿态是否在安全范围内
    bool pitch_roll_valid_theta_invalid{false};  ///< pitch/roll 正常但腿摆角异常
    bool standup_complete{false};                ///< 起立完成：双腿 theta 均小于阈值后置 true
    uint8_t standup_phase{0};                    ///< 起立阶段：0=摆腿, 1=收腿, 2=摆腿收敛, 3=完成
    float standup_theta_target{0.0f};            ///< 起立摆角 PID 目标当前值 [rad]
    bool mid_leg_dip_active{false};              ///< 中腿长下压激活中
    rm::f32 stair_t_bl_cmd{0.0f};                ///< 上台阶左腿摆角控制输出（PID 或 LQR）
    rm::f32 stair_t_br_cmd{0.0f};                ///< 上台阶右腿摆角控制输出（PID 或 LQR）

    wbr::CurrentState current_state{};  ///< 当前状态向量
  };

  /**
   * @brief 初始化控制器参数与估计器
   */
  void Init();

  /**
   * @brief 执行一次底盘控制更新
   */
  void Update(const UpdateInput &input);

  /**
   * @brief 安全停机，输出力矩清零
   */
  void SafeStop();

  /**
   * @brief 获取最近一次控制输出
   */
  [[nodiscard]] const UpdateOutput &GetOutput() const { return output_; }

 private:
  /**
   * @brief 计算六个执行器最终力矩
   */
  void ComputeActuatorTorque(const UpdateInput &input, const ChassisStateEstimatorOutput &state_output);

  /**
   * @brief 由关节实测力矩反解支撑力
   */
  void CalSupportForce();

  struct TunableParams {
    rm::f32 leg_target_length_m{wheel_legged::params::active::chassis_fsm::kMidLegLengthM};  ///< 当前腿长目标
  };

  ChassisStateEstimator state_estimator_{};
  wbr::WbrController lqr_controller_{};
  wbr::MomentumLeso leso_{};
  std::array<rm::f32, 4> previous_final_virtual_command_{};
  std::array<rm::f32, 4> applied_leso_compensation_{};
  bool prev_spin_active_{false};  ///< 上一周期是否处于自旋模式，用于切换 LQR 增益
  wbr::LegKinematics left_leg_{wheel_legged::params::active::chassis::kLegL1M,
                               wheel_legged::params::active::chassis::kLegL2M};
  wbr::LegKinematics right_leg_{wheel_legged::params::active::chassis::kLegL1M,
                                wheel_legged::params::active::chassis::kLegL2M};
  wbr::MotorTorque base_torque_{};
  TunableParams params_{};

  rm::f32 left_force_{0.0f};
  rm::f32 right_force_{0.0f};
  rm::f32 l_spring_torque_{0.0f};
  rm::f32 r_spring_torque_{0.0f};
  rm::f32 imu_roll_{0.0f};

  rm::f32 lf_real_torque_{0.0f};
  rm::f32 lb_real_torque_{0.0f};
  rm::f32 rf_real_torque_{0.0f};
  rm::f32 rb_real_torque_{0.0f};
  rm::f32 imu_acc_x_mps2_{0.0f};
  rm::f32 imu_acc_x_integral_mps_{0.0f};
  rm::f32 imu_acc_z_mps2_{0.0f};

  rm::f32 left_support_force_est_n_{0.0f};
  rm::f32 right_support_force_est_n_{0.0f};
  rm::f32 left_l0_dot_prev_{0.0f};
  rm::f32 right_l0_dot_prev_{0.0f};
  rm::modules::LowPassFilterConstDt<rm::f32> left_l0_ddot_filter_{};
  rm::modules::LowPassFilterConstDt<rm::f32> right_l0_ddot_filter_{};

  rm::f32 filtered_l0_dot_left_{0.0f};
  rm::f32 filtered_l0_dot_right_{0.0f};
  rm::modules::LowPassFilterConstDt<rm::f32> left_l0_dot_filter_{};
  rm::modules::LowPassFilterConstDt<rm::f32> right_l0_dot_filter_{};

  rm::modules::LowPassFilterConstDt<rm::f32> left_theta_dot_filter_{};
  rm::modules::LowPassFilterConstDt<rm::f32> right_theta_dot_filter_{};
  rm::f32 filtered_theta_ll_dot_{0.0f};
  rm::f32 filtered_theta_lr_dot_{0.0f};
  bool theta_dot_filter_initialized_{false};

  rm::f32 smoothed_leg_target_length_m_{wheel_legged::params::active::chassis_fsm::kLowLegLengthM};
  rm::f32 last_ramp_target_m_{
      wheel_legged::params::active::chassis_fsm::kLowLegLengthM};  ///< 上一次斜坡目标值(检测变化)
  rm::f32 ramp_step_per_tick_m_{0.0f};                             ///< 当前斜坡每周期步长

  bool prev_enable_output_{false};
  bool l0_dot_filter_initialized_{false};
  bool standup_complete_{false};             ///< 起立完成
  uint8_t standup_phase_{0};                 ///< 起立阶段：0=摆腿, 1=收腿, 2=摆腿收敛, 3=完成
  float standup_theta_target_{0.0f};         ///< 起立摆角 PID 目标斜坡当前值 [rad]
  uint8_t theta_recovery_phase_{0};          ///< 仅theta异常恢复阶段：0=收腿到0.14f, 1=摆腿
  bool theta_recovery_active_{false};        ///< theta恢复激活中（退出时跳Phase 0直接进Phase 1）
  bool standup_from_recovery_latch_{false};  ///< theta恢复完成后直接进起立Phase 1
  bool prev_fsm_was_recovery_{false};        ///< 上一周期是否在恢复状态

  uint16_t standup_phase_stable_ticks_{0};   ///< 起立阶段切换所需的连续满足周期数
  uint16_t off_ground_duration_ticks_{0};    ///< 离地持续时间（用于衰减气弹簧补偿）
  bool force_low_leg_{false};                ///< 离地后腿长过短时强制低腿长
  uint16_t force_low_leg_ticks_{0};          ///< 强制低腿长已持续时间
  bool mid_leg_dip_active_{false};           ///< 中腿长下压激活中
  bool mid_leg_dip_armed_{false};            ///< 中腿长下压待命（腿先低于阈值才可触发）
  uint16_t mid_leg_dip_ticks_{0};            ///< 中腿长下压已持续时间
  bool leg_was_high_{false};                 ///< 离地前腿长曾高于 0.3m（防止低腿长误触发）
  bool off_ground_kd_active_{false};         ///< 着地边沿后 Kd 增大锁存
  uint16_t kd_active_ticks_{0};              ///< Kd 增大已持续时间
  bool was_off_ground_{false};               ///< 上一周期离地状态（用于检测着地边沿）
  float spring_compensation_scale_{1.0f};    ///< 气弹簧补偿缩放（着地后衰减，腿长恢复后复原）
  bool off_ground_200ms_reached_{false};     ///< 离地已持续 200ms（落地后触发强制低腿长）

  rm::modules::PID left_l0_pid_{};
  rm::modules::PID right_l0_pid_{};
  rm::modules::PID left_l0_pid_jump_one_{};
  rm::modules::PID right_l0_pid_jump_one_{};
  rm::modules::PID left_l0_pid_jump_two_{};
  rm::modules::PID right_l0_pid_jump_two_{};
  rm::modules::PID left_l0_pid_jump_three_{};
  rm::modules::PID right_l0_pid_jump_three_{};
  rm::modules::PID left_l0_pid_dip_{};
  rm::modules::PID right_l0_pid_dip_{};
  rm::modules::PID roll_pid_{};
  rm::modules::PID left_leg_turn_pid_{};
  rm::modules::PID right_leg_turn_pid_{};
  rm::modules::PID left_leg_angle_pid_standup_{};
  rm::modules::PID right_leg_angle_pid_standup_{};
  rm::modules::PID left_leg_turn_pid_manual_{};
  rm::modules::PID right_leg_turn_pid_manual_{};
  rm::modules::PID left_stair_theta_pid_{};               ///< 台阶序列左腿摆角 PID
  rm::modules::PID right_stair_theta_pid_{};              ///< 台阶序列右腿摆角 PID
  rm::modules::PID left_leg_angle_pid_jump_retract2_{};   ///< 跳跃收腿第二阶段左腿摆角 PID
  rm::modules::PID right_leg_angle_pid_jump_retract2_{};  ///< 跳跃收腿第二阶段右腿摆角 PID

  UpdateOutput output_{};
};

}  // namespace chassis
