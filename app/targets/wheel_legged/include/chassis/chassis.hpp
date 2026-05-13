#pragma once

#include "state.hpp"
#include "fsm.hpp"
#include "lqr.hpp"
#include "../params.hpp"

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
  /**
   * @brief 单次控制更新输入
   */
  struct UpdateInput {
    ChassisStateEstimatorInput estimator_input{};  ///< 传感器反馈
    wbr::ExpectedState expected{};                 ///< 期望状态
    Fsm::State fsm_mode{Fsm::State::kDisabled};    ///< 当前状态机模式
    bool enable_output{false};                     ///< 是否允许输出电机命令
    bool run_chassis_update{false};                ///< 是否执行底盘控制计算
    bool spin_enable{false};                       ///< 是否开启小陀螺
    rm::f32 target_leg_length_m{wheel_legged::params::active::chassis_fsm::kMidLegLengthM};  ///< 目标腿长
  };

  /**
   * @brief 单次控制更新输出
   */
  struct UpdateOutput {
    rm::f32 lf_tau{0.0f};  ///< 左前关节电机力矩
    rm::f32 lb_tau{0.0f};  ///< 左后关节电机力矩
    rm::f32 rf_tau{0.0f};  ///< 右前关节电机力矩
    rm::f32 rb_tau{0.0f};  ///< 右后关节电机力矩
    rm::f32 lw_tau{0.0f};  ///< 左轮电机力矩
    rm::f32 rw_tau{0.0f};  ///< 右轮电机力矩

    rm::f32 left_l0_pid_out{0.0f};        ///< 左腿腿长 PID 输出
    rm::f32 right_l0_pid_out{0.0f};       ///< 右腿腿长 PID 输出
    rm::f32 left_force_n{0.0f};           ///< 左腿竖直力
    rm::f32 right_force_n{0.0f};          ///< 右腿竖直力
    rm::f32 left_support_force_n{0.0f};   ///< 左腿支撑力估计
    rm::f32 right_support_force_n{0.0f};  ///< 右腿支撑力估计
    rm::f32 left_F_bh_n{0.0f};            ///< 左腿雅可比反力（竖直分量）
    rm::f32 right_F_bh_n{0.0f};           ///< 右腿雅可比反力（竖直分量）
    rm::f32 left_gravity_support_n{0.0f}; ///< 左腿重力支撑分量
    rm::f32 right_gravity_support_n{0.0f}; ///< 右腿重力支撑分量
    rm::f32 left_dyn_support_n{0.0f};     ///< 左腿动力学补偿分量
    rm::f32 right_dyn_support_n{0.0f};    ///< 右腿动力学补偿分量
    rm::f32 mean_leg_length_m{0.0f};      ///< 平均腿长
    rm::f32 left_l0_dot_mps{0.0f};        ///< 左腿腿长变化率
    rm::f32 right_l0_dot_mps{0.0f};       ///< 右腿腿长变化率
    rm::f32 left_l0_ddot_mps2{0.0f};      ///< 左腿腿长加速度
    rm::f32 right_l0_ddot_mps2{0.0f};     ///< 右腿腿长加速度
    rm::f32 speed_mps{0.0f};              ///< 融合车速
    rm::f32 wheel_speed_mps{0.0f};        ///< 轮系解算车速
    rm::f32 raw_wheel_speed_mps{0.0f};    ///< 原始轮速观测
    rm::f32 raw_accel_speed_mps{0.0f};    ///< 原始加速度积分速度
    rm::f32 current_speed_mps{0.0f};      ///< 速度融合当前估计
    bool off_ground_in_mid_high_leg{false};
    bool posture_valid{true};                ///< 底盘姿态是否在安全范围内
    bool standup_complete{false};            ///< 起立完成：双腿 theta 均小于阈值后置 true
    bool stair_climb_ready_for_done{false};  ///< 上台阶回摆到位，可以进入 kStairClimbDone

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
  rm::f32 imu_acc_z_mps2_{0.0f};

  rm::f32 left_support_force_est_n_{0.0f};
  rm::f32 right_support_force_est_n_{0.0f};
  rm::f32 left_l0_dot_prev_{0.0f};
  rm::f32 right_l0_dot_prev_{0.0f};
  rm::modules::LowPassFilterConstDt<rm::f32> left_l0_ddot_filter_{};
  rm::modules::LowPassFilterConstDt<rm::f32> right_l0_ddot_filter_{};

  rm::f32 smoothed_leg_target_length_m_{wheel_legged::params::active::chassis_fsm::kLowLegLengthM};

  bool prev_enable_output_{false};
  bool standup_complete_{false};
  uint8_t stair_climb_phase_{0};  ///< 上台阶子阶段：0=转腿到目标摆角, 1=收腿压低车身, 2=回摆到0
  uint16_t stair_climb_stable_ticks_{0};   ///< 当前 Phase 条件连续满足的周期数
  uint16_t off_ground_duration_ticks_{0};  ///< 离地持续时间（用于衰减气弹簧补偿）
  bool force_low_leg_{false};              ///< 离地后腿长过短时强制低腿长
  uint16_t force_low_leg_ticks_{0};        ///< 强制低腿长已持续时间
  bool leg_was_high_{false};               ///< 离地前腿长曾高于 0.3m（防止低腿长误触发）

  rm::modules::PID left_l0_pid_{};
  rm::modules::PID right_l0_pid_{};
  rm::modules::PID left_l0_pid_jump_two_{};
  rm::modules::PID right_l0_pid_jump_two_{};
  rm::modules::PID left_l0_pid_jump_three_{};
  rm::modules::PID right_l0_pid_jump_three_{};
  rm::modules::PID roll_pid_{};
  rm::modules::PID left_leg_turn_pid_{};
  rm::modules::PID right_leg_turn_pid_{};
  rm::modules::PID left_stair_climb_theta_pid_{};   ///< 上台阶左腿摆角 PID（位置环）
  rm::modules::PID right_stair_climb_theta_pid_{};  ///< 上台阶右腿摆角 PID（位置环）

  UpdateOutput output_{};
};

}  // namespace chassis
