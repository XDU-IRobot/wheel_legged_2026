#ifndef BALANCE_CHASSIS_CHASSIS_HPP
#define BALANCE_CHASSIS_CHASSIS_HPP

#include "fsm/fsm.hpp"
#include "state/chassis_state.hpp"
#include "wbr/leg_kinematics.hpp"
#include "wbr/wbr_controller.hpp"
#include <librm.hpp>
#include <memory>

/**
 * @file  chassis/chassis.hpp
 * @brief 底盘控制与执行器封装接口
 */

using namespace hal;
using namespace rm;
using namespace rm::modules;
using namespace rm::device;

class Chassis {
public:
  struct UpdateInput {
    ChassisStateEstimatorInput estimator_input{};
    wbr::ExpectedState expected{};
    Fsm::State fsm_mode{Fsm::State::kNoForce};
    float target_leg_length_m{0.18f};
  };

  struct UpdateOutput {
    f32 lf_tau{0.0f};
    f32 lb_tau{0.0f};
    f32 rf_tau{0.0f};
    f32 rb_tau{0.0f};
    f32 lw_tau{0.0f};
    f32 rw_tau{0.0f};
    f32 wheel_speed_mps{0.0f};
    f32 raw_wheel_speed_mps{0.0f};
    f32 raw_accel_speed_mps{0.0f};
    f32 current_speed_mps{0.0f};
    f32 left_Fn{0.0f};
    f32 right_Fn{0.0f};
    wbr::CurrentState current_state{};
  };

  /**
   * @brief 构造函数
   * @param l1 腿部连杆参数 l1（m）
   * @param l2 腿部连杆参数 l2（m）
   */
  Chassis(f32 l1 = 0.215f, f32 l2 = 0.254f);
  ~Chassis() = default;

  /** @brief 初始化底盘对象 */
  void Init();

  /**
   * @brief 底盘控制单步更新
   * @note  包含状态估计、WBR 计算与六电机最终力矩合成。
   */
  void Update(const UpdateInput &input);
  void ComputeRightLegExcitation(float dt_s);

  /**
   * @brief 获取最近一次 Update 输出
   */
  [[nodiscard]] const UpdateOutput &GetOutput() const { return output_; }

  /**
   * @brief 由实测力矩反解支撑力
   */
  void Cal_Fn();

  /** @brief 获取当前融合状态 */
  [[nodiscard]] const wbr::CurrentState &GetCurrentState() const {
    return current_state_;
  }

private:
  struct TunableParams {
    f32 leg_target_length_m{0.18f};
    f32 support_force_bias{100.0f};
  };

  void ComputeUltiTau(Fsm::State mode);

  // 模型计算得到的基础力矩
  wbr::MotorTorque base_torque;

  TunableParams params_{};

  // 腿部支撑相关中间量
  f32 left_force_ = 0.f, right_force_ = 0.f;
  f32 l_spring_torque_{0.f}, r_spring_torque_{0.f};
  f32 first_wheel_speed_ = 0.0f;
  f32 imu_roll_ = 0.0f;
  f32 lb_tau_ = 0.0f;
  f32 lf_tau_ = 0.0f;
  f32 rb_tau_ = 0.0f;
  f32 rf_tau_ = 0.0f;
  f32 left_wheel_tau_ = 0.0f;
  f32 right_wheel_tau_ = 0.0f;
  UpdateOutput output_{};

  // 关节实测力矩（用于支撑力估计）
  f32 lf_real_torque_ = 0.f, lb_real_torque_ = 0.f;
  f32 rf_real_torque_ = 0.f, rb_real_torque_ = 0.f;

  // 双腿支撑力估计（内部缓存，最终写入 output_）
  f32 left_Fn_ = 0.f, right_Fn_ = 0.f;
  f32 left_l0_dot_prev_ = 0.f, right_l0_dot_prev_ = 0.f;
  f32 sysid_time_s_ = 0.f;
  f32 last_theta_lr_dot_ = 0.f;
  f32 filtered_ddtheta_lr_ = 0.f;

  // PID 控制器
  std::unique_ptr<rm::modules::PID> left_l0_pid{};
  std::unique_ptr<rm::modules::PID> right_l0_pid{};
  std::unique_ptr<rm::modules::PID> left_l0_pid_jump_two{};
  std::unique_ptr<rm::modules::PID> right_l0_pid_jump_two{};
  std::unique_ptr<rm::modules::PID> left_l0_pid_jump_three{};
  std::unique_ptr<rm::modules::PID> right_l0_pid_jump_three{};
  std::unique_ptr<rm::modules::PID> roll_pid{};
  std::unique_ptr<rm::modules::PID> left_leg_turn_pid{};
  std::unique_ptr<rm::modules::PID> right_leg_turn_pid{};

  // 解算器与控制器
  wbr::WbrController wbr_controller_;
  wbr::LegKinematics left_leg_;
  wbr::LegKinematics right_leg_;

  // 当前机体融合状态
  wbr::CurrentState current_state_{};
  ChassisStateEstimator state_estimator_{};
};

#endif // BALANCE_CHASSIS_CHASSIS_HPP
