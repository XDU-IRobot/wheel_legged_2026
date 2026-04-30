#pragma once

#include <array>

#include <librm.hpp>
#include "../wheel_legged_params.hpp"

/**
 * @file  targets/wheel_legged/include/chassis/lqr_controllers.hpp
 * @brief 轮腿底盘线性二次调节器与状态向量定义
 */

namespace chassis::wbr {

/**
 * @brief 当前状态向量
 */
struct CurrentState {
  rm::f32 s{0.0f};             ///< 车体纵向位移
  rm::f32 s_dot{0.0f};         ///< 车体纵向速度
  rm::f32 phi{0.0f};           ///< 车体偏航角
  rm::f32 phi_dot{0.0f};       ///< 车体偏航角速度
  rm::f32 theta_ll{0.0f};      ///< 左腿摆角
  rm::f32 theta_ll_dot{0.0f};  ///< 左腿摆角速度
  rm::f32 theta_lr{0.0f};      ///< 右腿摆角
  rm::f32 theta_lr_dot{0.0f};  ///< 右腿摆角速度
  rm::f32 theta_b{0.0f};       ///< 机体俯仰角
  rm::f32 theta_b_dot{0.0f};   ///< 机体俯仰角速度
  rm::f32 l_l{0.0f};           ///< 左腿长
  rm::f32 l_r{0.0f};           ///< 右腿长
};

/**
 * @brief 期望状态向量
 */
struct ExpectedState {
  rm::f32 s{0.0f};             ///< 期望纵向位移
  rm::f32 s_dot{wheel_legged::params::active::state_estimator::kDefaultExpectedSdotMps};  ///< 期望纵向速度
  rm::f32 phi{0.0f};           ///< 期望偏航角
  rm::f32 phi_dot{0.0f};       ///< 期望偏航角速度
  rm::f32 theta_ll{0.0f};      ///< 期望左腿摆角
  rm::f32 theta_ll_dot{0.0f};  ///< 期望左腿摆角速度
  rm::f32 theta_lr{0.0f};      ///< 期望右腿摆角
  rm::f32 theta_lr_dot{0.0f};  ///< 期望右腿摆角速度
  rm::f32 theta_b{0.0f};       ///< 期望机体俯仰角
  rm::f32 theta_b_dot{0.0f};   ///< 期望机体俯仰角速度
};

/**
 * @brief 调节器输出力矩
 */
struct MotorTorque {
  rm::f32 t_wl{0.0f};  ///< 左轮力矩
  rm::f32 t_wr{0.0f};  ///< 右轮力矩
  rm::f32 t_bl{0.0f};  ///< 左腿髋关节等效力矩
  rm::f32 t_br{0.0f};  ///< 右腿髋关节等效力矩
};

/**
 * @brief 基于腿长多项式拟合增益的线性二次调节器
 */
class WbrController {
 public:
  /**
   * @brief 设置调节器拟合系数矩阵
   * @param coeff_matrix 40x6 多项式系数矩阵
   */
  void SetLqrCoefficients(const std::array<std::array<rm::f32, 6>, 40> &coeff_matrix) { k_coeffs_ = coeff_matrix; }

  /**
   * @brief 单步控制解算
   */
  [[nodiscard]] MotorTorque ComputeControl(const CurrentState &current, const ExpectedState &expected) const {
    static constexpr rm::f32 kPi = wheel_legged::params::active::kPi;

    rm::f32 k_matrix[4][10]{};
    ComputeKMatrix(current.l_l, current.l_r, k_matrix);

    rm::f32 x_err[10]{};
    x_err[0] = current.s - expected.s;
    x_err[1] = current.s_dot - expected.s_dot;
    x_err[2] = rm::modules::Wrap(current.phi - expected.phi, -kPi, kPi);
    x_err[3] = current.phi_dot - expected.phi_dot;
    x_err[4] = current.theta_ll - expected.theta_ll;
    x_err[5] = current.theta_ll_dot - expected.theta_ll_dot;
    x_err[6] = current.theta_lr - expected.theta_lr;
    x_err[7] = current.theta_lr_dot - expected.theta_lr_dot;
    x_err[8] = current.theta_b - expected.theta_b;
    x_err[9] = current.theta_b_dot - expected.theta_b_dot;

    rm::f32 u_vec[4]{};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 10; ++j) {
        u_vec[i] -= k_matrix[i][j] * x_err[j];
      }
    }

    MotorTorque torque{};
    torque.t_wl = u_vec[0];
    torque.t_wr = u_vec[1];
    torque.t_bl = u_vec[2];
    torque.t_br = u_vec[3];
    return torque;
  }

 private:
  [[nodiscard]] static rm::f32 EvaluatePolynomial(const std::array<rm::f32, 6> &p, const rm::f32 l_l,
                                                  const rm::f32 l_r) {
    return p[0] + p[1] * l_l + p[2] * l_r + p[3] * l_l * l_l + p[4] * l_l * l_r + p[5] * l_r * l_r;
  }

  /**
   * @brief 由腿长计算当前 4x10 增益矩阵
   */
  void ComputeKMatrix(const rm::f32 l_l, const rm::f32 l_r, rm::f32 k_matrix[4][10]) const {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 10; ++j) {
        const int index = i * 10 + j;
        k_matrix[i][j] = EvaluatePolynomial(k_coeffs_[index], l_l, l_r);
      }
    }
  }

  std::array<std::array<rm::f32, 6>, 40> k_coeffs_{};  ///< 40 组多项式系数
};

}  // namespace chassis::wbr
