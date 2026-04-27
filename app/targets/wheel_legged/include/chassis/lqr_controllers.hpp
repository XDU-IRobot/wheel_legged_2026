#pragma once

#include <array>
#include <vector>

#include <librm.hpp>

namespace chassis::wbr {

struct CurrentState {
  rm::f32 s{0.0f};
  rm::f32 s_dot{0.0f};
  rm::f32 phi{0.0f};
  rm::f32 phi_dot{0.0f};
  rm::f32 theta_ll{0.0f};
  rm::f32 theta_ll_dot{0.0f};
  rm::f32 theta_lr{0.0f};
  rm::f32 theta_lr_dot{0.0f};
  rm::f32 theta_b{0.0f};
  rm::f32 theta_b_dot{0.0f};
  rm::f32 l_l{0.0f};
  rm::f32 l_r{0.0f};
};

struct ExpectedState {
  rm::f32 s{0.0f};
  rm::f32 s_dot{0.0f};
  rm::f32 phi{0.0f};
  rm::f32 phi_dot{0.0f};
  rm::f32 theta_ll{0.0f};
  rm::f32 theta_ll_dot{0.0f};
  rm::f32 theta_lr{0.0f};
  rm::f32 theta_lr_dot{0.0f};
  rm::f32 theta_b{0.0f};
  rm::f32 theta_b_dot{0.0f};
};

struct MotorTorque {
  rm::f32 t_wl{0.0f};
  rm::f32 t_wr{0.0f};
  rm::f32 t_bl{0.0f};
  rm::f32 t_br{0.0f};
};

class WbrController {
 public:
  WbrController() { k_coeffs_.assign(40, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}); }

  void SetLqrCoefficients(const std::vector<std::array<rm::f32, 6>> &coeff_matrix) {
    if (coeff_matrix.size() == 40) {
      k_coeffs_ = coeff_matrix;
    }
  }

  [[nodiscard]] MotorTorque ComputeControl(const CurrentState &current, const ExpectedState &expected) const {
    static constexpr rm::f32 kPi = 3.14159265358979323846f;

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

  void ComputeKMatrix(const rm::f32 l_l, const rm::f32 l_r, rm::f32 k_matrix[4][10]) const {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 10; ++j) {
        const int index = i * 10 + j;
        k_matrix[i][j] = EvaluatePolynomial(k_coeffs_[index], l_l, l_r);
      }
    }
  }

  std::vector<std::array<rm::f32, 6>> k_coeffs_{};
};

}  // namespace chassis::wbr
