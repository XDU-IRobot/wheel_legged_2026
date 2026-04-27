#pragma once

#include <algorithm>
#include <cmath>

#include <librm.hpp>

namespace chassis::wbr {

class LegKinematics {
 public:
  LegKinematics(rm::f32 l1 = 0.215f, rm::f32 l2 = 0.254f) : l1_(l1), l2_(l2) {}

  void SetPhi1(const rm::f32 phi1) { phi1_ = phi1; }
  void SetPhi4(const rm::f32 phi4) { phi4_ = phi4; }
  void SetWPhi1(const rm::f32 w_phi1) { w_phi1_ = w_phi1; }
  void SetWPhi4(const rm::f32 w_phi4) { w_phi4_ = w_phi4; }

  [[nodiscard]] rm::f32 l0() const { return l0_; }
  [[nodiscard]] rm::f32 l0_dot() const { return l0_dot_; }
  [[nodiscard]] rm::f32 phi0() const { return phi0_; }
  [[nodiscard]] rm::f32 beta() const { return beta_; }
  [[nodiscard]] rm::f32 beta_dot() const { return beta_dot_; }

  [[nodiscard]] rm::f32 jacobi_00() const { return jacobi_00_; }
  [[nodiscard]] rm::f32 jacobi_01() const { return jacobi_01_; }
  [[nodiscard]] rm::f32 jacobi_10() const { return jacobi_10_; }
  [[nodiscard]] rm::f32 jacobi_11() const { return jacobi_11_; }

  void Update(const rm::f32 dt_s = 0.002f) {
    static constexpr rm::f32 kPi = 3.14159265358979323846f;
    static constexpr rm::f32 kMinSin = 1e-5f;
    static constexpr rm::f32 kMinLen = 1e-5f;

    const rm::f32 dt = (dt_s > 1e-5f) ? dt_s : 0.002f;

    const rm::f32 prev_l0 = l0_;

    xb_ = l1_ * std::cos(phi1_);
    yb_ = l1_ * std::sin(phi1_);
    xd_ = l1_ * std::cos(phi4_);
    yd_ = l1_ * std::sin(phi4_);
    l_bd_ = std::hypot(xd_ - xb_, yd_ - yb_);

    A0_ = 2.0f * l2_ * (xd_ - xb_);
    B0_ = 2.0f * l2_ * (yd_ - yb_);
    C0_ = l_bd_ * l_bd_;
    const rm::f32 root_arg = std::max(0.0f, A0_ * A0_ + B0_ * B0_ - C0_ * C0_);
    const rm::f32 root = std::sqrt(root_arg);
    phi2_ = 2.0f * std::atan2(B0_ + root, A0_ + C0_);

    xc_ = xb_ + l2_ * std::cos(phi2_);
    yc_ = yb_ + l2_ * std::sin(phi2_);
    l0_ = std::hypot(xc_, yc_);
    phi0_ = std::atan2(yc_, xc_);

    beta_ = phi2_ + kPi - phi1_;
    rm::f32 beta_delta = beta_ - beta_last_;
    while (beta_delta > kPi) {
      beta_delta -= 2.0f * kPi;
    }
    while (beta_delta < -kPi) {
      beta_delta += 2.0f * kPi;
    }
    beta_dot_ = beta_delta / dt;
    beta_last_ = beta_;

    phi3_ = std::atan2(yc_ - yd_, xc_ - xd_);

    const rm::f32 sin_p12 = std::sin(phi1_ - phi2_);
    const rm::f32 sin_p34 = std::sin(phi3_ - phi4_);
    const rm::f32 sin_p23 = std::sin(phi2_ - phi3_);
    const rm::f32 sin_p32 = std::sin(phi3_ - phi2_);
    const rm::f32 sin_p03 = std::sin(phi0_ - phi3_);
    const rm::f32 cos_p03 = std::cos(phi0_ - phi3_);
    const rm::f32 sin_p02 = std::sin(phi0_ - phi2_);
    const rm::f32 cos_p02 = std::cos(phi0_ - phi2_);

    const rm::f32 sin_p2 = std::sin(phi2_);
    const rm::f32 cos_p2 = std::cos(phi2_);
    const rm::f32 sin_p3 = std::sin(phi3_);
    const rm::f32 cos_p3 = std::cos(phi3_);

    if (std::fabs(sin_p23) < kMinSin || l0_ < kMinLen || std::fabs(sin_p32) < kMinSin) {
      xc_dot_ = 0.0f;
      yc_dot_ = 0.0f;
      jacobi_00_ = 0.0f;
      jacobi_01_ = 0.0f;
      jacobi_10_ = 0.0f;
      jacobi_11_ = 0.0f;
      l0_dot_ = (l0_ - prev_l0) / dt;
      return;
    }

    xc_dot_ = l1_ * sin_p12 * sin_p3 / sin_p23 * w_phi1_ +
              l1_ * sin_p34 * sin_p2 / sin_p23 * w_phi4_;
    yc_dot_ = l1_ * sin_p12 * cos_p3 / sin_p23 * w_phi1_ -
              l1_ * sin_p34 * cos_p2 / sin_p23 * w_phi4_;

    jacobi_00_ = l1_ * sin_p03 * sin_p12 / sin_p32;
    jacobi_01_ = l1_ * cos_p03 * sin_p12 / (l0_ * sin_p32);
    jacobi_10_ = l1_ * sin_p02 * sin_p34 / sin_p32;
    jacobi_11_ = l1_ * cos_p02 * sin_p34 / (l0_ * sin_p32);

    l0_dot_ = (l0_ - prev_l0) / dt;
  }

 private:
  rm::f32 phi1_{0.0f};
  rm::f32 phi4_{0.0f};
  rm::f32 w_phi1_{0.0f};
  rm::f32 w_phi4_{0.0f};

  rm::f32 phi2_{0.0f};
  rm::f32 phi3_{0.0f};
  rm::f32 phi0_{0.0f};
  rm::f32 l0_{0.0f};
  rm::f32 l0_dot_{0.0f};

  rm::f32 beta_{0.0f};
  rm::f32 beta_last_{0.0f};
  rm::f32 beta_dot_{0.0f};

  rm::f32 xb_{0.0f};
  rm::f32 yb_{0.0f};
  rm::f32 xd_{0.0f};
  rm::f32 yd_{0.0f};
  rm::f32 xc_{0.0f};
  rm::f32 yc_{0.0f};
  rm::f32 xc_dot_{0.0f};
  rm::f32 yc_dot_{0.0f};
  rm::f32 l_bd_{0.0f};

  rm::f32 A0_{0.0f};
  rm::f32 B0_{0.0f};
  rm::f32 C0_{0.0f};

  rm::f32 jacobi_00_{0.0f};
  rm::f32 jacobi_01_{0.0f};
  rm::f32 jacobi_10_{0.0f};
  rm::f32 jacobi_11_{0.0f};

  rm::f32 l1_{0.215f};
  rm::f32 l2_{0.254f};
};

}  // namespace chassis::wbr
