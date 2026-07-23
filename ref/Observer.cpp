//
// Created by RedogJrome on 2026/2/27.
// (Refactored to Momentum-based Disturbance Observer)
//

#include "Observer.hpp"
#include <cstring>
#include "Observer.hpp"
#include <cstring>
#include "Definition.h"
#include "iLQR.hpp"

#include "debugc.hpp"

namespace Observer {

#define NUM_STATES 10
#define NUM_CONTROLS 4
#define NUM_DOF 5

static const float SP[NUM_STATES] = {-1, -1, -1, -1, -1, -1, -1, -1, 1, 1};
static const float UP[NUM_CONTROLS] = {-1, -1, 1, 1};
static const float Rw = 0.0645f;
static const float Rl = 0.203f;
static const float dt = 0.002f;
// =========================================================================
// 二阶 MDO (LESO) 状态变量
// =========================================================================
static float p_hat[NUM_DOF];  // 预估动量状态
static float d_hat[NUM_DOF];  // 【核心】：内部积分扰动状态，彻底消灭静差
void Init() {
  memset(p_hat, 0, sizeof(p_hat));
  memset(d_hat, 0, sizeof(d_hat));
}
__attribute__((section(".itcm_text"))) void Step(const float* M_5x5, const float* G_5x5, const float* B_tau_5x4,
                                                 const float* B_tau_pinv_4x5, const float* X_measured,
                                                 const float* U_actual_last, float* D_out, float* X_hat_out) {
  float X_matlab[NUM_STATES];
  float U_matlab[NUM_CONTROLS];
  for (int i = 0; i < NUM_STATES; i++) X_matlab[i] = X_measured[i] * SP[i];
  for (int i = 0; i < NUM_CONTROLS; i++) U_matlab[i] = U_actual_last[i] * UP[i];
  for (int i = 0; i < NUM_STATES; i++) X_hat_out[i] = X_measured[i];

  float q[NUM_DOF], dq[NUM_DOF];
  q[0] = (X_matlab[0] - X_matlab[2] * Rl) / Rw;
  q[1] = (X_matlab[0] + X_matlab[2] * Rl) / Rw;
  q[2] = X_matlab[4];
  q[3] = X_matlab[6];
  q[4] = X_matlab[8];

  dq[0] = (X_matlab[1] - X_matlab[3] * Rl) / Rw;
  dq[1] = (X_matlab[1] + X_matlab[3] * Rl) / Rw;
  dq[2] = X_matlab[5];
  dq[3] = X_matlab[7];
  dq[4] = X_matlab[9];

  // =========================================================
  // 本地复制矩阵，然后直接砍掉“轮子 <- Pitch”耦合
  // DOF顺序: 0左轮, 1右轮, 2左腿, 3右腿, 4机体Pitch
  // =========================================================
  float M_eff[NUM_DOF * NUM_DOF];
  float G_eff[NUM_DOF * NUM_DOF];
  float Bpinv_eff[NUM_CONTROLS * NUM_DOF];

  memcpy(M_eff, M_5x5, sizeof(M_eff));
  memcpy(G_eff, G_5x5, sizeof(G_eff));
  memcpy(Bpinv_eff, B_tau_pinv_4x5, sizeof(Bpinv_eff));

  float p[NUM_DOF] = {0.0f};
  float tau_model[NUM_DOF] = {0.0f};

  // const float W_O[NUM_DOF] = {0.0f, 0.0f, 0.0f, 0.0f, 60.0f};
  const float W_O[NUM_DOF] = {5.0f, 5.0f, 5.0f, 5.0f, 10.0f};
  // const float W_O[NUM_DOF] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f};

  for (int i = 0; i < NUM_DOF; i++) {
    for (int j = 0; j < NUM_DOF; j++) p[i] += M_eff[i * NUM_DOF + j] * dq[j];
    for (int j = 0; j < NUM_CONTROLS; j++) tau_model[i] += B_tau_5x4[i * NUM_CONTROLS + j] * U_matlab[j];
    for (int j = 0; j < NUM_DOF; j++) tau_model[i] -= G_eff[i * NUM_DOF + j] * q[j];

    float e = p[i] - p_hat[i];

    float beta1 = 2.0f * W_O[i];
    float beta2 = W_O[i] * W_O[i];

    d_hat[i] += beta2 * e * dt;
    p_hat[i] += (tau_model[i] + d_hat[i] + beta1 * e) * dt;
  }

  float D_out_matlab[NUM_CONTROLS] = {0.0f};
  for (int i = 0; i < NUM_CONTROLS; i++) {
    for (int j = 0; j < NUM_DOF; j++) {
      D_out_matlab[i] += Bpinv_eff[i * NUM_DOF + j] * d_hat[j];
    }
  }

  static float last_D_out[NUM_CONTROLS] = {0.0f};
  const float LPF_ALPHA = 1.0f;

  for (int i = 0; i < NUM_CONTROLS; i++) {
    float raw_d = D_out_matlab[i] * UP[i];
    D_out[i] = LPF_ALPHA * raw_d + (1.0f - LPF_ALPHA) * last_D_out[i];
    last_D_out[i] = D_out[i];
    if (i <= 1)
      D_out[i] = MyLimit(D_out[i], LIMIT_WHEEL_L_MIN, LIMIT_WHEEL_L_MAX);
    else
      D_out[i] = MyLimit(D_out[i], -LIMIT_LEG, LIMIT_LEG);
  }
}
}  // namespace Observer
