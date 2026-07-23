//
// Created by RedogJrome on 2026/2/27.
// (Modified for Momentum-based Disturbance Observer)
//

#ifndef INC_2026_BANLANCED_INFANTRY_OBSERVER_HPP
#define INC_2026_BANLANCED_INFANTRY_OBSERVER_HPP

namespace Observer {
// 系统初始化 (在进入主循环前调用一次，清空积分器)
void Init();
// 观测器核心步进函数 (每个控制周期调用)
// M_5x5          : [输入] 5x5 质量/惯量矩阵
// G_5x5          : [输入] 5x5 重力/刚度耦合矩阵
// B_tau_5x4      : [输入] 5x4 输入映射矩阵
// B_tau_pinv_4x5 : [输入] 4x5 输入映射的伪逆矩阵
// X_measured     : [输入] 10维，传感器真实读数 (物理系)
// U_actual_last  : [输入] 4维，上一周期实际发给电机的总力矩 (物理系)
// D_out          : [输出] 4维，MDO 扰动观测结果 (物理系，极致平滑)
// X_hat_out      : [输出] 10维，平滑状态 (由于MDO不观测状态，此处直接透传输入)
void Step(const float* M_5x5, const float* G_5x5, const float* B_tau_5x4, const float* B_tau_pinv_4x5,
          const float* X_measured, const float* U_actual_last, float* D_out, float* X_hat_out);

}  // namespace Observer

#endif  // INC_2026_BANLANCED_INFANTRY_OBSERVER_HPP