/**
 * @file  targets/wheel_legged/include/utils/kalman.hpp
 * @brief 卡尔曼滤波器
 */

#ifndef KALMAN_HPP
#define KALMAN_HPP

#ifdef __cplusplus

#if defined(__arm__) || defined(__aarch64__) || defined(STM32)
#include "arm_math.h"
#else
// 非嵌入式构建仅提供 CMSIS-DSP 矩阵 API 占位定义，便于语法检查，不执行真实矩阵运算。
#include <cstdint>
typedef struct {
  uint16_t numRows;
  uint16_t numCols;
  float *pData;
} arm_matrix_instance_f32;

inline void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nCols, float *pData) {
  S->numRows = nRows;
  S->numCols = nCols;
  S->pData = pData;
}
inline int arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst) {
  return 0;
}
inline int arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst) {
  return 0;
}
inline int arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst) {
  return 0;
}
inline int arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst) { return 0; }
inline int arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst) { return 0; }
#endif

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <functional>

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

#include "librm.hpp"

using namespace rm;

class KalmanFilter {
 public:
  KalmanFilter(u8 xhatSize, u8 uSize, u8 zSize);
  ~KalmanFilter();

  // 禁用拷贝构造和赋值
  KalmanFilter(const KalmanFilter&) = delete;
  KalmanFilter& operator=(const KalmanFilter&) = delete;

  void Measure();
  void UpdateXhatMinus();
  void UpdatePMinus();
  void SetK();
  void UpdateXhat();
  void UpdateP();
  f32* Update();

  f32* FilteredValue;
  f32* MeasuredVector;
  f32* ControlVector;

  u8 xhatSize;
  u8 uSize;
  u8 zSize;

  u8 UseAutoAdjustment;
  u8 MeasurementValidNum;

  u8* MeasurementMap;          // 观测量与状态量映射关系
  f32* MeasurementDegree;      // 观测量对应 H 矩阵元素值
  f32* MatR_DiagonalElements;  // 观测噪声协方差对角元素
  f32* StateMinVariance;       // 最小状态方差约束
  u8* temp;

  bool SkipEq1 = false;
  bool SkipEq2 = false;
  bool SkipEq3 = false;
  bool SkipEq4 = false;
  bool SkipEq5 = false;

  mat xhat, xhatminus, u, z, P, Pminus, F, FT, B, H, HT, Q, R, K;
  mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

  i8 MatStatus;

  // 用户扩展回调（C++11）
  std::function<void(KalmanFilter*)> User_Func0_f;
  std::function<void(KalmanFilter*)> User_Func1_f;
  std::function<void(KalmanFilter*)> User_Func2_f;
  std::function<void(KalmanFilter*)> User_Func3_f;
  std::function<void(KalmanFilter*)> User_Func4_f;
  std::function<void(KalmanFilter*)> User_Func5_f;
  std::function<void(KalmanFilter*)> User_Func6_f;

  f32 *xhat_data, *xhatminus_data;
  f32* u_data;
  f32* z_data;
  f32 *P_data, *Pminus_data;
  f32 *F_data, *FT_data;
  f32* B_data;
  f32 *H_data, *HT_data;
  f32* Q_data;
  f32* R_data;
  f32* K_data;
  f32 *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;

 private:
  void HKRAdjustment();
};

#endif

#endif
