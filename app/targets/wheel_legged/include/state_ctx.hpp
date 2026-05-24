#pragma once

#include <cstdint>

#include "chassis/fsm.hpp"
#include "params.hpp"
#include "librm/modules/pid.hpp"

/**
 * @file  targets/wheel_legged/include/state_ctx.hpp
 * @brief 底盘跨周期控制状态、偏航跟随、定点锁定与速率斜坡
 */

namespace wheel_legged::control_loop {

/**
 * @brief 偏航跟随对齐模式
 */
enum class YawFollowAlignMode : uint8_t {
  kForward = 0,   ///< 前进/后退方向对齐
  kSidePositive,  ///< 右侧方向对齐
  kSideNegative,  ///< 左侧方向对齐
};

/**
 * @brief 偏航跟随目标选择结果
 */
struct YawFollowTargetSelection {
  float target_rad{0.0f};  ///< 选择的偏航目标角 [rad]
  float drive_sign{1.0f};  ///< 驱动方向符号（+1 或 -1）
};

using SdotRampParams = wheel_legged::params::active::control_loop::SdotRampParams;

/**
 * @brief 底盘控制环跨周期状态
 * @note  封装所有需要跨控制周期保持的中间状态（滤波值、PID 状态、锁定状态等），
 *        避免在 ControlLoop() 中散落大量 static 变量。
 */
struct ChassisStateContext {
  // ── 速率滤波 ──
  float filtered_s_dot{0.0f};    ///< 滤波后的纵向速度
  float filtered_yaw_dot{0.0f};  ///< 滤波后的偏航角速度
  float expected_s{0.0f};        ///< 期望纵向位置（定点锁定 blend 输出）

  // ── 位置保持（I 项）──
  bool integrate_position{false};  ///< 是否对期望纵向位置进行积分（速度目标归零后冻结为锚点）

  // ── 落地减速（离地→落地边沿触发）──
  bool landing_decel_active{false};        ///< 落地减速是否激活
  float landing_theta_bias{0.0f};          ///< 落地减速腿摆角偏置 [rad]
  uint32_t landing_stable_ticks{0U};       ///< 落地减速稳定计数器
  uint32_t off_ground_duration_ticks{0U};  ///< 离地持续时间计数器（防单帧误判）
  bool prev_off_ground_in_mid_leg{false};  ///< 上一周期中腿长是否离地

  // ── 偏航跟随 ──
  rm::modules::PID yaw_follow_pid{};                                      ///< 偏航跟随 PID
  bool yaw_follow_pid_initialized{false};                                 ///< 偏航跟随 PID 是否已初始化
  chassis::Fsm::State last_chassis_mode{chassis::Fsm::State::kDisabled};  ///< 上一周期底盘模式

  // ── 云台启动归中 ──
  bool gimbal_startup_align_complete{false};       ///< 归中是否完成
  bool gimbal_startup_align_was_active{false};     ///< 上一周期是否处于归中
  uint32_t gimbal_startup_align_stable_ticks{0U};  ///< 归中判稳计数器
  float gimbal_startup_align_target_rad{0.0f};     ///< 归中目标偏航角

  // ── 偏航跟随目标 ──
  YawFollowAlignMode yaw_follow_align_mode{YawFollowAlignMode::kForward};  ///< 当前对齐模式
  YawFollowTargetSelection yaw_follow_target{};                            ///< 当前选定的偏航目标
  bool yaw_follow_target_initialized{false};                               ///< 偏航目标是否已初始化
  bool yaw_follow_drive_ready{false};                ///< 偏航是否已就绪（允许纵向驱动）
  uint32_t yaw_follow_drive_ready_stable_ticks{0U};  ///< 偏航就绪判稳计数器
  bool chassis_has_been_driven{false};  ///< 使能后是否曾被摇杆驱动过（首次驱动前不启用定点锁定）
  bool spin_exit_recovery{false};       ///< 小陀螺退出恢复中，使用快速偏航斜坡
  bool flip_180_in_progress{false};     ///< R 键云台 180° 旋转中，暂时抑制偏航跟随
  uint32_t flip_180_ticks{0U};         ///< flip_180 抑制计数器

  /**
   * @brief 在底盘模式切换时重置相关状态
   * @param current_s     当前纵向位置
   * @param current_s_dot 当前纵向速度
   */
  void ResetOnModeChange(float current_s, float current_s_dot);
};

/**
 * @brief 带加速/制动斜坡限速的数值跟踪
 * @param target     目标值
 * @param value      当前值（读写，逐步逼近 target）
 * @param ramp_params 斜坡参数（accel_step / brake_step）
 * @note  方向变化或减速时使用 brake_step，加速时使用 accel_step。
 */
void RampValueToTarget(float target, float &value, const SdotRampParams &ramp_params);

/**
 * @brief 偏航角速度斜坡跟踪
 * @param target_yaw_dot   目标偏航角速度
 * @param filtered_yaw_dot 当前滤波值（读写）
 * @param ramp_step        每周期步长
 */
void RampYawDotToTarget(float target_yaw_dot, float &filtered_yaw_dot, float ramp_step);

/**
 * @brief 选择离当前偏航电机角最近的目标角（双候选：fixed 与 fixed+pi）
 * @param yaw_motor_rad     当前偏航电机角
 * @param target_offset_rad  目标偏移量（由对齐模式决定）
 * @return 选定的目标角与驱动方向符号
 */
YawFollowTargetSelection SelectNearestYawTarget(float yaw_motor_rad, float target_offset_rad);

/**
 * @brief 选择最近的车头方向目标角（offset=0）
 * @param yaw_motor_rad 当前偏航电机角
 * @return 最近的车头方向角
 */
float SelectNearestYawCenterTarget(float yaw_motor_rad);

/**
 * @brief 根据对齐模式返回目标偏航角偏移
 * @param mode 对齐模式
 * @return 偏移角 [rad]
 */
float YawFollowTargetOffset(YawFollowAlignMode mode);

/**
 * @brief 根据对齐模式修正驱动方向符号
 * @param mode              对齐模式
 * @param target_drive_sign 原始目标驱动方向
 * @return 修正后的驱动方向符号
 */
float YawFollowDriveSign(YawFollowAlignMode mode, float target_drive_sign);

/**
 * @brief 判断偏航电机是否已到达启动归中目标
 * @param yaw_target_rad      目标偏航角
 * @param yaw_motor_rad       当前偏航电机编码器角
 * @param yaw_motor_vel_rad_s 当前偏航电机角速度
 * @return true 表示位置与速度均在阈值内
 */
bool IsYawAtStartupTarget(float yaw_target_rad, float yaw_motor_rad, float yaw_motor_vel_rad_s);

/**
 * @brief 判断偏航是否已就绪（允许底盘纵向驱动）
 * @param yaw_target_rad      目标偏航角
 * @param yaw_motor_rad       当前偏航电机编码器角
 * @param yaw_motor_vel_rad_s 当前偏航电机角速度
 * @return true 表示偏航跟踪到位，可以开始纵向移动
 */
bool IsYawFollowDriveReady(float yaw_target_rad, float yaw_motor_rad, float yaw_motor_vel_rad_s);

/**
 * @brief 根据底盘状态机模式选择合适的纵向速度斜坡参数
 * @param mode 底盘模式
 * @return 对应该模式的 SdotRampParams
 */
SdotRampParams ResolveSdotRampParams(chassis::Fsm::State mode);
SdotRampParams ResolveSdotRampParams(chassis::Fsm::State mode, bool mid_leg_g);

}  // namespace wheel_legged::control_loop
