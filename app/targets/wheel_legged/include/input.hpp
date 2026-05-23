#pragma once

#include <cstdint>

#include "librm/device/remote/dr16.hpp"

#include "chassis/chassis.hpp"
#include "chassis/state.hpp"
#include "chassis/fsm.hpp"
#include "fsm_common.hpp"
#include "gimbal/fsm.hpp"

struct SharedResources;

namespace chassis_runtime {
class Actuators;
}  // namespace chassis_runtime

/**
 * @file  targets/wheel_legged/include/input.hpp
 * @brief 硬件输入采集、DR16/图传语义折叠与 FSM 输入构建
 */

namespace wheel_legged::control_loop {

/**
 * @brief DR16 遥控器原始通道值（去语义化）
 */
struct Dr16RawInput {
  bool online{false};                                                                     ///< DR16 是否在线
  rm::device::DR16::SwitchPosition switch_l{rm::device::DR16::SwitchPosition::kUnknown};  ///< 左拨杆
  rm::device::DR16::SwitchPosition switch_r{rm::device::DR16::SwitchPosition::kUnknown};  ///< 右拨杆
  int16_t right_y{0};       ///< 右摇杆 Y（前进/后退）
  int16_t right_x{0};       ///< 右摇杆 X（平移）
  int16_t left_x{0};        ///< 左摇杆 X（偏航速率）
  int16_t left_y{0};        ///< 左摇杆 Y（俯仰速率）
  int16_t dial{0};          ///< 拨轮值
  int16_t mouse_x{0};       ///< 鼠标 X 增量
  int16_t mouse_y{0};       ///< 鼠标 Y 增量
  bool mouse_left{false};   ///< 鼠标左键
  bool mouse_right{false};  ///< 鼠标右键
  uint16_t keyboard{0};     ///< 键盘按键位掩码
};

/**
 * @brief 图传键鼠输入（通过云台 C 板 CAN 桥转发）
 */
struct TcRemoteInput {
  bool valid{false};           ///< 键鼠数据是否有效
  int16_t mouse_x{0};          ///< 鼠标 X 增量
  int16_t mouse_y{0};          ///< 鼠标 Y 增量
  bool left_button{false};     ///< 鼠标左键
  bool right_button{false};    ///< 鼠标右键
  uint16_t keyboard_value{0};  ///< 键盘按键位掩码
  bool tc_from_dr16{false};    ///< 数据是否来自 DR16 回退（非真实 VT03）
};

/**
 * @brief 归一化后的驾驶输入方向
 */
struct DriveInputNorm {
  float forward{0.0f};  ///< 前进分量 [-1, 1]
  float side{0.0f};     ///< 平移分量 [-1, 1]
};

/**
 * @brief 单周期控制输入快照
 * @note  每个控制周期由 UpdateRawFeedbackAndInputSnapshot() 完整填充一次，
 *        后续所有模块从此结构体消费数据，不直接访问硬件。
 */
struct InputSnapshot {
  bool input_valid{false};                                ///< 是否有可信的输入源
  Dr16RawInput dr16{};                                    ///< DR16 原始值
  TcRemoteInput tc_remote{};                              ///< 图传键鼠数据（CAN 桥）
  chassis::ChassisStateEstimatorInput estimator_input{};  ///< 底盘传感器反馈（估计器输入）
  wheel_legged::ModeRequest mode_request{};               ///< 整车语义请求
  bool auto_jump_enabled{false};                          ///< 自动跳跃当前是否已开启
  bool ui_refresh_key{false};                             ///< E 键按下（UI 刷新使能）
  float gimbal_imu_yaw_rad{0.0f};                         ///< 云台惯导偏航角
  float gimbal_imu_pitch_rad{0.0f};                       ///< 云台惯导俯仰角
  float gimbal_imu_gyro_z_rad_s{0.0f};                    ///< 云台惯导 Z 轴角速度（偏航轴）
  float gimbal_imu_gyro_x_rad_s{0.0f};                    ///< 云台惯导 X 轴角速度（俯仰轴）
};

/**
 * @brief DR16 语义状态（跨周期保持）
 */
struct Dr16SemanticState {
  bool wheel_action_armed{true};           ///< 拨轮动作是否已就绪（防重复触发）
  bool gimbal_target_initialized{false};   ///< 云台积分目标是否已初始化
  bool last_auto_aim{false};               ///< 上一周期是否处于自瞄模式
  wheel_legged::GimbalTarget rc_target{};  ///< RC 积分目标角
};

/**
 * @brief 图传语义状态（跨周期保持）
 */
struct TcSemanticState {
  bool mid_leg_c_armed{true};         ///< C 键是否已就绪（上升沿检测）
  bool mid_leg_g_armed{true};         ///< G 键是否已就绪（上升沿检测）
  bool mid_leg_g{false};              ///< G 键触发的中腿长（区分斜坡参数）
  bool mid_leg_hold{false};           ///< 是否保持中腿长
  bool q_domain_armed{true};          ///< Q 键是否已就绪（上升沿检测）
  uint8_t domain_state{0};            ///< Q domain cycle: 0=disabled, 1=standby, 2=enabled
  bool v_high_leg_armed{true};        ///< V 键是否已就绪（上升沿检测）
  bool b_high_leg_armed{true};        ///< B 键是否已就绪（上升沿检测）
  bool f_jump_armed{true};            ///< F 键是否已就绪（上升沿检测）
  bool high_leg_hold{false};          ///< 是否保持高腿长
  bool b_double_mode{false};          ///< B 模式：需完成两次上台阶
  uint8_t b_attempt{0};               ///< B 模式已完成上台阶次数
  bool stair_climb_done{false};       ///< 上台阶完成后锁定低腿长
  bool dr16_parallel{false};          ///< DR16 是否并行生效
  bool z_fric_dec_armed{true};        ///< Ctrl+Z 组合键是否已就绪（上升沿检测，摩擦轮减速）
  bool x_fric_inc_armed{true};        ///< Ctrl+X 组合键是否已就绪（上升沿检测，摩擦轮升速）
  float fric_speed_target_rpm{0.0f};  ///< 摩擦轮目标转速 [rpm]（运行时可调，0 表示未初始化）
  bool e_ui_refresh{false};           ///< E 键是否按下（UI 刷新控制）
  bool auto_aim_hold{false};          ///< 鼠标右键按住时自瞄模式（电平有效）
  enum class AimMode : uint8_t { kAmmo, kFuSmall, kFuBig };
  bool ctrl_f_armed{true};           ///< Ctrl+F 组合键上升沿检测
  bool ctrl_g_armed{true};           ///< Ctrl+G 组合键上升沿检测
  AimMode aim_mode{AimMode::kAmmo};  ///< 右键自瞄子模式
  bool recovery_manual_mode{false};  ///< 倒地自启手动模式（Z 键长按切换）
  bool z_recovery_armed{true};       ///< Z 键是否已就绪（长按防抖）
  float z_hold_ms{0.0f};             ///< Z 键已按住时长 [ms]
};

/**
 * @brief 将 DR16 摇杆轴值归一化到 [-1, 1]
 * @param axis        原始轴值
 * @param axis_max_abs 轴最大值绝对值
 * @return 归一化后的值
 */
float NormalizeDr16Axis(int16_t axis, int16_t axis_max_abs);

/**
 * @brief 综合 DR16 摇杆与图传键盘解析驾驶方向
 * @param dr16         DR16 原始输入
 * @param tc_remote    图传键鼠输入
 * @param dr16_parallel DR16 是否并行生效
 * @return 归一化驾驶方向（forward, side）
 * @note  图传 WASD 优先于 DR16 摇杆；并行模式下键鼠无输入时降级到 DR16
 */
DriveInputNorm ResolveDriveInput(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote, bool dr16_parallel);

/**
 * @brief 将 DR16 右拨杆映射为腿长档位
 * @param switch_r 右拨杆位置
 * @return 腿长档位
 */
wheel_legged::LegProfile ResolveLegProfile(rm::device::DR16::SwitchPosition switch_r);

/**
 * @brief 将 DR16 左拨杆映射为整车工作域
 * @param switch_l 左拨杆位置
 * @return 工作域请求
 */
wheel_legged::DomainRequest ResolveDomainRequest(rm::device::DR16::SwitchPosition switch_l);

/**
 * @brief 将原始 DR16/图传输入折叠为整车语义请求
 * @param dr16           DR16 原始输入
 * @param tc_remote      图传键鼠输入
 * @param semantic_state DR16 跨周期语义状态（读写）
 * @param tc_state       图传跨周期语义状态（读写）
 * @param input          输入快照（写入 mode_request 等字段）
 * @note  本函数是遥控器原始值到整车语义的唯一切换点，
 *        后续模块不应直接解析 DR16 的 switch/dial/axis 原始量。
 */
void ResolveInputSemantics(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote, Dr16SemanticState &semantic_state,
                           TcSemanticState &tc_state, InputSnapshot &input);

/**
 * @brief 采集硬件反馈并刷新输入快照
 * @param g              SharedResources 全局资源
 * @param actuators      执行器（用于 FillEstimatorInput）
 * @param input          输入快照（全量填充）
 * @param semantic_state DR16 跨周期语义状态
 * @param tc_state       图传跨周期语义状态
 * @note  每个控制周期调用一次，在 FSM 与控制器之前执行。
 */
void UpdateRawFeedbackAndInputSnapshot(SharedResources &g, chassis_runtime::Actuators &actuators, InputSnapshot &input,
                                       Dr16SemanticState &semantic_state, TcSemanticState &tc_state);

/**
 * @brief 从 InputSnapshot 构建底盘 FSM 输入
 * @param input           输入快照
 * @param tick_ms         当前系统 tick
 * @param chassis_output  上周期底盘控制输出（回灌腿长、摆角）
 * @return 底盘 FSM 输入
 */
chassis::Fsm::Input BuildChassisFsmInput(const InputSnapshot &input, uint32_t tick_ms,
                                         const chassis::Chassis::UpdateOutput &chassis_output);

/**
 * @brief 从 InputSnapshot 和底盘 FSM 输出构建云台 FSM 输入
 * @param input                 输入快照
 * @param chassis_output        本周期底盘 FSM 输出
 * @param startup_align_complete 云台启动归中是否完成
 * @return 云台 FSM 输入
 */
gimbal::Fsm::Input BuildGimbalFsmInput(const InputSnapshot &input, const chassis::Fsm::Output &chassis_output,
                                       bool startup_align_complete);

}  // namespace wheel_legged::control_loop
