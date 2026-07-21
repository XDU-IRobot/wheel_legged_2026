#pragma once

#include <cstdint>

#include "librm/device/remote/dr16.hpp"

#include "chassis/chassis.hpp"
#include "chassis/state.hpp"
#include "chassis/fsm.hpp"
#include "fsm_common.hpp"
#include "gimbal/fsm.hpp"
#include "tof_mode.hpp"

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
  int16_t mouse_z{0};          ///< 鼠标滚轮
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
 * @brief 驾驶输入斜坡状态（跨周期保持）
 */
struct DriveInputRampState {
  float forward{0.0f};  ///< 前进斜坡当前值
  float side{0.0f};     ///< 平移斜坡当前值
};

/**
 * @brief DR16 拨杆→模式解析结果
 */
struct Dr16ModeResult {
  wheel_legged::DomainRequest domain{wheel_legged::DomainRequest::kDisabled};
  wheel_legged::LegProfile leg{wheel_legged::LegProfile::kLow};
  wheel_legged::CombatProfile combat{wheel_legged::CombatProfile::kNormal};
  wheel_legged::GimbalTestProfile gimbal_test{wheel_legged::GimbalTestProfile::kNormal};
  bool spin_hold{false};
  float spin_dir{0.0f};
  bool jump_trigger{false};
};

/**
 * @brief TC 键鼠→模式解析结果
 */
struct TcModeResult {
  wheel_legged::DomainRequest domain{wheel_legged::DomainRequest::kDisabled};
  wheel_legged::LegProfile leg{wheel_legged::LegProfile::kLow};
  wheel_legged::CombatProfile combat{wheel_legged::CombatProfile::kNormal};
  bool spin_hold{false};
  bool jump_trigger{false};
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
  bool auto_jump_triggered{false};                        ///< 本周期自动跳跃触发
  bool auto_jump_enabled{false};                          ///< 已按 Z，正在等待或执行一次自动跳跃
  bool auto_jump_both_close{false};                       ///< 调试：both_close 条件
  bool auto_jump_tof_armed_debug{false};                  ///< 调试：auto_jump_tof_armed 条件
  bool auto_jump_both_active{false};                      ///< 调试：both_active 条件（含200ms消抖）
  bool auto_jump_trigger_ready{false};                    ///< 调试：both_close && tof_armed && both_active
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
  bool mid_leg_c_armed{true};  ///< C 键是否已就绪（上升沿检测）
  bool g_aim_armed{true};      ///< G 键是否已就绪（上升沿检测，aim_mode 切换）
  // 自动小跳字段已移除
  bool mid_leg_hold{false};         ///< 是否保持中腿长
  bool mid_leg_f{false};            ///< F 键触发的 mid_leg_f 中腿长模式（慢速斜坡参数）
  bool q_domain_armed{true};        ///< Q 键是否已就绪（上升沿检测）
  bool ctrl_q_standby_armed{true};  ///< Ctrl+Q 组合键是否已就绪（上升沿检测，进入 standby）
  uint8_t domain_state{0};          ///< Q domain: 0=disabled, 1=standby(Ctrl+Q), 2=enabled
  bool v_high_leg_armed{true};      ///< V 键是否已就绪（上升沿检测）
  bool b_high_leg_armed{true};      ///< B 键是否已就绪（上升沿检测）
  bool f_slow_armed{true};          ///< F 键是否已就绪（上升沿检测，mid_leg_f 模式切换）
  bool e_ui_refresh{false};         ///< E 键是否按下（UI 刷新控制）
  bool auto_aim_hold{false};        ///< 鼠标右键按住时自瞄模式（电平有效）
  enum class AimMode : uint8_t { kAmmo, kFuSmall, kFuBig };
  AimMode aim_mode{AimMode::kAmmo};  ///< 右键自瞄子模式
  bool r_flip_armed{true};           ///< R 键 180° 翻转上升沿检测
  wheel_legged::TofMode requested_tof_mode{wheel_legged::TofMode::kAutoJump};
  bool x_tof_mode_armed{true};            ///< X 键 ToF 模式切换上升沿
  bool stair_descend_in_progress{false};  ///< 预留：下台阶动作执行中时禁止 X 手动退出
  bool stair_descend_completed{false};    ///< 预留：下台阶逻辑完成后置 true，自动切回前向 ToF
  bool auto_jump_enabled{false};          ///< Z 键启动的一次性自动跳跃正在等待/执行
  bool auto_jump_in_progress{false};      ///< 已由自动测距触发并进入底盘跳跃状态
  bool z_auto_jump_armed{true};           ///< Z 键上升沿检测
  bool auto_jump_tof_armed{true};         ///< 自动跳跃 TOF 边沿检测
  uint32_t both_active_start_ms{0};       ///< both_active 条件首次满足的时刻
  bool dial_jump_armed{true};             ///< DR16 拨轮跳跃边沿检测
  bool mouse_z_jump_armed{true};          ///< 鼠标滚轮跳跃边沿检测
  wheel_legged::DomainRequest prev_domain{wheel_legged::DomainRequest::kDisabled};  ///< 使能边沿检测用
  enum class PendingAction : uint8_t { kNone, kC, kV, kB };
  PendingAction pending_action{PendingAction::kNone};   ///< yaw 对齐完成后待执行的动作
  wheel_legged::StairTaskRequest deferred_stair_request{///< 延迟到下一周期的台阶请求
                                                        wheel_legged::StairTaskRequest::kNone};
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
 * @param ramp         键盘速度斜坡状态（跨周期保持）
 * @return 归一化驾驶方向（forward, side）
 * @note  图传 WASD 优先于 DR16 摇杆；键鼠无输入时若数据来自 DR16 回退则降级到摇杆
 */
DriveInputNorm ResolveDriveInput(const Dr16RawInput &dr16, const TcRemoteInput &tc_remote, DriveInputRampState &ramp);

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
                                       Dr16SemanticState &semantic_state, TcSemanticState &tc_state,
                                       uint32_t now_ms);

/**
 * @brief 从 InputSnapshot 构建底盘 FSM 输入
 * @param input               输入快照
 * @param tick_ms             当前系统 tick
 * @param chassis_output      上周期底盘控制输出（回灌腿长、摆角）
 * @param fall_start_ms       倒地开始时刻 [ms]（跨周期保持）
 * @param was_posture_invalid 上一周期姿态是否异常（跨周期保持）
 * @return 底盘 FSM 输入
 */
chassis::Fsm::Input BuildChassisFsmInput(const InputSnapshot &input, uint32_t tick_ms,
                                         const chassis::Chassis::UpdateOutput &chassis_output, uint32_t &fall_start_ms,
                                         bool &was_posture_invalid);

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
