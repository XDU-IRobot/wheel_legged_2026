#pragma once

#include <algorithm>
#include <optional>

#include "librm/device/actuator/dm_motor.hpp"
#include "librm/device/actuator/dji_motor.hpp"
#include "librm/modules/pid.hpp"
#include "librm/modules/angle.hpp"

#include "wheel_legged_params.hpp"

/**
 * @file  targets/wheel_legged/include/shoot_controller.hpp
 * @brief 发射机构控制器（拨盘 + 摩擦轮），从 temp/balance_hero_gb/shoot.cc 迁移
 */

namespace wheel_legged {

class ShootController {
  using FwMotor = rm::device::M3508;
  using BoosterMotor = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;

  enum class State : uint8_t { kStop, kInitialize, kReady, kShooting, kCooling };

  static constexpr float kPi = params::active::kPi;
  static constexpr float kZeroPoint = params::active::gimbal::kBoosterZeroPointRad;
  static constexpr float kSegment = params::active::gimbal::kSegmentAngleRad;
  static constexpr uint16_t kInitTicks = params::active::gimbal::kInitDelayTicks;
  static constexpr uint16_t kShootTicks = params::active::gimbal::kShootDelayTicks;
  static constexpr float kStallThres = params::active::gimbal::kStallThresholdRad;
  static constexpr float kStallFallback = params::active::gimbal::kStallFallbackRad;
  static constexpr float kFwReadyRpm = params::active::gimbal::kFwReadySpeedThresholdRpm;

  static constexpr uint16_t kShootDelayMin = 250;             ///< 最短发射间隔 [tick]
  static constexpr uint16_t kShootDelayMax = 800;             ///< 最长发射间隔 [tick]
  static constexpr int32_t kHeatCoolingGuard = 100;           ///< 热量余量达到此值允许再次射击
  static constexpr int32_t kLocalHeatDecrementPerShot = 120;  ///< 每次射击本地热量扣减
  static constexpr uint16_t kDefaultHeatLimit = 100;          ///< 裁判离线时的默认热量上限

 public:
  ShootController() = default;

  void Attach(FwMotor *fw1, FwMotor *fw2, FwMotor *fw3, BoosterMotor *booster) {
    fw_[0] = fw1;
    fw_[1] = fw2;
    fw_[2] = fw3;
    booster_ = booster;
  }

  void Init() {
    constexpr auto &pp = params::active::gimbal::kBoosterPositionPid;
    constexpr auto &sp = params::active::gimbal::kBoosterSpeedPid;
    booster_pos_pid_.emplace(pp.kp, pp.ki, pp.kd, pp.max_out, pp.max_iout);
    booster_speed_pid_.emplace(sp.kp, sp.ki, sp.kd, sp.max_out, sp.max_iout);
    booster_pos_pid_->SetCircular(true);
    booster_pos_pid_->SetCircularCycle(2.0f * kPi);

    constexpr auto &fp = params::active::gimbal::kFwSpeedPid;
    fw_speed_pid_[0].emplace(fp.kp, fp.ki, fp.kd, fp.max_out, fp.max_iout);
    fw_speed_pid_[1].emplace(fp.kp, fp.ki, fp.kd, fp.max_out, fp.max_iout);
    fw_speed_pid_[2].emplace(fp.kp, fp.ki, fp.kd, fp.max_out, fp.max_iout);
  }

  void SetHeat(uint16_t current_heat, uint16_t heat_limit, bool referee_online) {
    if (referee_online) {
      heat_delta_ = static_cast<int32_t>(heat_limit) - static_cast<int32_t>(current_heat);
      heat_limit_ = heat_limit;
      referee_online_ = true;
    } else {
      referee_online_ = false;
      heat_limit_ = kDefaultHeatLimit;
    }
  }

  /**
   * @brief 射击状态机 + PID，每 500Hz 控制周期调用一次
   * @param enter_shoot  true = 进入射击模式（switch_r=UP 等效条件）
   * @param fire_trigger true = 触发发射（dial > 阈值）
   */
  void Update(bool enter_shoot, bool fire_trigger) {
    booster_pos_ = booster_->pos();

    switch (state_) {
      case State::kStop:
        if (enter_shoot) {
          state_ = State::kInitialize;
          now_angle_ = kZeroPoint;
          next_angle_ = kZeroPoint;
          init_time_ = kInitTicks;
        }
        break;

      case State::kInitialize:
        if (init_time_ > 0) {
          init_time_--;
          if (init_time_ == 50) {
            booster_disable_ = true;
          }
        } else {
          // 对齐到最近 60° 段
          while (now_angle_ < booster_pos_) {
            now_angle_ += kSegment;
          }
          while (now_angle_ > booster_pos_) {
            now_angle_ -= kSegment;
          }
          if (booster_pos_ - now_angle_ > kSegment / 2.0f) {
            now_angle_ += kSegment;
            if (now_angle_ > kPi) {
              now_angle_ -= 2.0f * kPi;
            }
            next_angle_ = now_angle_ + kSegment;
            if (next_angle_ > kPi) {
              next_angle_ -= 2.0f * kPi;
            }
          } else {
            next_angle_ = now_angle_ + kSegment;
            if (next_angle_ > kPi) {
              next_angle_ -= 2.0f * kPi;
            }
            now_angle_ = booster_pos_;
          }
          // 摩擦轮转速不足时等待加速
          if (static_cast<float>(fw_[0]->rpm()) >= kFwReadyRpm) {
            booster_enable_ = true;
            state_ = State::kReady;
          } else {
            init_time_ = kInitTicks;  // 重新延时等待
          }
        }
        break;

      case State::kReady:
        if (!enter_shoot) {
          booster_disable_ = true;
          state_ = State::kStop;
        } else if (fire_trigger) {
          state_ = State::kShooting;
          shoot_time_ = std::clamp(
              static_cast<uint16_t>(static_cast<int32_t>(kShootTicks) - (static_cast<int32_t>(heat_limit_) - 100) / 2),
              kShootDelayMin, kShootDelayMax);
          now_angle_ = next_angle_;
          if (!referee_online_) {
            heat_delta_ -= kLocalHeatDecrementPerShot;
          }
        }
        break;

      case State::kShooting:
        if (shoot_time_ > 0) {
          shoot_time_--;
        } else {
          state_ = State::kCooling;
          // 堵转检测
          if (rm::modules::Wrap(now_angle_ - booster_pos_, -kPi, kPi) > kStallThres) {
            now_angle_ = booster_pos_ - kStallFallback;
          } else {
            next_angle_ = now_angle_ + kSegment;
            if (next_angle_ > kPi) {
              next_angle_ -= 2.0f * kPi;
            }
          }
        }
        break;

      case State::kCooling:
        if (!enter_shoot) {
          booster_disable_ = true;
          state_ = State::kStop;
        } else if (referee_online_ && heat_delta_ < kHeatCoolingGuard) {
        } else {
          state_ = State::kReady;
        }
        break;

      default:
        state_ = State::kStop;
        break;
    }

    // 摩擦轮速度 PID（enter_shoot 时调速，否则转速>2000时制动，<2000时发0）
    {
      constexpr float kFwTarget = params::active::gimbal::kFwTargetSpeedRpm;
      constexpr float kFwBrakeTargetRpm = -100.0f;
      constexpr float kFwBrakeThresholdRpm = 500.0f;

      for (int i = 0; i < 3; ++i) {
        if (enter_shoot) {
          fw_speed_pid_[i]->Update(kFwTarget, static_cast<float>(fw_[i]->rpm()));
          fw_[i]->SetCurrent(static_cast<std::int16_t>(fw_speed_pid_[i]->out()));
        } else if (static_cast<float>(fw_[i]->rpm()) >= kFwBrakeThresholdRpm) {
          fw_speed_pid_[i]->Update(kFwBrakeTargetRpm, static_cast<float>(fw_[i]->rpm()));
          fw_[i]->SetCurrent(static_cast<std::int16_t>(fw_speed_pid_[i]->out()));
        } else {
          fw_speed_pid_[i]->Clear();
          fw_[i]->SetCurrent(0);
        }
      }
    }

    // 拨盘 PID 计算
    booster_pos_pid_->Update(now_angle_, booster_->pos());
    booster_speed_pid_->Update(booster_pos_pid_->out(), booster_->vel());

    // 电机命令下发
    if (booster_enable_) {
      booster_->SendInstruction(rm::device::DmMotorInstructions::kClearError);
      booster_->SendInstruction(rm::device::DmMotorInstructions::kEnable);
      booster_enable_ = false;
    } else if (booster_disable_) {
      booster_->SendInstruction(rm::device::DmMotorInstructions::kClearError);
      booster_->SendInstruction(rm::device::DmMotorInstructions::kDisable);
      booster_disable_ = false;
    } else if (state_ == State::kReady || state_ == State::kCooling || state_ == State::kShooting) {
      // booster_->SetMitCommand(0, 0.0f, booster_speed_pid_->out(), 0.0f, 0.0f);
    }
  }

  [[nodiscard]] State state() const { return state_; }
  [[nodiscard]] float booster_pos() const { return booster_pos_; }

 private:
  State state_{State::kStop};
  bool booster_enable_{false};
  bool booster_disable_{false};

  float now_angle_{0.0f};
  float next_angle_{0.0f};
  float init_time_{0.0f};
  float shoot_time_{0.0f};
  float booster_pos_{0.0f};

  int32_t heat_delta_{0};
  uint16_t heat_limit_{kDefaultHeatLimit};
  bool referee_online_{false};

  FwMotor *fw_[3]{nullptr};
  BoosterMotor *booster_{nullptr};

  std::optional<rm::modules::PID> fw_speed_pid_[3];
  std::optional<rm::modules::PID> booster_pos_pid_{};
  std::optional<rm::modules::PID> booster_speed_pid_{};
};

}  // namespace wheel_legged