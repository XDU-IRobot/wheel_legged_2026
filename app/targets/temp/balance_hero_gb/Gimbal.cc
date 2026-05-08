#include "Gimbal.hpp"

void Gimbal::GimbalInit() {
  gimbal->gimbal_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->pitch();
}

void Gimbal::GimbalTask() { gimbal->GimbalStateUpdate(); }

void Gimbal::GimbalStateUpdate() {
  if (!globals->device_gimbal.all_device_ok()) {
    gimbal->GimbalDisableUpdate();
  } else {
    switch (globals->StateMachine_) {
      case kNoForce:                    // 无力模式下，所有电机失能
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;

      case kTest:
        switch (gimbal->GimbalMove_) {
          case kGbRemote:
            gimbal->GimbalEnableUpdate();
            break;
          case kGbAimbot:
            gimbal->GimbalEnableUpdate();
            break;
          default:
            gimbal->GimbalDisableUpdate();
            break;
        }
        break;
      case kMatch:
        gimbal->GimbalMatchUpdate();
        break;
      default:
        gimbal->GimbalDisableUpdate();  // 云台电机失能计算
        break;
    }
  }
}

void Gimbal::GimbalRCTargetUpdate() {
  gimbal->gimbal_yaw_target_ -= rm::modules::Map(globals->rc->left_x(), -globals->rc_max_value_, globals->rc_max_value_,
                                                 -gimbal->sensitivity_, gimbal->sensitivity_);  // 上部yaw轴目标值
  gimbal->gimbal_pitch_target_ -= rm::modules::Map(globals->rc->left_y(), -globals->rc_max_value_,  // pitch轴目标值
                                                   globals->rc_max_value_, -gimbal->sensitivity_, gimbal->sensitivity_);
  gimbal->gimbal_yaw_target_ =
      rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);  // yaw轴周期限位
  gimbal->gimbal_pitch_target_ = rm::modules::Clamp(gimbal->gimbal_pitch_target_, gimbal->lowest_pitch_angle_,
                                                    gimbal->highest_pitch_angle_);  // pitch轴限位
}

void Gimbal::GimbalAimbotTargetUpdate() {
  if (globals->can_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->gimbal_yaw_target_ = globals->can_communicator->yaw();
    gimbal->gimbal_pitch_target_ = globals->can_communicator->pitch();
  } else {
    gimbal->GimbalRCTargetUpdate();
  }
  gimbal->gimbal_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_pitch_target_ =
      rm::modules::Clamp(gimbal->gimbal_pitch_target_, gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalScanTargetUpdate() {
  f32 yaw_range = static_cast<f32>(M_PI);
  if (gimbal->gimbal_yaw_target_ >= yaw_range) {
    gimbal->scan_yaw_flag_ = true;
  } else if (gimbal->gimbal_yaw_target_ <= -yaw_range) {
    gimbal->scan_yaw_flag_ = false;
  }
  if (gimbal->scan_yaw_flag_) {
    gimbal->gimbal_yaw_target_ -= 0.008f;
  } else {
    gimbal->gimbal_yaw_target_ += 0.008f;
  }
  if (gimbal->gimbal_pitch_target_ >= gimbal->highest_pitch_angle_) {
    gimbal->scan_pitch_flag_ = true;
  } else if (gimbal->gimbal_pitch_target_ <= gimbal->lowest_pitch_angle_) {
    gimbal->scan_pitch_flag_ = false;
  }
  if (gimbal->scan_pitch_flag_) {
    gimbal->gimbal_pitch_target_ -= 0.003f;
  } else {
    gimbal->gimbal_pitch_target_ += 0.003f;
  }
  gimbal->gimbal_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);
  gimbal->gimbal_pitch_target_ =
      rm::modules::Clamp(gimbal->gimbal_pitch_target_, gimbal->lowest_pitch_angle_, gimbal->highest_pitch_angle_);
}

void Gimbal::GimbalDownYawFollow() {
  gimbal->gimbal_yaw_target_ = rm::modules::Wrap(gimbal->gimbal_yaw_target_, -static_cast<f32>(M_PI), M_PI);
}

void Gimbal::GimbalMatchUpdate() {
  if (globals->can_communicator->aimbot_state() >> 0 & 0x01) {
    gimbal->GimbalMove_ = kGbAimbot;
  } else {
    gimbal->GimbalMove_ = kGbRemote;
  }
  gimbal->GimbalEnableUpdate();
}

void Gimbal::GimbalMovePIDUpdate() {
  // 前馈，无roll轴补偿
  globals->yaw_speed_feedforward->Update(gimbal_yaw_target_);
  if (gimbal->GimbalMove_ == kGbRemote && globals->rc->switch_l() == DR16::SwitchPosition::kDown &&
      globals->rc->switch_r() == DR16::SwitchPosition::kUp) {
    globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_,
                                         globals->yaw_speed_feedforward->GetYawSpeedFeedforward() - 10);
  } else {
    globals->gimbal_controller.SetTarget(gimbal->gimbal_yaw_target_, gimbal->gimbal_pitch_target_,
                                         globals->yaw_speed_feedforward->GetYawSpeedFeedforward());
  }
  globals->gimbal_controller.Update(globals->hipnuc_imu->yaw(), globals->yaw_motor->vel(), globals->hipnuc_imu->pitch(),
                                    globals->pitch_motor->vel());
}

void Gimbal::GimbalEnableUpdate() {
  gimbal->DaMiaoMotorEnable();
  globals->gimbal_controller.Enable(true);
  if (gimbal->GimbalMove_ == kGbRemote) {
    // globals->GimbalData.aim_mode = 0x00;
    // globals->aim_mode = 0x00;
    gimbal->GimbalRCTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else if (gimbal->GimbalMove_ == kGbAimbot) {
    // globals->GimbalData.aim_mode = 0x01;
    // globals->aim_mode = 0x01;
    gimbal->GimbalAimbotTargetUpdate();
    gimbal->GimbalMovePIDUpdate();
  } else {
    // globals->GimbalData.aim_mode = 0x00;
    // globals->aim_mode = 0x00;
    globals->gimbal_controller.Enable(false);
  }
}

void Gimbal::GimbalDisableUpdate() {
  gimbal->DaMiaoMotorDisable();
  globals->gimbal_controller.Enable(false);
  // globals->GimbalData.aim_mode = 0x00;
  gimbal->gimbal_yaw_target_ = globals->hipnuc_imu->yaw();
  gimbal->gimbal_pitch_target_ = globals->hipnuc_imu->pitch();
  gimbal->GimbalMovePIDUpdate();
}

void Gimbal::DaMiaoMotorEnable() {
  if (gimbal->DM_enable_flag_ == false) {
    // 使达妙电机使能
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    HAL_Delay(0.0003);
    globals->yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    HAL_Delay(0.0003);
    gimbal->DM_enable_flag_ = true;
  }
}

void Gimbal::DaMiaoMotorDisable() {
  if (gimbal->DM_enable_flag_ == true) {
    // 使达妙电机失能
    // while (globals->yaw_motor->status())
    globals->yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    HAL_Delay(0.0003);
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    HAL_Delay(0.0003);
    gimbal->DM_enable_flag_ = false;
  }
}