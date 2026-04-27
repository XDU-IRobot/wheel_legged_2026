#include "communication.h"

namespace communication {

ChassisReceive::ChassisReceive(rm::hal::CanInterface &can, rm::u32 rx_std_id) : rm::device::CanDevice(can, rx_std_id) {
  command_.chassis.state = ChassisState::kUnable;
  command_.chassis.leg_length = LegLength::kNormal;
  command_.chassis.move_x = 0;
  command_.chassis.move_y = 0;
  command_.ui = {0, 0, 0, 0};
}

void ChassisReceive::RxCallback(const rm::hal::CanFrame *msg) {
  if (msg == nullptr || msg->dlc < 4U) {
    return;
  }

  // 收到有效协议帧即认为设备在线，便于上层做离线保护。
  ReportStatus(rm::device::Device::kOk);

  // 协议保持不变：
  // byte0: [高4位=腿长][低4位=状态]
  // byte1: move_x
  // byte2: move_y
  // byte3~6: UI 指令
  command_.chassis.state = static_cast<ChassisState>(msg->data[0] & 0x0FU);
  command_.chassis.leg_length = static_cast<LegLength>((msg->data[0] >> 4) & 0x0FU);
  command_.chassis.move_y = static_cast<rm::i8>(msg->data[1]);
  command_.chassis.move_x = static_cast<rm::i8>(msg->data[2]);
  command_.ui.ui1 = msg->data[3];
}

Command ChassisReceive::getCommand() const { return command_; }

void Communication::SendCommandToGimbal() {
  // 当前项目只保留接收协议，发送协议后续按裁判系统/云台回传再补齐。
  tx_buf_gimbal_.fill(0);
}

}  // namespace communication
