#include "protocol_user.hpp"
#include "referee_user.hpp"
#include "TaskScheduler.hpp"
#include "librm.hpp"
#include "targets/wheel_legged/include/globals.hpp"
#include "targets/wheel_legged/include/globals_no_dtcm.hpp"

rm::u8 info[256];
void static1_func();
void static2_func();
void static3_func();
void static4_func();
void dynamic1_func();
void dynamic2_func();
static auto schedule = rm::device::UITaskScheduler(30);
static auto UI_static1 = rm::device::UITask(static1_func,0.5);
static auto UI_static2 = rm::device::UITask(static2_func,0.5);
static auto UI_static3 = rm::device::UITask(static3_func,0.5);
static auto UI_static4 = rm::device::UITask(static4_func,0.5);
static auto UI_dynamic1 = rm::device::UITask(dynamic1_func,1.5);
static auto UI_dynamic2 = rm::device::UITask(dynamic2_func,1.5);

void ui_init() {
  schedule.addTask(&UI_static1);
  schedule.addTask(&UI_static2);
  schedule.addTask(&UI_static3);
  schedule.addTask(&UI_static4);
  schedule.addTask(&UI_dynamic1);
  schedule.addTask(&UI_dynamic2);
}

void static1_func() {
  static rm::device::UICharacter static_1;
  if (globals->dr16.right_y() > 300) {
    static_1.character.fillCharacter("py",device::UIFigure1::Operation::Add,0,device::UIFigure1::Color::Yellow,
                                      3,8.728,738.454,30,5);
    memcpy(static_1.data,"P:\nY:",5);
    u8 len = rm::device::Referee0x301Prepare(info,0,static_1,0x01,0x01+256);
    globals_no_dtcm.referee_uart.Write(info,len,10);
  }
}

void static2_func() {
  static rm::device::UICharacter static_2;
  if (globals->dr16.right_y() > 300) {
    static_2.character.fillCharacter("leg",device::UIFigure1::Operation::Add,0,device::UIFigure1::Color::Yellow,
                                      3,1600.849,862.595,30,10);
    memcpy(static_2.data,"LEG: L M H",10);
    u8 len = rm::device::Referee0x301Prepare(info,0,static_2,0x01,0x01+256);
    globals_no_dtcm.referee_uart.Write(info,len,10);
  }
}

void static3_func() {
  static rm::device::UIFigure2 static_3;
  if (globals->dr16.right_y() > 300) {
    static_3.figure1.fillLine("ll",device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::RedBlue,
                              5,547,86,722,516);
    static_3.figure2.fillLine("rl",device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::RedBlue,
                              5,1378,86,1173,516);
    u8 len = rm::device::Referee0x301Prepare(info,0,static_3,0x01,0x01+256);
    globals_no_dtcm.referee_uart.Write(info,len,10);
  }
}

void static4_func() {
  static rm::device::UIFigure2 static_4;
  if (globals->dr16.right_y() > 300) {
    static_4.figure1.fillRec("r1_", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::White,
                             3, 781, 361, 1129, 709);
    static_4.figure2.fillRec("r2_", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow,
                             3, 598, 836, 1315, 870);
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_4, 0x01, 0x01 + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

void dynamic1_func() {
  static rm::device::UIFigure2 static_d1;
  static bool added = false;
  if (globals->dr16.right_y() > 300) {
    static_d1.figure1.fillFloat("pit", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green,
                            3, 77.748, 730.583, 25, globals->gimbal_rx->pitch_rad() * 1000);
    static_d1.figure2.fillFloat("yaw", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green,
                            3, 77.348, 690.988, 25, globals->gimbal_rx->yaw_rad() * 1000);
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d1, 0x01, 0x01 + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
    added = true;
  } else if (added) {
    static_d1.figure1.fillFloat("pit", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green,
                        3, 77.748, 730.583, 25, globals->gimbal_rx->pitch_rad() * 1000);
    static_d1.figure2.fillFloat("yaw", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green,
                        3, 77.348, 690.988, 25, globals->gimbal_rx->yaw_rad() * 1000);
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d1, 0x01, 0x01 + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

void dynamic2_func() {
  static rm::device::UIFigure7 static_d2;
  static bool added = false;
  u16 rec_x_start = 1735, rec_x_end = 1785;
  switch (globals->chassis_fsm.mode()) {
    case chassis::Fsm::State::kMidLeg: rec_x_start = 1785; rec_x_end = 1845; break;
    case chassis::Fsm::State::kHighLeg: rec_x_start = 1850; rec_x_end = 1910; break;
    default: break;
  }
  if (globals->dr16.right_y() > 300) {
    static_d2.figure1.fillLine("l1", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green,
                               34, 598, 853, 598 + 100, 853);
    static_d2.figure2.fillLine("l2", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::RedBlue,
                               5, 549, 766, 660, 766);
    static_d2.figure3.fillLine("l3", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::RedBlue,
                               5, 549, 763, 649, 663);
    static_d2.figure4.fillLine("l4", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::RedBlue,
                                5, 1235, 759, 1346, 759);
    static_d2.figure5.fillLine("l5", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::RedBlue,
                              5, 1343, 761, 1232, 661);
    static_d2.figure6.fillArc("a1", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow,
                              5, 957, 538, 340, 20, 77, 77);
    static_d2.figure7.fillRec("r1", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::White,
                              2, rec_x_start, 810, rec_x_end, 880);
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d2, 0x01, 0x01 + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 50);
    added = true;
  } else if (added) {
    static_d2.figure1.fillLine("l1", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green,
                               34, 598, 853, 598 + 100, 853);
    static_d2.figure2.fillLine("l2", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::RedBlue,
                               5, 549, 766, 660, 766);
    static_d2.figure3.fillLine("l3", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::RedBlue,
                                5, 549, 763, 649, 663);
    static_d2.figure4.fillLine("l4", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::RedBlue,
                               5, 1235, 759, 1346, 759);
    static_d2.figure5.fillLine("l5", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::RedBlue,
                               5, 1343, 761, 1232, 661);
    static_d2.figure6.fillArc("a1", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Yellow,
                              5, 957, 538, 340, 20, 77, 77);
    static_d2.figure7.fillRec("r1", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::White,
                              2, rec_x_start, 810, rec_x_end, 880);
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d2, 0x01, 0x01 + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 50);
  }
}

