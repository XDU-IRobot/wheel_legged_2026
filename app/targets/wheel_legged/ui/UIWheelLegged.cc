//
// Created by refactored from ui.hpp — drone_gb_new style
//

#include "UIWheelLegged.hpp"

#include "librm.hpp"
#include "protocol_user.hpp"
#include "referee_user.hpp"
#include "ui_snapshot.hpp"
#include "targets/wheel_legged/include/globals.hpp"
#include "targets/wheel_legged/include/globals_no_dtcm.hpp"

#include <cmath>

using namespace rm;
using namespace rm::device;

constexpr float leg_big = 107;
constexpr float leg_small = 127;

u8 dataBox[256];
extern SharedResources *globals;
extern SharedResourcesNoDtcm globals_no_dtcm;

// ── Shared working variables for leg kinematics ──
static f32 x, y;
static i16 s_ang = 0, e_ang = 0;
static f32 xx, yy;

void calcPointC(double x1, double y1, double x2, double y2, double L1, double L2, int sel, float *x3_out,
                float *y3_out) {
  double a = 2.0 * (x2 - x1);
  double b = 2.0 * (y2 - y1);
  double K = L1 * L1 - L2 * L2 + (x2 * x2 + y2 * y2) - (x1 * x1 + y1 * y1);

  double p = a * a + b * b;
  double q = -2.0 * b * b * x1 - 2.0 * a * (K - b * y1);
  double r = (K - b * y1) * (K - b * y1) + b * b * (x1 * x1 - L1 * L1);

  double delta = q * q - 4.0 * p * r;
  if (delta < 0) {
    *x3_out = NAN;
    *y3_out = NAN;
    return;
  }

  double sqrtD = sqrt(delta);
  double x3_1 = (-q + sqrtD) / (2.0 * p);
  double x3_2 = (-q - sqrtD) / (2.0 * p);

  double y3_1 = (K - a * x3_1) / b;
  double y3_2 = (K - a * x3_2) / b;

  if (sel == 1) {
    *x3_out = static_cast<float>(x3_1);
    *y3_out = static_cast<float>(y3_1);
  } else {
    *x3_out = static_cast<float>(x3_2);
    *y3_out = static_cast<float>(y3_2);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Static labels (one-shot add)
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedLabelPY_add() {
  UICharacter fig;
  fig.character.fillCharacter("py", UIFigure::Operation::Add, 0, UIFigure::Color::Yellow, 3, static_cast<u16>(8.728),
                              static_cast<u16>(738.454), 30, 5);
  memcpy(fig.data, "P:\nY:", 5);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIWheelLeggedLabelLeg_add() {
  UICharacter fig;
  fig.character.fillCharacter("leg", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, static_cast<u16>(1448),
                              static_cast<u16>(748), 30, 5);
  memcpy(fig.data, "L M H", 10);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIWheelLeggedDecorativeRect_add() {
  UIFigure1 fig;
  fig.figure1.fillRec("dr_", UIFigure::Operation::Add, 0, UIFigure::Color::Yellow, 3, 598, 86, 1315, 120);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Crosshair
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedCrosshair_add() {
  static UIFigure2 crosshair;

  constexpr float kPitchMin = 0.f;
  constexpr float kPitchMax = 0.3f;
  constexpr u16 kEndYAtMinPitch = 290;
  constexpr u16 kEndYAtMaxPitch = 480;

  float pitch = ui_snapshot.gimbal_pitch_rad;
  if (pitch < kPitchMin) pitch = kPitchMin;
  if (pitch > kPitchMax) pitch = kPitchMax;
  const u16 end_y = static_cast<u16>(kEndYAtMinPitch + (pitch - kPitchMin) / (kPitchMax - kPitchMin) *
                                                           (kEndYAtMaxPitch - kEndYAtMinPitch));

  crosshair.figure1.fillLine("ll", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, 547, 86, 732, end_y);
  crosshair.figure2.fillLine("rl", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, 1378, 86, 1170, end_y);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, crosshair, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIWheelLeggedCrosshair_edit() {
  static UIFigure2 crosshair;

  constexpr float kPitchMin = 0.f;
  constexpr float kPitchMax = 0.3f;
  constexpr u16 kEndYAtMinPitch = 290;
  constexpr u16 kEndYAtMaxPitch = 480;

  float pitch = ui_snapshot.gimbal_pitch_rad;
  if (pitch < kPitchMin) pitch = kPitchMin;
  if (pitch > kPitchMax) pitch = kPitchMax;
  const u16 end_y = static_cast<u16>(kEndYAtMinPitch + (pitch - kPitchMin) / (kPitchMax - kPitchMin) *
                                                           (kEndYAtMaxPitch - kEndYAtMinPitch));

  crosshair.figure1.fillLine("ll", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, 547, 86, 732, end_y);
  crosshair.figure2.fillLine("rl", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, 1378, 86, 1170, end_y);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, crosshair, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Gimbal data display (hero variant)
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedGimbalData_add() {
  static UIFigure2 fig;
  fig.figure1.fillFloat("pit", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, static_cast<u16>(77.748),
                        static_cast<u16>(730.583), 25, ui_snapshot.gimbal_pitch_rad * 1000);
  fig.figure2.fillFloat("yaw", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, static_cast<u16>(77.348),
                        static_cast<u16>(690.988), 25, ui_snapshot.gimbal_yaw_rad * 1000);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIWheelLeggedGimbalData_edit() {
  static UIFigure2 fig;
  fig.figure1.fillFloat("pit", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, static_cast<u16>(77.748),
                        static_cast<u16>(730.583), 25, ui_snapshot.gimbal_pitch_rad * 1000);
  fig.figure2.fillFloat("yaw", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, static_cast<u16>(77.348),
                        static_cast<u16>(690.988), 25, ui_snapshot.gimbal_yaw_rad * 1000);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Leg kinematics + supercap energy bar
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedKinematics_add() {
  static UIFigure7 fig;

  u16 rec_x_start = 1435, rec_x_end = 1485;
  {
    switch (static_cast<chassis::Fsm::State>(ui_snapshot.chassis_fsm_state)) {
      case chassis::Fsm::State::kMidLeg:
        rec_x_start = 1485;
        rec_x_end = 1545;
        break;
      case chassis::Fsm::State::kHighLeg:
      case chassis::Fsm::State::kStairTask:
        rec_x_start = 1550;
        rec_x_end = 1610;
        break;
      default:
        break;
    }
  }

  int cap_len = 718 * ui_snapshot.supercap_cap_energy / 255;
  UIFigure::Color cap_color;
  if (ui_snapshot.supercap_cap_energy / 255 < 0.4f) {
    cap_color = UIFigure::Color::Pink;
  } else {
    cap_color = UIFigure::Color::Green;
  }

  u16 lb_x1, lb_y1, lb_x2 = 770, lb_y2 = 700;
  u16 lm_x1, lm_y1, lm_x2, lm_y2;
  u16 rb_x1 = 1135, rb_y1 = 700, rb_x2, rb_y2;
  u16 rm_x1, rm_y1, rm_x2, rm_y2;
  float left_len = ui_snapshot.left_leg_length_m * 500.0f, right_len = ui_snapshot.right_leg_length_m * 500.0f;
  float left_the = ui_snapshot.left_leg_theta_rad, right_the = ui_snapshot.right_leg_theta_rad;
  {
    {
      right_the = rm::modules::Wrap(right_the, 0, 2 * M_PI);
      rm_x2 = static_cast<u16>(static_cast<float>(rb_x1) + right_len * static_cast<float>(sin(right_the)));
      rm_y2 = static_cast<u16>(static_cast<float>(rb_y1) - right_len * static_cast<float>(cos(right_the)));
      xx = rm_x2;
      yy = rm_y2;

      if (right_the > 1.5 * M_PI || right_the < M_PI / 2) {
        calcPointC(rb_x1, rb_y1, rm_x2, rm_y2, leg_big, leg_small, 1, &x, &y);
      } else {
        calcPointC(rb_x1, rb_y1, rm_x2, rm_y2, leg_big, leg_small, 2, &x, &y);
      }
      rm_x1 = static_cast<u16>(x);
      rm_y1 = static_cast<u16>(y);
      rb_x2 = rm_x1;
      rb_y2 = rm_y1;
    }

    {
      left_the = rm::modules::Wrap(left_the, 0, 2 * M_PI);
      lm_x2 = static_cast<u16>(static_cast<float>(lb_x2) - left_len * static_cast<float>(sin(left_the)));
      lm_y2 = static_cast<u16>(static_cast<float>(lb_y2) - left_len * static_cast<float>(cos(left_the)));

      if (left_the > 1.5 * M_PI || left_the < M_PI / 2) {
        calcPointC(lb_x2, lb_y2, lm_x2, lm_y2, leg_big, leg_small, 2, &x, &y);
      } else {
        calcPointC(lb_x2, lb_y2, lm_x2, lm_y2, leg_big, leg_small, 1, &x, &y);
      }
      lm_x1 = static_cast<u16>(x);
      lm_y1 = static_cast<u16>(y);
      lb_x1 = lm_x1;
      lb_y1 = lm_y1;
    }
  }

  if (ui_snapshot.leg_view_flip) {
    constexpr u16 left_axis_x = 660;
    constexpr u16 right_axis_x = 1235;
    auto mirror_x = [](const u16 axis_x, const u16 value) -> u16 {
      const int mirrored = static_cast<int>(axis_x) * 2 - static_cast<int>(value);
      return static_cast<u16>(mirrored < 0 ? 0 : mirrored);
    };

    lb_x1 = mirror_x(left_axis_x, lb_x1);
    lm_x1 = mirror_x(left_axis_x, lm_x1);
    lm_x2 = mirror_x(left_axis_x, lm_x2);
    rb_x2 = mirror_x(right_axis_x, rb_x2);
    rm_x1 = mirror_x(right_axis_x, rm_x1);
    rm_x2 = mirror_x(right_axis_x, rm_x2);
  }

  {
    s_ang = static_cast<i16>((ui_snapshot.yaw_motor_raw_pos_rad) * 57.3 - 30);
    e_ang = static_cast<i16>((ui_snapshot.yaw_motor_raw_pos_rad) * 57.3 + 30);
    if (s_ang < 0) s_ang += 360;
    if (e_ang < 0) e_ang += 360;
    if (s_ang > 360) s_ang -= 360;
    if (e_ang > 360) e_ang -= 360;
  }

  fig.figure1.fillLine("l1", UIFigure::Operation::Add, 0, cap_color, 34, 598, 103, 598 + cap_len, 103);
  fig.figure2.fillLine("l2", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, lb_x1, lb_y1, lb_x2, lb_y2);
  fig.figure3.fillLine("l3", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, lm_x1, lm_y1, lm_x2, lm_y2);
  fig.figure4.fillLine("l4", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, rb_x1, rb_y1, rb_x2, rb_y2);
  fig.figure5.fillLine("l5", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 5, rm_x1, rm_y1, rm_x2, rm_y2);
  fig.figure6.fillArc("a1", UIFigure::Operation::Add, 0, UIFigure::Color::Yellow, 5, 957, 538, s_ang, e_ang, 77, 77);
  fig.figure7.fillRec("r1", UIFigure::Operation::Add, 0, UIFigure::Color::White, 2, rec_x_start, 700, rec_x_end, 770);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 50);
}

void UIWheelLeggedKinematics_edit() {
  static UIFigure7 fig;

  u16 rec_x_start = 1435, rec_x_end = 1485;
  {
    switch (static_cast<chassis::Fsm::State>(ui_snapshot.chassis_fsm_state)) {
      case chassis::Fsm::State::kMidLeg:
        rec_x_start = 1485;
        rec_x_end = 1545;
        break;
      case chassis::Fsm::State::kHighLeg:
      case chassis::Fsm::State::kStairTask:
        rec_x_start = 1550;
        rec_x_end = 1610;
        break;
      default:
        break;
    }
  }

  int cap_len = 718 * ui_snapshot.supercap_cap_energy / 255;
  UIFigure::Color cap_color;
  if (ui_snapshot.supercap_cap_energy / 255 < 0.4f) {
    cap_color = UIFigure::Color::Pink;
  } else {
    cap_color = UIFigure::Color::Green;
  }

  u16 lb_x1, lb_y1, lb_x2 = 770, lb_y2 = 700;
  u16 lm_x1, lm_y1, lm_x2, lm_y2;
  u16 rb_x1 = 1135, rb_y1 = 700, rb_x2, rb_y2;
  u16 rm_x1, rm_y1, rm_x2, rm_y2;
  float left_len = ui_snapshot.left_leg_length_m * 500.0f, right_len = ui_snapshot.right_leg_length_m * 500.0f;
  float left_the = ui_snapshot.left_leg_theta_rad, right_the = ui_snapshot.right_leg_theta_rad;
  {
    {
      right_the = rm::modules::Wrap(right_the, 0, 2 * M_PI);
      rm_x2 = static_cast<u16>(static_cast<float>(rb_x1) + right_len * static_cast<float>(sin(right_the)));
      rm_y2 = static_cast<u16>(static_cast<float>(rb_y1) - right_len * static_cast<float>(cos(right_the)));
      xx = rm_x2;
      yy = rm_y2;

      if (right_the > 1.5 * M_PI || right_the < M_PI / 2) {
        calcPointC(rb_x1, rb_y1, rm_x2, rm_y2, leg_big, leg_small, 1, &x, &y);
      } else {
        calcPointC(rb_x1, rb_y1, rm_x2, rm_y2, leg_big, leg_small, 2, &x, &y);
      }
      rm_x1 = static_cast<u16>(x);
      rm_y1 = static_cast<u16>(y);
      rb_x2 = rm_x1;
      rb_y2 = rm_y1;
    }

    {
      left_the = rm::modules::Wrap(left_the, 0, 2 * M_PI);
      lm_x2 = static_cast<u16>(static_cast<float>(lb_x2) - left_len * static_cast<float>(sin(left_the)));
      lm_y2 = static_cast<u16>(static_cast<float>(lb_y2) - left_len * static_cast<float>(cos(left_the)));

      if (left_the > 1.5 * M_PI || left_the < M_PI / 2) {
        calcPointC(lb_x2, lb_y2, lm_x2, lm_y2, leg_big, leg_small, 2, &x, &y);
      } else {
        calcPointC(lb_x2, lb_y2, lm_x2, lm_y2, leg_big, leg_small, 1, &x, &y);
      }
      lm_x1 = static_cast<u16>(x);
      lm_y1 = static_cast<u16>(y);
      lb_x1 = lm_x1;
      lb_y1 = lm_y1;
    }
  }

  if (ui_snapshot.leg_view_flip) {
    constexpr u16 left_axis_x = 660;
    constexpr u16 right_axis_x = 1235;
    auto mirror_x = [](const u16 axis_x, const u16 value) -> u16 {
      const int mirrored = static_cast<int>(axis_x) * 2 - static_cast<int>(value);
      return static_cast<u16>(mirrored < 0 ? 0 : mirrored);
    };

    lb_x1 = mirror_x(left_axis_x, lb_x1);
    lm_x1 = mirror_x(left_axis_x, lm_x1);
    lm_x2 = mirror_x(left_axis_x, lm_x2);
    rb_x2 = mirror_x(right_axis_x, rb_x2);
    rm_x1 = mirror_x(right_axis_x, rm_x1);
    rm_x2 = mirror_x(right_axis_x, rm_x2);
  }

  {
    s_ang = static_cast<i16>((ui_snapshot.yaw_motor_raw_pos_rad) * 57.3 - 30);
    e_ang = static_cast<i16>((ui_snapshot.yaw_motor_raw_pos_rad) * 57.3 + 30);
    if (s_ang < 0) s_ang += 360;
    if (e_ang < 0) e_ang += 360;
    if (s_ang > 360) s_ang -= 360;
    if (e_ang > 360) e_ang -= 360;
  }

  fig.figure1.fillLine("l1", UIFigure::Operation::Edit, 0, cap_color, 34, 598, 103, 598 + cap_len, 103);
  fig.figure2.fillLine("l2", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, lb_x1, lb_y1, lb_x2, lb_y2);
  fig.figure3.fillLine("l3", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, lm_x1, lm_y1, lm_x2, lm_y2);
  fig.figure4.fillLine("l4", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, rb_x1, rb_y1, rb_x2, rb_y2);
  fig.figure5.fillLine("l5", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 5, rm_x1, rm_y1, rm_x2, rm_y2);
  fig.figure6.fillArc("a1", UIFigure::Operation::Edit, 0, UIFigure::Color::Yellow, 5, 957, 538, s_ang, e_ang, 77, 77);
  fig.figure7.fillRec("r1", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 2, rec_x_start, 700, rec_x_end, 770);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 50);
}

// ═══════════════════════════════════════════════════════════════════════════
// Friction wheel RPM
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedFricRPM_add() {
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  static UIFigure5 fig;
  fig.figure1.fillIntegrate("f1_", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, 1750, 722, 20,
                            static_cast<i32>(ui_snapshot.fw_raw_rpm_1));
  fig.figure2.fillIntegrate("f2_", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, 1750, 762, 20,
                            static_cast<i32>(ui_snapshot.fw_raw_rpm_2));
  fig.figure3.fillIntegrate("f3_", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, 1750, 802, 20,
                            static_cast<i32>(ui_snapshot.fw_raw_rpm_3));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 50);
#else
  static UIFigure2 fig;
  fig.figure1.fillIntegrate("fL_", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, 1450, 646, 20,
                            static_cast<i32>(ui_snapshot.fric_left_rpm));
  fig.figure2.fillIntegrate("fR_", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, 1450, 676, 20,
                            static_cast<i32>(ui_snapshot.fric_right_rpm));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
#endif
}

void UIWheelLeggedFricRPM_edit() {
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  static UIFigure5 fig;
  fig.figure1.fillIntegrate("f1_", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, 1750, 722, 20,
                            static_cast<i32>(ui_snapshot.fw_raw_rpm_1));
  fig.figure2.fillIntegrate("f2_", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, 1750, 762, 20,
                            static_cast<i32>(ui_snapshot.fw_raw_rpm_2));
  fig.figure3.fillIntegrate("f3_", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, 1750, 802, 20,
                            static_cast<i32>(ui_snapshot.fw_raw_rpm_3));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 50);
#else
  static UIFigure2 fig;
  fig.figure1.fillIntegrate("fL_", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, 1450, 646, 20,
                            static_cast<i32>(ui_snapshot.fric_left_rpm));
  fig.figure2.fillIntegrate("fR_", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, 1450, 676, 20,
                            static_cast<i32>(ui_snapshot.fric_right_rpm));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// Bullet speed + projectile allowance
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedBulletData_add() {
  static UIFigure2 fig;
  fig.figure1.fillFloat("spd", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, static_cast<u16>(1343.216),
                        static_cast<u16>(644.661), 25, ui_snapshot.bullet_speed_mps);
  fig.figure2.fillIntegrate("amm", UIFigure::Operation::Add, 0, UIFigure::Color::Green, 3, static_cast<u16>(1346.347),
                            static_cast<u16>(472.9), 25, static_cast<i32>(ui_snapshot.projectile_allowance));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIWheelLeggedBulletData_edit() {
  static UIFigure2 fig;
  fig.figure1.fillFloat("spd", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, static_cast<u16>(1343.216),
                        static_cast<u16>(644.661), 25, ui_snapshot.bullet_speed_mps);
  fig.figure2.fillIntegrate("amm", UIFigure::Operation::Edit, 0, UIFigure::Color::Green, 3, static_cast<u16>(1346.347),
                            static_cast<u16>(472.9), 25, static_cast<i32>(ui_snapshot.projectile_allowance));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Status labels (infantry variant, rotating)
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedStatusLabel_add() {
  static UICharacter status_line;
  static u8 status_line_index = 0;
  static u8 init_count = 0;

  const u8 sender = ui_snapshot.referee_robot_id;
  const auto op = init_count < 3 ? UIFigure::Operation::Add : UIFigure::Operation::Edit;

  memset(status_line.data, 0, sizeof(status_line.data));
  switch (status_line_index) {
    case 0:
      status_line.character.fillCharacter("st1", op, 0, UIFigure::Color::Yellow, 2, 740, 348, 20, 22);
      memcpy(status_line.data, "DISABLE STANDBY ENABLE", 22);
      break;
    case 1:
      status_line.character.fillCharacter("st2", op, 0, UIFigure::Color::Yellow, 2, 740, 308, 20, 16);
      if (ui_snapshot.ad_active) {
        status_line.character.fillCharacter("sa2", op, 0, UIFigure::Color::Yellow, 2, 760, 308, 6, 16);
      } else {
        status_line.character.fillCharacter("sa2", UIFigure::Operation::Delete, 0, UIFigure::Color::Yellow, 2, 760, 308,
                                            6, 16);
      }
      memcpy(status_line.data, "SPIN CROSS AD", 13);
      break;
    default:
      status_line.character.fillCharacter("st3", op, 0, UIFigure::Color::Yellow, 2, 740, 268, 20, 16);
      memcpy(status_line.data, "NORMAL SMALL BIG", 16);
      break;
  }

  u8 len = Referee0x301Prepare(dataBox, 0, status_line, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
  status_line_index = (status_line_index + 1) % 3;
  if (init_count < 3) init_count++;
}

// ═══════════════════════════════════════════════════════════════════════════
// State indicator (infantry variant)
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedStateIndicator_add() {
  static UIFigure5 fig;

  auto fill_hidden = [](UIFigure &figure, const char *name, UIFigure::Operation op) {
    figure.fillRec(name, op, 0, UIFigure::Color::Black, 1, 0, 0, 1, 1);
  };

  auto fill_rect = [](UIFigure &figure, const char *name, UIFigure::Operation op, u16 x1, u16 y1, u16 x2, u16 y2) {
    figure.fillRec(name, op, 0, UIFigure::Color::White, 2, x1, y1, x2, y2);
  };

  const bool disabled = ui_snapshot.domain_request == static_cast<u8>(wheel_legged::DomainRequest::kDisabled) ||
                        ui_snapshot.chassis_fsm_state == static_cast<u8>(chassis::Fsm::State::kDisabled);
  const bool enabled = !disabled && !ui_snapshot.standby;

  if (disabled) {
    fill_rect(fig.figure1, "ps_", UIFigure::Operation::Add, 736, 318, 872, 358);
  } else if (ui_snapshot.standby) {
    fill_rect(fig.figure1, "ps_", UIFigure::Operation::Add, 880, 318, 1048, 358);
  } else if (enabled) {
    fill_rect(fig.figure1, "ps_", UIFigure::Operation::Add, 1058, 318, 1200, 358);
  } else {
    fill_hidden(fig.figure1, "ps_", UIFigure::Operation::Add);
  }

  if (ui_snapshot.spin_active) {
    fill_rect(fig.figure2, "mv_", UIFigure::Operation::Add, 736, 278, 832, 318);
  } else if (ui_snapshot.cross_active) {
    fill_rect(fig.figure2, "mv_", UIFigure::Operation::Add, 842, 278, 965, 318);
  } else {
    fill_hidden(fig.figure2, "mv_", UIFigure::Operation::Add);
  }

  {
    const bool has_target = globals->aimbot.has_value() && (globals->aimbot->aimbot_state() & 1) == 1;
    UIFigure::Color am_color;
    if (ui_snapshot.auto_aim_hold) {
      am_color = has_target ? UIFigure::Color::Green : UIFigure::Color::Yellow;
    } else {
      am_color = UIFigure::Color::White;
    }
    u16 x1 = 290, y1 = 238, x2 = 290, y2 = 278;
    switch (ui_snapshot.aim_mode) {
      case 1:
        x1 = 876;
        x2 = 976;
        break;
      case 2:
        x1 = 994;
        x2 = 1060;
        break;
      default:
        x1 = 736;
        x2 = 858;
        break;
    }
    fig.figure3.fillRec("am_", UIFigure::Operation::Add, 0, am_color, 2, x1, y1, x2, y2);
  }

  fill_hidden(fig.figure4, "u4_", UIFigure::Operation::Add);
  fill_hidden(fig.figure5, "u5_", UIFigure::Operation::Add);

  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIWheelLeggedStateIndicator_edit() {
  static UIFigure5 fig;

  auto fill_hidden = [](UIFigure &figure, const char *name, UIFigure::Operation op) {
    figure.fillRec(name, op, 0, UIFigure::Color::Black, 1, 0, 0, 1, 1);
  };

  auto fill_rect = [](UIFigure &figure, const char *name, UIFigure::Operation op, u16 x1, u16 y1, u16 x2, u16 y2) {
    figure.fillRec(name, op, 0, UIFigure::Color::White, 2, x1, y1, x2, y2);
  };

  const bool disabled = ui_snapshot.domain_request == static_cast<u8>(wheel_legged::DomainRequest::kDisabled) ||
                        ui_snapshot.chassis_fsm_state == static_cast<u8>(chassis::Fsm::State::kDisabled);
  const bool enabled = !disabled && !ui_snapshot.standby;

  if (disabled) {
    fill_rect(fig.figure1, "ps_", UIFigure::Operation::Edit, 736, 318, 872, 358);
  } else if (ui_snapshot.standby) {
    fill_rect(fig.figure1, "ps_", UIFigure::Operation::Edit, 880, 318, 1048, 358);
  } else if (enabled) {
    fill_rect(fig.figure1, "ps_", UIFigure::Operation::Edit, 1058, 318, 1200, 358);
  } else {
    fill_hidden(fig.figure1, "ps_", UIFigure::Operation::Edit);
  }

  if (ui_snapshot.spin_active) {
    fill_rect(fig.figure2, "mv_", UIFigure::Operation::Edit, 736, 278, 832, 318);
  } else if (ui_snapshot.cross_active) {
    fill_rect(fig.figure2, "mv_", UIFigure::Operation::Edit, 842, 278, 965, 318);
  } else {
    fill_hidden(fig.figure2, "mv_", UIFigure::Operation::Edit);
  }

  {
    const bool has_target = globals->aimbot.has_value() && (globals->aimbot->aimbot_state() & 1) == 1;
    UIFigure::Color am_color;
    if (ui_snapshot.auto_aim_hold) {
      am_color = has_target ? UIFigure::Color::Green : UIFigure::Color::Yellow;
    } else {
      am_color = UIFigure::Color::White;
    }
    u16 x1 = 290, y1 = 238, x2 = 290, y2 = 278;
    switch (ui_snapshot.aim_mode) {
      case 1:
        x1 = 876;
        x2 = 976;
        break;
      case 2:
        x1 = 994;
        x2 = 1060;
        break;
      default:
        x1 = 736;
        x2 = 858;
        break;
    }
    fig.figure3.fillRec("am_", UIFigure::Operation::Edit, 0, am_color, 2, x1, y1, x2, y2);
  }

  fill_hidden(fig.figure4, "u4_", UIFigure::Operation::Edit);
  fill_hidden(fig.figure5, "u5_", UIFigure::Operation::Edit);

  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Aimbot target box
// ═══════════════════════════════════════════════════════════════════════════

void UIWheelLeggedAimbotBox_add() {
  static UIFigure5 box;

  const bool has_target = globals->aimbot.has_value() && (globals->aimbot->aimbot_state() & 1) == 1;
  const auto color = has_target ? UIFigure::Color::Green : UIFigure::Color::White;

  auto fill_hidden = [](UIFigure &figure, const char *name, UIFigure::Operation op) {
    figure.fillRec(name, op, 0, UIFigure::Color::Black, 1, 0, 0, 1, 1);
  };

  box.figure1.fillRec("r1_", UIFigure::Operation::Add, 0, color, 3, 781, 361, 1129, 709);
  if (has_target) {
    box.figure2.fillIntegrate("ahp", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 3, 790, 420, 20,
                              static_cast<i32>(ui_snapshot.aimbot_target_hp));
    box.figure3.fillIntegrate("aal", UIFigure::Operation::Add, 0, UIFigure::Color::White, 3, 790, 380, 20,
                              static_cast<i32>(ui_snapshot.aimbot_target_allowance));
  } else {
    fill_hidden(box.figure2, "ahp", UIFigure::Operation::Add);
    fill_hidden(box.figure3, "aal", UIFigure::Operation::Add);
  }

  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, box, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIWheelLeggedAimbotBox_edit() {
  static UIFigure5 box;

  const bool has_target = globals->aimbot.has_value() && (globals->aimbot->aimbot_state() & 1) == 1;
  const auto color = has_target ? UIFigure::Color::Green : UIFigure::Color::White;

  auto fill_hidden = [](UIFigure &figure, const char *name, UIFigure::Operation op) {
    figure.fillRec(name, op, 0, UIFigure::Color::Black, 1, 0, 0, 1, 1);
  };

  box.figure1.fillRec("r1_", UIFigure::Operation::Edit, 0, color, 3, 781, 361, 1129, 709);
  if (has_target) {
    box.figure2.fillIntegrate("ahp", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 3, 790, 420, 20,
                              static_cast<i32>(ui_snapshot.aimbot_target_hp));
    box.figure3.fillIntegrate("aal", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 3, 790, 380, 20,
                              static_cast<i32>(ui_snapshot.aimbot_target_allowance));
  } else {
    fill_hidden(box.figure2, "ahp", UIFigure::Operation::Edit);
    fill_hidden(box.figure3, "aal", UIFigure::Operation::Edit);
  }

  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, box, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}
