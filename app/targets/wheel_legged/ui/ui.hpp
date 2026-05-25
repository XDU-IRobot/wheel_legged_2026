
#include "protocol_user.hpp"
#include "referee_user.hpp"
#include "TaskScheduler.hpp"
#include "ui_snapshot.hpp"
#include "librm.hpp"
#include "targets/wheel_legged/include/globals.hpp"
#include "targets/wheel_legged/include/globals_no_dtcm.hpp"
constexpr float leg_big = 107;
constexpr float leg_small = 127;

inline f32 x, y;
inline i16 s_ang = 0, e_ang = 0;
inline f32 xx, yy;

inline rm::u8 info[256];
void static1_func();
void static2_func();
void dynamic3_crosshair_func();
void static4_func();
void static5_func();
void static_status_func();
void dynamic1_func();
void dynamic2_func();
void dynamic3_func();
void dynamic4_func();
void dynamic_status_func();
void dynamic_aimbot_box_func();
inline auto schedule = rm::device::UITaskScheduler(30);
inline auto UI_static1 = rm::device::UITask(static1_func, 0.5);
inline auto UI_static2 = rm::device::UITask(static2_func, 0.5);
inline auto UI_static3 = rm::device::UITask(dynamic3_crosshair_func, 1.5);
inline auto UI_static4 = rm::device::UITask(static4_func, 0.5);
inline auto UI_static5 = rm::device::UITask(static5_func, 0.5);
inline auto UI_static_status = rm::device::UITask(static_status_func, 3);
inline auto UI_dynamic1 = rm::device::UITask(dynamic1_func, 1.5);
inline auto UI_dynamic2 = rm::device::UITask(dynamic2_func, 3);
inline auto UI_dynamic3 = rm::device::UITask(dynamic3_func, 3);
inline auto UI_dynamic4 = rm::device::UITask(dynamic4_func, 3);
inline auto UI_dynamic_status = rm::device::UITask(dynamic_status_func, 3);
inline auto UI_dynamic_aimbot_box = rm::device::UITask(dynamic_aimbot_box_func, 3);

inline u8 robot_id() { return ui_snapshot.referee_robot_id; }

inline void calcPointC(double x1, double y1, double x2, double y2, double L1, double L2, int sel, float *x3_out,
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

inline void ui_init() {
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  schedule.addTask(&UI_static1);
#else
  schedule.addTask(&UI_static_status);
#endif
  schedule.addTask(&UI_static2);
  schedule.addTask(&UI_static4);
  schedule.addTask(&UI_static5);
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  schedule.addTask(&UI_dynamic1);
#else
  schedule.addTask(&UI_dynamic_status);
#endif
  schedule.addTask(&UI_dynamic2);
  schedule.addTask(&UI_static3);
  schedule.addTask(&UI_dynamic3);
  schedule.addTask(&UI_dynamic4);
  schedule.addTask(&UI_dynamic_aimbot_box);
}

inline void static1_func() {
  static rm::device::UICharacter static_1;
  if (globals->ui_refresh_key) {
    static_1.character.fillCharacter("py", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow, 3,
                                     static_cast<u16>(8.728), static_cast<u16>(738.454), 30, 5);
    memcpy(static_1.data, "P:\nY:", 5);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_1, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

inline void static2_func() {
  static rm::device::UICharacter static_2;
  if (globals->ui_refresh_key) {
    static_2.character.fillCharacter("leg", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                     static_cast<u16>(1748), static_cast<u16>(748), 30, 5);
    memcpy(static_2.data, "L M H", 10);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_2, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

inline void dynamic3_crosshair_func() {
  static rm::device::UIFigure2 crosshair;
  static bool added = false;

  constexpr float kPitchMin = 0.f;
  constexpr float kPitchMax = 0.3f;
  constexpr u16 kEndYAtMinPitch = 290;
  constexpr u16 kEndYAtMaxPitch = 480;

  float pitch = ui_snapshot.gimbal_pitch_rad;
  if (pitch < kPitchMin) pitch = kPitchMin;
  if (pitch > kPitchMax) pitch = kPitchMax;
  const u16 end_y = static_cast<u16>(kEndYAtMinPitch + (pitch - kPitchMin) / (kPitchMax - kPitchMin) *
                                                           (kEndYAtMaxPitch - kEndYAtMinPitch));

  if (globals->ui_refresh_key) {
    crosshair.figure1.fillLine("ll", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Cyan, 5, 547, 86,
                               732, end_y);
    crosshair.figure2.fillLine("rl", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Cyan, 5, 1378, 86,
                               1170, end_y);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, crosshair, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
    added = true;
  } else if (added) {
    crosshair.figure1.fillLine("ll", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Cyan, 5, 547, 86,
                               732, end_y);
    crosshair.figure2.fillLine("rl", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Cyan, 5, 1378, 86,
                               1170, end_y);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, crosshair, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

inline void static4_func() {
  static rm::device::UIFigure1 static_4;
  if (globals->ui_refresh_key) {
    static_4.fillRec("r2_", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow, 3, 598, 836, 1315,
                     870);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_4, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

inline void dynamic_aimbot_box_func() {
  static rm::device::UIFigure1 box;
  static bool added = false;

  const bool has_target = globals->aimbot.has_value() && globals->aimbot->aimbot_target() == 1;
  const auto color = has_target ? device::UIFigure1::Color::Green : device::UIFigure1::Color::White;

  if (globals->ui_refresh_key) {
    box.fillRec("r1_", device::UIFigure1::Operation::Add, 0, color, 3, 781, 361, 1129, 709);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, box, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
    added = true;
  } else if (added) {
    box.fillRec("r1_", device::UIFigure1::Operation::Edit, 0, color, 3, 781, 361, 1129, 709);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, box, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

inline void static5_func() {
  static rm::device::UICharacter static_5;
  if (globals->ui_refresh_key) {
    static_5.character.fillCharacter("fric", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow, 3,
                                     1600, 562, 30, 5);
    // memcpy(static_5.data, "FRIC:", 5);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_5, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

inline void static_status_func() {
  static rm::device::UICharacter status_line;
  if (!globals->ui_refresh_key) return;

  const u8 sender = robot_id();
  static u8 status_line_index = 0;

  memset(status_line.data, 0, sizeof(status_line.data));
  switch (status_line_index) {
    case 0:
      status_line.character.fillCharacter("st1", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow,
                                          2, 10, 748, 20, 22);
      memcpy(status_line.data, "DISABLE STANDBY ENABLE", 22);
      break;
    case 1:
      status_line.character.fillCharacter("st2", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow,
                                          2, 10, 708, 20, 10);
      memcpy(status_line.data, "SPIN CROSS", 10);
      break;
    default:
      status_line.character.fillCharacter("st3", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow,
                                          2, 10, 668, 20, 16);
      memcpy(status_line.data, "NORMAL SMALL BIG", 16);
      break;
  }

  u8 len = rm::device::Referee0x301Prepare(info, 0, status_line, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(info, len, 10);
  status_line_index = (status_line_index + 1) % 3;
}

inline void dynamic1_func() {
  static rm::device::UIFigure2 static_d1;
  static bool added = false;
  if (globals->ui_refresh_key) {
    static_d1.figure1.fillFloat("pit", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                static_cast<u16>(77.748), static_cast<u16>(730.583), 25,
                                ui_snapshot.gimbal_pitch_rad * 1000);
    static_d1.figure2.fillFloat("yaw", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                static_cast<u16>(77.348), static_cast<u16>(690.988), 25,
                                ui_snapshot.gimbal_yaw_rad * 1000);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d1, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
    added = true;
  } else if (added) {
    static_d1.figure1.fillFloat("pit", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                static_cast<u16>(77.748), static_cast<u16>(730.583), 25,
                                ui_snapshot.gimbal_pitch_rad * 1000);
    static_d1.figure2.fillFloat("yaw", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                static_cast<u16>(77.348), static_cast<u16>(690.988), 25,
                                ui_snapshot.gimbal_yaw_rad * 1000);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d1, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}

inline void dynamic2_func() {
  static rm::device::UIFigure7 static_d2;
  static bool added = false;

  u16 rec_x_start = 1735, rec_x_end = 1785;
  {
    switch (static_cast<chassis::Fsm::State>(ui_snapshot.chassis_fsm_state)) {
      case chassis::Fsm::State::kMidLeg:
        rec_x_start = 1785;
        rec_x_end = 1845;
        break;
      case chassis::Fsm::State::kHighLeg:
        rec_x_start = 1850;
        rec_x_end = 1910;
        break;
      default:
        break;
    }
  }

  int cap_len = 718 * ui_snapshot.supercap_cap_energy / 255;
  // int cap_len = 718;
  rm::device::UIFigure1::Color cap_color;
  if (ui_snapshot.supercap_cap_energy / 255 < 0.4f) {
    cap_color = device::UIFigure1::Color::Pink;
  } else {
    cap_color = device::UIFigure1::Color::Green;
  }

  u16 lb_x1, lb_y1, lb_x2 = 660, lb_y2 = 766;
  u16 lm_x1, lm_y1, lm_x2, lm_y2;
  u16 rb_x1 = 1235, rb_y1 = 766, rb_x2, rb_y2;
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

  if (globals->ui_refresh_key) {
    static_d2.figure1.fillLine("l1", device::UIFigure1::Operation::Add, 0, cap_color, 34, 598, 853, 598 + cap_len, 853);
    static_d2.figure2.fillLine("l2", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Cyan, 5, lb_x1,
                               lb_y1, lb_x2, lb_y2);
    static_d2.figure3.fillLine("l3", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Cyan, 5, lm_x1,
                               lm_y1, lm_x2, lm_y2);
    static_d2.figure4.fillLine("l4", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Cyan, 5, rb_x1,
                               rb_y1, rb_x2, rb_y2);
    static_d2.figure5.fillLine("l5", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Cyan, 5, rm_x1,
                               rm_y1, rm_x2, rm_y2);
    static_d2.figure6.fillArc("a1", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Yellow, 5, 957, 538,
                              s_ang, e_ang, 77, 77);
    static_d2.figure7.fillRec("r1", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::White, 2,
                              rec_x_start, 700, rec_x_end, 770);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d2, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 50);
    added = true;
  } else if (added) {
    static_d2.figure1.fillLine("l1", device::UIFigure1::Operation::Edit, 0, cap_color, 34, 598, 853, 598 + cap_len,
                               853);
    static_d2.figure2.fillLine("l2", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Cyan, 5, lb_x1,
                               lb_y1, lb_x2, lb_y2);
    static_d2.figure3.fillLine("l3", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Cyan, 5, lm_x1,
                               lm_y1, lm_x2, lm_y2);
    static_d2.figure4.fillLine("l4", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Cyan, 5, rb_x1,
                               rb_y1, rb_x2, rb_y2);
    static_d2.figure5.fillLine("l5", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Cyan, 5, rm_x1,
                               rm_y1, rm_x2, rm_y2);
    static_d2.figure6.fillArc("a1", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Yellow, 5, 957,
                              538, s_ang, e_ang, 77, 77);
    static_d2.figure7.fillRec("r1", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::White, 2,
                              rec_x_start, 700, rec_x_end, 770);
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d2, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 50);
  }
}

inline void dynamic_status_func() {
  static rm::device::UIFigure5 status_frame;
  static bool added = false;

  auto fill_hidden = [](rm::device::UIFigure1 &figure, const char *name, device::UIFigure1::Operation op) {
    figure.fillRec(name, op, 0, device::UIFigure1::Color::Black, 1, 0, 0, 1, 1);
  };

  auto fill_rect = [](rm::device::UIFigure1 &figure, const char *name, device::UIFigure1::Operation op, u16 x1, u16 y1,
                      u16 x2,
                      u16 y2) { figure.fillRec(name, op, 0, device::UIFigure1::Color::White, 2, x1, y1, x2, y2); };

  const auto op = globals->ui_refresh_key ? device::UIFigure1::Operation::Add : device::UIFigure1::Operation::Edit;
  const bool disabled = ui_snapshot.domain_request == static_cast<u8>(wheel_legged::DomainRequest::kDisabled) ||
                        ui_snapshot.chassis_fsm_state == static_cast<u8>(chassis::Fsm::State::kDisabled);
  const bool enabled = !disabled && !ui_snapshot.standby;

  if (disabled) {
    fill_rect(status_frame.figure1, "ps_", op, 6, 718, 142, 758);
  } else if (ui_snapshot.standby) {
    fill_rect(status_frame.figure1, "ps_", op, 150, 718, 318, 758);
  } else if (enabled) {
    fill_rect(status_frame.figure1, "ps_", op, 328, 718, 470, 758);
  } else {
    fill_hidden(status_frame.figure1, "ps_", op);
  }

  if (ui_snapshot.spin_active) {
    fill_rect(status_frame.figure2, "mv_", op, 6, 678, 102, 718);
  } else if (ui_snapshot.cross_active) {
    fill_rect(status_frame.figure2, "mv_", op, 112, 678, 235, 718);
  } else {
    fill_hidden(status_frame.figure2, "mv_", op);
  }

  switch (static_cast<wheel_legged::CombatProfile>(ui_snapshot.combat_profile)) {
    case wheel_legged::CombatProfile::kAutoAimAmmo:
      fill_rect(status_frame.figure3, "am_", op, 6, 638, 158, 678);
      break;
    case wheel_legged::CombatProfile::kAutoAimFuSmall:
      fill_rect(status_frame.figure3, "am_", op, 166, 638, 296, 678);
      break;
    case wheel_legged::CombatProfile::kAutoAimFuBig:
      fill_rect(status_frame.figure3, "am_", op, 304, 638, 390, 678);
      break;
    case wheel_legged::CombatProfile::kNormal:
    default:
      fill_hidden(status_frame.figure3, "am_", op);
      break;
  }

  fill_hidden(status_frame.figure4, "u4_", op);
  fill_hidden(status_frame.figure5, "u5_", op);

  if (globals->ui_refresh_key || added) {
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, status_frame, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
    added = true;
  }
}

inline void dynamic3_func() {
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  static rm::device::UIFigure5 static_d3;
  static bool added = false;
  if (globals->ui_refresh_key) {
    static_d3.figure1.fillIntegrate("f1_", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 722, 20, static_cast<i32>(ui_snapshot.fw_raw_rpm_1));
    static_d3.figure2.fillIntegrate("f2_", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 762, 20, static_cast<i32>(ui_snapshot.fw_raw_rpm_2));
    static_d3.figure3.fillIntegrate("f3_", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 802, 20, static_cast<i32>(ui_snapshot.fw_raw_rpm_3));
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d3, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 50);
    added = true;
  } else if (added) {
    static_d3.figure1.fillIntegrate("f1_", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 722, 20, static_cast<i32>(ui_snapshot.fw_raw_rpm_1));
    static_d3.figure2.fillIntegrate("f2_", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 762, 20, static_cast<i32>(ui_snapshot.fw_raw_rpm_2));
    static_d3.figure3.fillIntegrate("f3_", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 802, 20, static_cast<i32>(ui_snapshot.fw_raw_rpm_3));
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d3, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 50);
  }
#else
  static rm::device::UIFigure2 static_d3;
  static bool added = false;
  if (globals->ui_refresh_key) {
    static_d3.figure1.fillIntegrate("fL_", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 646, 20, static_cast<i32>(ui_snapshot.fric_left_rpm));
    static_d3.figure2.fillIntegrate("fR_", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 676, 20, static_cast<i32>(ui_snapshot.fric_right_rpm));
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d3, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
    added = true;
  } else if (added) {
    static_d3.figure1.fillIntegrate("fL_", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 646, 20, static_cast<i32>(ui_snapshot.fric_left_rpm));
    static_d3.figure2.fillIntegrate("fR_", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                    1750, 676, 20, static_cast<i32>(ui_snapshot.fric_right_rpm));
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d3, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
#endif
}

inline void dynamic4_func() {
  static rm::device::UIFigure2 static_d4;
  static bool added = false;
  if (globals->ui_refresh_key) {
    static_d4.figure1.fillFloat("spd", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                static_cast<u16>(1343.216), static_cast<u16>(644.661), 25,
                                ui_snapshot.bullet_speed_mps);
    static_d4.figure2.fillIntegrate("amm", device::UIFigure1::Operation::Add, 0, device::UIFigure1::Color::Green, 3,
                                    static_cast<u16>(1346.347), static_cast<u16>(472.9), 25,
                                    static_cast<i32>(ui_snapshot.projectile_allowance));
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d4, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
    added = true;
  } else if (added) {
    static_d4.figure1.fillFloat("spd", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                static_cast<u16>(1343.216), static_cast<u16>(644.661), 25,
                                ui_snapshot.bullet_speed_mps);
    static_d4.figure2.fillIntegrate("amm", device::UIFigure1::Operation::Edit, 0, device::UIFigure1::Color::Green, 3,
                                    static_cast<u16>(1346.347), static_cast<u16>(472.9), 25,
                                    static_cast<i32>(ui_snapshot.projectile_allowance));
    u8 sender = robot_id();
    u8 len = rm::device::Referee0x301Prepare(info, 0, static_d4, sender, static_cast<u16>(sender) + 256);
    globals_no_dtcm.referee_uart.Write(info, len, 10);
  }
}
