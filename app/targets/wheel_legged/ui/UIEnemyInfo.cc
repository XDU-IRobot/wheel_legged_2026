//
// Created by refactored from ui.hpp — drone_gb_new style
//

#include "UIEnemyInfo.hpp"

#include "librm.hpp"
#include "protocol_user.hpp"
#include "referee_user.hpp"
#include "ui_snapshot.hpp"
#include "targets/wheel_legged/include/globals_no_dtcm.hpp"

using namespace rm;
using namespace rm::device;

extern u8 dataBox[256];
extern SharedResourcesNoDtcm globals_no_dtcm;

static bool is_red_team() { return ui_snapshot.referee_robot_id > 0 && ui_snapshot.referee_robot_id <= 100; }

// ═══════════════════════════════════════════════════════════════════════════
// Red team — enemy robot HP header
// ═══════════════════════════════════════════════════════════════════════════

void UIEnemyHeaderRed_add() {
  if (!is_red_team()) return;
  static UICharacter hp_header;
  hp_header.character.fillCharacter("Hed", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 6, 1170, 890, 24, 30);
  memcpy(hp_header.data, "HRO1 ENG2 STD3 STD4 DRO6 SEN7", 30);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, hp_header, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Red team — enemy robot HP values
// ═══════════════════════════════════════════════════════════════════════════

void UIEnemyHPRed_add() {
  if (!is_red_team()) return;
  static UIFigure5 fig;
  fig.figure1.fillIntegrate("HP1", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1170, 850, 20,
                            static_cast<i32>(ui_snapshot.hero_1_HP));
  fig.figure2.fillIntegrate("HP2", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1290, 850, 20,
                            static_cast<i32>(ui_snapshot.engineer_2_HP));
  fig.figure3.fillIntegrate("HP3", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1410, 850, 20,
                            static_cast<i32>(ui_snapshot.standard_3_HP));
  fig.figure4.fillIntegrate("HP4", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1530, 850, 20,
                            static_cast<i32>(ui_snapshot.standard_4_HP));
  fig.figure5.fillIntegrate("HP5", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 1770, 850, 20,
                            static_cast<i32>(ui_snapshot.sentry_7_HP));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIEnemyHPRed_edit() {
  if (!is_red_team()) return;
  static UIFigure5 fig;
  const auto hp1_color = (ui_snapshot.enemy_hero_1_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  const auto hp2_color = (ui_snapshot.enemy_engineer_2_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  const auto hp3_color = (ui_snapshot.enemy_standard_3_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  const auto hp4_color = (ui_snapshot.enemy_standard_4_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  const auto hp5_color = (ui_snapshot.enemy_sentry_7_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  fig.figure1.fillIntegrate("HP1", UIFigure::Operation::Edit, 0, hp1_color, 4, 1170, 850, 20,
                            static_cast<i32>(ui_snapshot.hero_1_HP));
  fig.figure2.fillIntegrate("HP2", UIFigure::Operation::Edit, 0, hp2_color, 4, 1290, 850, 20,
                            static_cast<i32>(ui_snapshot.engineer_2_HP));
  fig.figure3.fillIntegrate("HP3", UIFigure::Operation::Edit, 0, hp3_color, 4, 1410, 850, 20,
                            static_cast<i32>(ui_snapshot.standard_3_HP));
  fig.figure4.fillIntegrate("HP4", UIFigure::Operation::Edit, 0, hp4_color, 4, 1530, 850, 20,
                            static_cast<i32>(ui_snapshot.standard_4_HP));
  fig.figure5.fillIntegrate("HP5", UIFigure::Operation::Edit, 0, hp5_color, 4, 1770, 850, 20,
                            static_cast<i32>(ui_snapshot.sentry_7_HP));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Red team — enemy projectile allowance
// ═══════════════════════════════════════════════════════════════════════════

void UIEnemyAllowanceRed_add() {
  if (!is_red_team()) return;
  static UIFigure5 fig;
  fig.figure1.fillIntegrate("AL1", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1170, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_hero_1_allowance));
  fig.figure2.fillIntegrate("AL2", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1410, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_standard_3_allowance));
  fig.figure3.fillIntegrate("AL3", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1530, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_standard_4_allowance));
  fig.figure4.fillIntegrate("AL4", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1650, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_drone_6_allowance));
  fig.figure5.fillIntegrate("AL5", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 1770, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_sentry_7_allowance));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIEnemyAllowanceRed_edit() {
  if (!is_red_team()) return;
  static UIFigure5 fig;
  fig.figure1.fillIntegrate("AL1", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1170, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_hero_1_allowance));
  fig.figure2.fillIntegrate("AL2", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1410, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_standard_3_allowance));
  fig.figure3.fillIntegrate("AL3", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1530, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_standard_4_allowance));
  fig.figure4.fillIntegrate("AL4", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1650, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_drone_6_allowance));
  fig.figure5.fillIntegrate("AL5", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 1770, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_sentry_7_allowance));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Blue team — enemy robot HP header
// ═══════════════════════════════════════════════════════════════════════════

void UIEnemyHeaderBlue_add() {
  if (is_red_team()) return;
  static UICharacter hp_header;
  hp_header.character.fillCharacter("Bhd", UIFigure::Operation::Add, 0, UIFigure::Color::Orange, 6, 55, 890, 24, 29);
  memcpy(hp_header.data, "SEN7 DRO6 STD4 STD3 ENG2 HRO1", 29);
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, hp_header, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Blue team — enemy robot HP values
// ═══════════════════════════════════════════════════════════════════════════

void UIEnemyHPBlue_add() {
  if (is_red_team()) return;
  static UIFigure5 fig;
  fig.figure1.fillIntegrate("BH1", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 55, 850, 20,
                            static_cast<i32>(ui_snapshot.sentry_7_HP));
  fig.figure2.fillIntegrate("BH2", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 295, 850, 20,
                            static_cast<i32>(ui_snapshot.standard_4_HP));
  fig.figure3.fillIntegrate("BH3", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 415, 850, 20,
                            static_cast<i32>(ui_snapshot.standard_3_HP));
  fig.figure4.fillIntegrate("BH4", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 535, 850, 20,
                            static_cast<i32>(ui_snapshot.engineer_2_HP));
  fig.figure5.fillIntegrate("BH5", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 655, 850, 20,
                            static_cast<i32>(ui_snapshot.hero_1_HP));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIEnemyHPBlue_edit() {
  if (is_red_team()) return;
  static UIFigure5 fig;
  const auto bh1_color = (ui_snapshot.enemy_sentry_7_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  const auto bh2_color = (ui_snapshot.enemy_standard_4_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  const auto bh3_color = (ui_snapshot.enemy_standard_3_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  const auto bh4_color = (ui_snapshot.enemy_engineer_2_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  const auto bh5_color = (ui_snapshot.enemy_hero_1_defense >= 100) ? UIFigure::Color::Yellow : UIFigure::Color::RedBlue;
  fig.figure1.fillIntegrate("BH1", UIFigure::Operation::Edit, 0, bh1_color, 4, 55, 850, 20,
                            static_cast<i32>(ui_snapshot.sentry_7_HP));
  fig.figure2.fillIntegrate("BH2", UIFigure::Operation::Edit, 0, bh2_color, 4, 295, 850, 20,
                            static_cast<i32>(ui_snapshot.standard_4_HP));
  fig.figure3.fillIntegrate("BH3", UIFigure::Operation::Edit, 0, bh3_color, 4, 415, 850, 20,
                            static_cast<i32>(ui_snapshot.standard_3_HP));
  fig.figure4.fillIntegrate("BH4", UIFigure::Operation::Edit, 0, bh4_color, 4, 535, 850, 20,
                            static_cast<i32>(ui_snapshot.engineer_2_HP));
  fig.figure5.fillIntegrate("BH5", UIFigure::Operation::Edit, 0, bh5_color, 4, 655, 850, 20,
                            static_cast<i32>(ui_snapshot.hero_1_HP));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Blue team — enemy projectile allowance
// ═══════════════════════════════════════════════════════════════════════════

void UIEnemyAllowanceBlue_add() {
  if (is_red_team()) return;
  static UIFigure5 fig;
  fig.figure1.fillIntegrate("BA1", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 55, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_sentry_7_allowance));
  fig.figure2.fillIntegrate("BA2", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 175, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_drone_6_allowance));
  fig.figure3.fillIntegrate("BA3", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 295, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_standard_4_allowance));
  fig.figure4.fillIntegrate("BA4", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 415, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_standard_3_allowance));
  fig.figure5.fillIntegrate("BA5", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 655, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_hero_1_allowance));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIEnemyAllowanceBlue_edit() {
  if (is_red_team()) return;
  static UIFigure5 fig;
  fig.figure1.fillIntegrate("BA1", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 55, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_sentry_7_allowance));
  fig.figure2.fillIntegrate("BA2", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 175, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_drone_6_allowance));
  fig.figure3.fillIntegrate("BA3", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 295, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_standard_4_allowance));
  fig.figure4.fillIntegrate("BA4", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 415, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_standard_3_allowance));
  fig.figure5.fillIntegrate("BA5", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 655, 810, 16,
                            static_cast<i32>(ui_snapshot.enemy_hero_1_allowance));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

// ═══════════════════════════════════════════════════════════════════════════
// Gold coin (both teams)
// ═══════════════════════════════════════════════════════════════════════════

void UIGoldCoin_add() {
  static UIFigure2 fig;
  fig.figure1.fillIntegrate("sco", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 4, 988, 900, 18,
                            static_cast<i32>(ui_snapshot.enemy_gold_total));
  fig.figure2.fillIntegrate("cco", UIFigure::Operation::Add, 0, UIFigure::Color::White, 4, 988, 865, 24,
                            static_cast<i32>(ui_snapshot.enemy_gold_remaining));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}

void UIGoldCoin_edit() {
  static UIFigure2 fig;
  fig.figure1.fillIntegrate("sco", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 4, 988, 900, 18,
                            static_cast<i32>(ui_snapshot.enemy_gold_total));
  fig.figure2.fillIntegrate("cco", UIFigure::Operation::Edit, 0, UIFigure::Color::White, 4, 988, 865, 24,
                            static_cast<i32>(ui_snapshot.enemy_gold_remaining));
  u8 sender = ui_snapshot.referee_robot_id;
  u8 len = Referee0x301Prepare(dataBox, 0, fig, sender, static_cast<u16>(sender) + 256);
  globals_no_dtcm.referee_uart.Write(dataBox, len, 10);
}
