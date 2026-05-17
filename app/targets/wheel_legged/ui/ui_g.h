//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_number_t *ui_g_Dynamic_Bult_Amount_Num;
extern ui_interface_number_t *ui_g_Dynamic_Fric_Rpm;
extern ui_interface_number_t *ui_g_Dynamic_Offset_Pitch_Num;
extern ui_interface_number_t *ui_g_Dynamic_Distance_Num;
extern ui_interface_number_t *ui_g_Dynamic_Bult_Spd_Num;

void ui_init_g_Dynamic();
void ui_update_g_Dynamic();
void ui_remove_g_Dynamic();

extern ui_interface_number_t *ui_g_Dynamic2_Leg_length;
extern ui_interface_number_t *ui_g_Dynamic2_Chassis_State;
extern ui_interface_number_t *ui_g_Dynamic2_Gimbal_State;
extern ui_interface_number_t *ui_g_Dynamic2_Ammo_State;
extern ui_interface_line_t *ui_g_Dynamic2_HealthyPower_Line;
extern ui_interface_line_t *ui_g_Dynamic2_UnhealthyPowre_Line;
extern ui_interface_number_t *ui_g_Dynamic2_Offset_Yaw_Num;

void ui_init_g_Dynamic2();
void ui_update_g_Dynamic2();
void ui_remove_g_Dynamic2();

extern ui_interface_line_t *ui_g_Dynamic3_left_leg_up;
extern ui_interface_line_t *ui_g_Dynamic3_left_leg_down;
extern ui_interface_line_t *ui_g_Dynamic3_right_leg_up;
extern ui_interface_line_t *ui_g_Dynamic3_right_leg_down;
extern ui_interface_arc_t *ui_g_Dynamic3_Chassis_Position;

void ui_init_g_Dynamic3();
void ui_update_g_Dynamic3();
void ui_remove_g_Dynamic3();

extern ui_interface_string_t *ui_g_Static_Fric;
extern ui_interface_string_t *ui_g_Static_Offset_Pitch;
extern ui_interface_string_t *ui_g_Static_Offset_Yaw;
extern ui_interface_string_t *ui_g_Static_Distance;

void ui_init_g_Static();
void ui_update_g_Static();
void ui_remove_g_Static();

extern ui_interface_rect_t *ui_g_Static2_Power;
extern ui_interface_string_t *ui_g_Static2_Leg_length;
extern ui_interface_string_t *ui_g_Static2_Chassis;
extern ui_interface_string_t *ui_g_Static2_Gimbal;
extern ui_interface_string_t *ui_g_Static2_Ammo;

void ui_init_g_Static2();
void ui_update_g_Static2();
void ui_remove_g_Static2();

extern ui_interface_line_t *ui_g_Static3_left_car_line;
extern ui_interface_line_t *ui_g_Static3_right_car_line;

void ui_init_g_Static3();
void ui_update_g_Static3();
void ui_remove_g_Static3();


#endif // UI_g_H
