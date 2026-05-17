//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_g_Dynamic_0;

ui_interface_number_t *ui_g_Dynamic_Bult_Amount_Num = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[0]);
ui_interface_number_t *ui_g_Dynamic_Fric_Rpm = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[1]);
ui_interface_number_t *ui_g_Dynamic_Offset_Pitch_Num = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[2]);
ui_interface_number_t *ui_g_Dynamic_Distance_Num = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[3]);
ui_interface_number_t *ui_g_Dynamic_Bult_Spd_Num = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[4]);

void _ui_init_g_Dynamic_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Dynamic_0.data[i].figure_name[0] = 0;
        ui_g_Dynamic_0.data[i].figure_name[1] = 0;
        ui_g_Dynamic_0.data[i].figure_name[2] = i + 0;
        ui_g_Dynamic_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 0;
    }

    ui_g_Dynamic_Bult_Amount_Num->figure_type = 6;
    ui_g_Dynamic_Bult_Amount_Num->operate_type = 1;
    ui_g_Dynamic_Bult_Amount_Num->layer = 0;
    ui_g_Dynamic_Bult_Amount_Num->color = 2;
    ui_g_Dynamic_Bult_Amount_Num->start_x = 1346;
    ui_g_Dynamic_Bult_Amount_Num->start_y = 473;
    ui_g_Dynamic_Bult_Amount_Num->width = 3;
    ui_g_Dynamic_Bult_Amount_Num->font_size = 25;
    ui_g_Dynamic_Bult_Amount_Num->number = 12345;

    ui_g_Dynamic_Fric_Rpm->figure_type = 6;
    ui_g_Dynamic_Fric_Rpm->operate_type = 1;
    ui_g_Dynamic_Fric_Rpm->layer = 0;
    ui_g_Dynamic_Fric_Rpm->color = 2;
    ui_g_Dynamic_Fric_Rpm->start_x = 155;
    ui_g_Dynamic_Fric_Rpm->start_y = 802;
    ui_g_Dynamic_Fric_Rpm->width = 3;
    ui_g_Dynamic_Fric_Rpm->font_size = 25;
    ui_g_Dynamic_Fric_Rpm->number = 12345;

    ui_g_Dynamic_Offset_Pitch_Num->figure_type = 6;
    ui_g_Dynamic_Offset_Pitch_Num->operate_type = 1;
    ui_g_Dynamic_Offset_Pitch_Num->layer = 0;
    ui_g_Dynamic_Offset_Pitch_Num->color = 2;
    ui_g_Dynamic_Offset_Pitch_Num->start_x = 78;
    ui_g_Dynamic_Offset_Pitch_Num->start_y = 743;
    ui_g_Dynamic_Offset_Pitch_Num->width = 3;
    ui_g_Dynamic_Offset_Pitch_Num->font_size = 25;
    ui_g_Dynamic_Offset_Pitch_Num->number = 12345;

    ui_g_Dynamic_Distance_Num->figure_type = 6;
    ui_g_Dynamic_Distance_Num->operate_type = 1;
    ui_g_Dynamic_Distance_Num->layer = 0;
    ui_g_Dynamic_Distance_Num->color = 2;
    ui_g_Dynamic_Distance_Num->start_x = 930;
    ui_g_Dynamic_Distance_Num->start_y = 237;
    ui_g_Dynamic_Distance_Num->width = 3;
    ui_g_Dynamic_Distance_Num->font_size = 25;
    ui_g_Dynamic_Distance_Num->number = 12345;

    ui_g_Dynamic_Bult_Spd_Num->figure_type = 6;
    ui_g_Dynamic_Bult_Spd_Num->operate_type = 1;
    ui_g_Dynamic_Bult_Spd_Num->layer = 0;
    ui_g_Dynamic_Bult_Spd_Num->color = 2;
    ui_g_Dynamic_Bult_Spd_Num->start_x = 1343;
    ui_g_Dynamic_Bult_Spd_Num->start_y = 615;
    ui_g_Dynamic_Bult_Spd_Num->width = 3;
    ui_g_Dynamic_Bult_Spd_Num->font_size = 25;
    ui_g_Dynamic_Bult_Spd_Num->number = 12345;


    ui_proc_5_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}

void _ui_update_g_Dynamic_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}

void _ui_remove_g_Dynamic_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}


void ui_init_g_Dynamic() {
    _ui_init_g_Dynamic_0();
}

void ui_update_g_Dynamic() {
    _ui_update_g_Dynamic_0();
}

void ui_remove_g_Dynamic() {
    _ui_remove_g_Dynamic_0();
}

ui_7_frame_t ui_g_Dynamic2_0;

ui_interface_number_t *ui_g_Dynamic2_Leg_length = (ui_interface_number_t*)&(ui_g_Dynamic2_0.data[0]);
ui_interface_number_t *ui_g_Dynamic2_Chassis_State = (ui_interface_number_t*)&(ui_g_Dynamic2_0.data[1]);
ui_interface_number_t *ui_g_Dynamic2_Gimbal_State = (ui_interface_number_t*)&(ui_g_Dynamic2_0.data[2]);
ui_interface_number_t *ui_g_Dynamic2_Ammo_State = (ui_interface_number_t*)&(ui_g_Dynamic2_0.data[3]);
ui_interface_line_t *ui_g_Dynamic2_HealthyPower_Line = (ui_interface_line_t*)&(ui_g_Dynamic2_0.data[4]);
ui_interface_line_t *ui_g_Dynamic2_UnhealthyPowre_Line = (ui_interface_line_t*)&(ui_g_Dynamic2_0.data[5]);
ui_interface_number_t *ui_g_Dynamic2_Offset_Yaw_Num = (ui_interface_number_t*)&(ui_g_Dynamic2_0.data[6]);

void _ui_init_g_Dynamic2_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic2_0.data[i].figure_name[0] = 0;
        ui_g_Dynamic2_0.data[i].figure_name[1] = 1;
        ui_g_Dynamic2_0.data[i].figure_name[2] = i + 0;
        ui_g_Dynamic2_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Dynamic2_0.data[i].operate_type = 0;
    }

    ui_g_Dynamic2_Leg_length->figure_type = 6;
    ui_g_Dynamic2_Leg_length->operate_type = 1;
    ui_g_Dynamic2_Leg_length->layer = 0;
    ui_g_Dynamic2_Leg_length->color = 6;
    ui_g_Dynamic2_Leg_length->start_x = 148;
    ui_g_Dynamic2_Leg_length->start_y = 503;
    ui_g_Dynamic2_Leg_length->width = 2;
    ui_g_Dynamic2_Leg_length->font_size = 24;
    ui_g_Dynamic2_Leg_length->number = 1;

    ui_g_Dynamic2_Chassis_State->figure_type = 6;
    ui_g_Dynamic2_Chassis_State->operate_type = 1;
    ui_g_Dynamic2_Chassis_State->layer = 0;
    ui_g_Dynamic2_Chassis_State->color = 6;
    ui_g_Dynamic2_Chassis_State->start_x = 210;
    ui_g_Dynamic2_Chassis_State->start_y = 435;
    ui_g_Dynamic2_Chassis_State->width = 2;
    ui_g_Dynamic2_Chassis_State->font_size = 24;
    ui_g_Dynamic2_Chassis_State->number = 1;

    ui_g_Dynamic2_Gimbal_State->figure_type = 6;
    ui_g_Dynamic2_Gimbal_State->operate_type = 1;
    ui_g_Dynamic2_Gimbal_State->layer = 0;
    ui_g_Dynamic2_Gimbal_State->color = 6;
    ui_g_Dynamic2_Gimbal_State->start_x = 199;
    ui_g_Dynamic2_Gimbal_State->start_y = 378;
    ui_g_Dynamic2_Gimbal_State->width = 2;
    ui_g_Dynamic2_Gimbal_State->font_size = 24;
    ui_g_Dynamic2_Gimbal_State->number = 1;

    ui_g_Dynamic2_Ammo_State->figure_type = 6;
    ui_g_Dynamic2_Ammo_State->operate_type = 1;
    ui_g_Dynamic2_Ammo_State->layer = 0;
    ui_g_Dynamic2_Ammo_State->color = 6;
    ui_g_Dynamic2_Ammo_State->start_x = 150;
    ui_g_Dynamic2_Ammo_State->start_y = 317;
    ui_g_Dynamic2_Ammo_State->width = 2;
    ui_g_Dynamic2_Ammo_State->font_size = 24;
    ui_g_Dynamic2_Ammo_State->number = 1;

    ui_g_Dynamic2_HealthyPower_Line->figure_type = 0;
    ui_g_Dynamic2_HealthyPower_Line->operate_type = 1;
    ui_g_Dynamic2_HealthyPower_Line->layer = 0;
    ui_g_Dynamic2_HealthyPower_Line->color = 2;
    ui_g_Dynamic2_HealthyPower_Line->start_x = 946;
    ui_g_Dynamic2_HealthyPower_Line->start_y = 836;
    ui_g_Dynamic2_HealthyPower_Line->width = 34;
    ui_g_Dynamic2_HealthyPower_Line->end_x = 1317;
    ui_g_Dynamic2_HealthyPower_Line->end_y = 836;

    ui_g_Dynamic2_UnhealthyPowre_Line->figure_type = 0;
    ui_g_Dynamic2_UnhealthyPowre_Line->operate_type = 1;
    ui_g_Dynamic2_UnhealthyPowre_Line->layer = 0;
    ui_g_Dynamic2_UnhealthyPowre_Line->color = 5;
    ui_g_Dynamic2_UnhealthyPowre_Line->start_x = 586;
    ui_g_Dynamic2_UnhealthyPowre_Line->start_y = 835;
    ui_g_Dynamic2_UnhealthyPowre_Line->width = 34;
    ui_g_Dynamic2_UnhealthyPowre_Line->end_x = 927;
    ui_g_Dynamic2_UnhealthyPowre_Line->end_y = 833;

    ui_g_Dynamic2_Offset_Yaw_Num->figure_type = 6;
    ui_g_Dynamic2_Offset_Yaw_Num->operate_type = 1;
    ui_g_Dynamic2_Offset_Yaw_Num->layer = 0;
    ui_g_Dynamic2_Offset_Yaw_Num->color = 2;
    ui_g_Dynamic2_Offset_Yaw_Num->start_x = 80;
    ui_g_Dynamic2_Offset_Yaw_Num->start_y = 676;
    ui_g_Dynamic2_Offset_Yaw_Num->width = 3;
    ui_g_Dynamic2_Offset_Yaw_Num->font_size = 25;
    ui_g_Dynamic2_Offset_Yaw_Num->number = 12345;


    ui_proc_7_frame(&ui_g_Dynamic2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic2_0, sizeof(ui_g_Dynamic2_0));
}

void _ui_update_g_Dynamic2_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic2_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Dynamic2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic2_0, sizeof(ui_g_Dynamic2_0));
}

void _ui_remove_g_Dynamic2_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic2_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Dynamic2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic2_0, sizeof(ui_g_Dynamic2_0));
}


void ui_init_g_Dynamic2() {
    _ui_init_g_Dynamic2_0();
}

void ui_update_g_Dynamic2() {
    _ui_update_g_Dynamic2_0();
}

void ui_remove_g_Dynamic2() {
    _ui_remove_g_Dynamic2_0();
}

ui_5_frame_t ui_g_Dynamic3_0;

ui_interface_line_t *ui_g_Dynamic3_left_leg_up = (ui_interface_line_t*)&(ui_g_Dynamic3_0.data[0]);
ui_interface_line_t *ui_g_Dynamic3_left_leg_down = (ui_interface_line_t*)&(ui_g_Dynamic3_0.data[1]);
ui_interface_line_t *ui_g_Dynamic3_right_leg_up = (ui_interface_line_t*)&(ui_g_Dynamic3_0.data[2]);
ui_interface_line_t *ui_g_Dynamic3_right_leg_down = (ui_interface_line_t*)&(ui_g_Dynamic3_0.data[3]);
ui_interface_arc_t *ui_g_Dynamic3_Chassis_Position = (ui_interface_arc_t*)&(ui_g_Dynamic3_0.data[4]);

void _ui_init_g_Dynamic3_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Dynamic3_0.data[i].figure_name[0] = 0;
        ui_g_Dynamic3_0.data[i].figure_name[1] = 2;
        ui_g_Dynamic3_0.data[i].figure_name[2] = i + 0;
        ui_g_Dynamic3_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_g_Dynamic3_0.data[i].operate_type = 0;
    }

    ui_g_Dynamic3_left_leg_up->figure_type = 0;
    ui_g_Dynamic3_left_leg_up->operate_type = 1;
    ui_g_Dynamic3_left_leg_up->layer = 0;
    ui_g_Dynamic3_left_leg_up->color = 0;
    ui_g_Dynamic3_left_leg_up->start_x = 549;
    ui_g_Dynamic3_left_leg_up->start_y = 766;
    ui_g_Dynamic3_left_leg_up->width = 5;
    ui_g_Dynamic3_left_leg_up->end_x = 660;
    ui_g_Dynamic3_left_leg_up->end_y = 766;

    ui_g_Dynamic3_left_leg_down->figure_type = 0;
    ui_g_Dynamic3_left_leg_down->operate_type = 1;
    ui_g_Dynamic3_left_leg_down->layer = 0;
    ui_g_Dynamic3_left_leg_down->color = 0;
    ui_g_Dynamic3_left_leg_down->start_x = 549;
    ui_g_Dynamic3_left_leg_down->start_y = 763;
    ui_g_Dynamic3_left_leg_down->width = 5;
    ui_g_Dynamic3_left_leg_down->end_x = 649;
    ui_g_Dynamic3_left_leg_down->end_y = 663;

    ui_g_Dynamic3_right_leg_up->figure_type = 0;
    ui_g_Dynamic3_right_leg_up->operate_type = 1;
    ui_g_Dynamic3_right_leg_up->layer = 0;
    ui_g_Dynamic3_right_leg_up->color = 0;
    ui_g_Dynamic3_right_leg_up->start_x = 1235;
    ui_g_Dynamic3_right_leg_up->start_y = 759;
    ui_g_Dynamic3_right_leg_up->width = 5;
    ui_g_Dynamic3_right_leg_up->end_x = 1346;
    ui_g_Dynamic3_right_leg_up->end_y = 759;

    ui_g_Dynamic3_right_leg_down->figure_type = 0;
    ui_g_Dynamic3_right_leg_down->operate_type = 1;
    ui_g_Dynamic3_right_leg_down->layer = 0;
    ui_g_Dynamic3_right_leg_down->color = 0;
    ui_g_Dynamic3_right_leg_down->start_x = 1346;
    ui_g_Dynamic3_right_leg_down->start_y = 759;
    ui_g_Dynamic3_right_leg_down->width = 5;
    ui_g_Dynamic3_right_leg_down->end_x = 1235;
    ui_g_Dynamic3_right_leg_down->end_y = 659;

    ui_g_Dynamic3_Chassis_Position->figure_type = 4;
    ui_g_Dynamic3_Chassis_Position->operate_type = 1;
    ui_g_Dynamic3_Chassis_Position->layer = 0;
    ui_g_Dynamic3_Chassis_Position->color = 1;
    ui_g_Dynamic3_Chassis_Position->start_x = 957;
    ui_g_Dynamic3_Chassis_Position->start_y = 538;
    ui_g_Dynamic3_Chassis_Position->width = 5;
    ui_g_Dynamic3_Chassis_Position->start_angle = 315;
    ui_g_Dynamic3_Chassis_Position->end_angle = 45;
    ui_g_Dynamic3_Chassis_Position->rx = 77;
    ui_g_Dynamic3_Chassis_Position->ry = 77;


    ui_proc_5_frame(&ui_g_Dynamic3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic3_0, sizeof(ui_g_Dynamic3_0));
}

void _ui_update_g_Dynamic3_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Dynamic3_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Dynamic3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic3_0, sizeof(ui_g_Dynamic3_0));
}

void _ui_remove_g_Dynamic3_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Dynamic3_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Dynamic3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic3_0, sizeof(ui_g_Dynamic3_0));
}


void ui_init_g_Dynamic3() {
    _ui_init_g_Dynamic3_0();
}

void ui_update_g_Dynamic3() {
    _ui_update_g_Dynamic3_0();
}

void ui_remove_g_Dynamic3() {
    _ui_remove_g_Dynamic3_0();
}


ui_string_frame_t ui_g_Static_0;
ui_interface_string_t* ui_g_Static_Fric = &(ui_g_Static_0.option);

void _ui_init_g_Static_0() {
    ui_g_Static_0.option.figure_name[0] = 0;
    ui_g_Static_0.option.figure_name[1] = 3;
    ui_g_Static_0.option.figure_name[2] = 0;
    ui_g_Static_0.option.operate_type = 1;

    ui_g_Static_Fric->figure_type = 7;
    ui_g_Static_Fric->operate_type = 1;
    ui_g_Static_Fric->layer = 0;
    ui_g_Static_Fric->color = 1;
    ui_g_Static_Fric->start_x = 7;
    ui_g_Static_Fric->start_y = 802;
    ui_g_Static_Fric->width = 3;
    ui_g_Static_Fric->font_size = 25;
    ui_g_Static_Fric->str_length = 5;
    strcpy(ui_g_Static_Fric->string, "Fric:");


    ui_proc_string_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}

void _ui_update_g_Static_0() {
    ui_g_Static_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}

void _ui_remove_g_Static_0() {
    ui_g_Static_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}
ui_string_frame_t ui_g_Static_1;
ui_interface_string_t* ui_g_Static_Offset_Pitch = &(ui_g_Static_1.option);

void _ui_init_g_Static_1() {
    ui_g_Static_1.option.figure_name[0] = 0;
    ui_g_Static_1.option.figure_name[1] = 3;
    ui_g_Static_1.option.figure_name[2] = 1;
    ui_g_Static_1.option.operate_type = 1;

    ui_g_Static_Offset_Pitch->figure_type = 7;
    ui_g_Static_Offset_Pitch->operate_type = 1;
    ui_g_Static_Offset_Pitch->layer = 0;
    ui_g_Static_Offset_Pitch->color = 1;
    ui_g_Static_Offset_Pitch->start_x = 9;
    ui_g_Static_Offset_Pitch->start_y = 738;
    ui_g_Static_Offset_Pitch->width = 3;
    ui_g_Static_Offset_Pitch->font_size = 25;
    ui_g_Static_Offset_Pitch->str_length = 2;
    strcpy(ui_g_Static_Offset_Pitch->string, "P:");


    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}

void _ui_update_g_Static_1() {
    ui_g_Static_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}

void _ui_remove_g_Static_1() {
    ui_g_Static_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}
ui_string_frame_t ui_g_Static_2;
ui_interface_string_t* ui_g_Static_Offset_Yaw = &(ui_g_Static_2.option);

void _ui_init_g_Static_2() {
    ui_g_Static_2.option.figure_name[0] = 0;
    ui_g_Static_2.option.figure_name[1] = 3;
    ui_g_Static_2.option.figure_name[2] = 2;
    ui_g_Static_2.option.operate_type = 1;

    ui_g_Static_Offset_Yaw->figure_type = 7;
    ui_g_Static_Offset_Yaw->operate_type = 1;
    ui_g_Static_Offset_Yaw->layer = 0;
    ui_g_Static_Offset_Yaw->color = 1;
    ui_g_Static_Offset_Yaw->start_x = 9;
    ui_g_Static_Offset_Yaw->start_y = 672;
    ui_g_Static_Offset_Yaw->width = 3;
    ui_g_Static_Offset_Yaw->font_size = 25;
    ui_g_Static_Offset_Yaw->str_length = 2;
    strcpy(ui_g_Static_Offset_Yaw->string, "Y:");


    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}

void _ui_update_g_Static_2() {
    ui_g_Static_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}

void _ui_remove_g_Static_2() {
    ui_g_Static_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}
ui_string_frame_t ui_g_Static_3;
ui_interface_string_t* ui_g_Static_Distance = &(ui_g_Static_3.option);

void _ui_init_g_Static_3() {
    ui_g_Static_3.option.figure_name[0] = 0;
    ui_g_Static_3.option.figure_name[1] = 3;
    ui_g_Static_3.option.figure_name[2] = 3;
    ui_g_Static_3.option.operate_type = 1;

    ui_g_Static_Distance->figure_type = 7;
    ui_g_Static_Distance->operate_type = 1;
    ui_g_Static_Distance->layer = 0;
    ui_g_Static_Distance->color = 1;
    ui_g_Static_Distance->start_x = 859;
    ui_g_Static_Distance->start_y = 239;
    ui_g_Static_Distance->width = 3;
    ui_g_Static_Distance->font_size = 25;
    ui_g_Static_Distance->str_length = 2;
    strcpy(ui_g_Static_Distance->string, "D:");


    ui_proc_string_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}

void _ui_update_g_Static_3() {
    ui_g_Static_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}

void _ui_remove_g_Static_3() {
    ui_g_Static_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}

void ui_init_g_Static() {
    _ui_init_g_Static_0();
    // _ui_init_g_Static_1();
    // _ui_init_g_Static_2();
    // _ui_init_g_Static_3();
}

void ui_update_g_Static() {
    _ui_update_g_Static_0();
    _ui_update_g_Static_1();
    _ui_update_g_Static_2();
    _ui_update_g_Static_3();
}

void ui_remove_g_Static() {
    _ui_remove_g_Static_0();
    _ui_remove_g_Static_1();
    _ui_remove_g_Static_2();
    _ui_remove_g_Static_3();
}

ui_1_frame_t ui_g_Static2_0;

ui_interface_rect_t *ui_g_Static2_Power = (ui_interface_rect_t*)&(ui_g_Static2_0.data[0]);

void _ui_init_g_Static2_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_Static2_0.data[i].figure_name[0] = 0;
        ui_g_Static2_0.data[i].figure_name[1] = 4;
        ui_g_Static2_0.data[i].figure_name[2] = i + 0;
        ui_g_Static2_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_Static2_0.data[i].operate_type = 0;
    }

    ui_g_Static2_Power->figure_type = 1;
    ui_g_Static2_Power->operate_type = 1;
    ui_g_Static2_Power->layer = 0;
    ui_g_Static2_Power->color = 1;
    ui_g_Static2_Power->start_x = 602;
    ui_g_Static2_Power->start_y = 836;
    ui_g_Static2_Power->width = 1;
    ui_g_Static2_Power->end_x = 1319;
    ui_g_Static2_Power->end_y = 870;


    ui_proc_1_frame(&ui_g_Static2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_0, sizeof(ui_g_Static2_0));
}

void _ui_update_g_Static2_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_Static2_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_Static2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_0, sizeof(ui_g_Static2_0));
}

void _ui_remove_g_Static2_0() {
    for (int i = 0; i < 1; i++) {
        ui_g_Static2_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_Static2_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_0, sizeof(ui_g_Static2_0));
}

ui_string_frame_t ui_g_Static2_1;
ui_interface_string_t* ui_g_Static2_Leg_length = &(ui_g_Static2_1.option);

void _ui_init_g_Static2_1() {
    ui_g_Static2_1.option.figure_name[0] = 0;
    ui_g_Static2_1.option.figure_name[1] = 4;
    ui_g_Static2_1.option.figure_name[2] = 1;
    ui_g_Static2_1.option.operate_type = 1;

    ui_g_Static2_Leg_length->figure_type = 7;
    ui_g_Static2_Leg_length->operate_type = 1;
    ui_g_Static2_Leg_length->layer = 0;
    ui_g_Static2_Leg_length->color = 1;
    ui_g_Static2_Leg_length->start_x = 18;
    ui_g_Static2_Leg_length->start_y = 501;
    ui_g_Static2_Leg_length->width = 2;
    ui_g_Static2_Leg_length->font_size = 24;
    ui_g_Static2_Leg_length->str_length = 5;
    strcpy(ui_g_Static2_Leg_length->string, "LEG :");


    ui_proc_string_frame(&ui_g_Static2_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_1, sizeof(ui_g_Static2_1));
}

void _ui_update_g_Static2_1() {
    ui_g_Static2_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static2_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_1, sizeof(ui_g_Static2_1));
}

void _ui_remove_g_Static2_1() {
    ui_g_Static2_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static2_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_1, sizeof(ui_g_Static2_1));
}
ui_string_frame_t ui_g_Static2_2;
ui_interface_string_t* ui_g_Static2_Chassis = &(ui_g_Static2_2.option);

void _ui_init_g_Static2_2() {
    ui_g_Static2_2.option.figure_name[0] = 0;
    ui_g_Static2_2.option.figure_name[1] = 4;
    ui_g_Static2_2.option.figure_name[2] = 2;
    ui_g_Static2_2.option.operate_type = 1;

    ui_g_Static2_Chassis->figure_type = 7;
    ui_g_Static2_Chassis->operate_type = 1;
    ui_g_Static2_Chassis->layer = 0;
    ui_g_Static2_Chassis->color = 1;
    ui_g_Static2_Chassis->start_x = 16;
    ui_g_Static2_Chassis->start_y = 437;
    ui_g_Static2_Chassis->width = 2;
    ui_g_Static2_Chassis->font_size = 24;
    ui_g_Static2_Chassis->str_length = 8;
    strcpy(ui_g_Static2_Chassis->string, "Chassis:");


    ui_proc_string_frame(&ui_g_Static2_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_2, sizeof(ui_g_Static2_2));
}

void _ui_update_g_Static2_2() {
    ui_g_Static2_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static2_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_2, sizeof(ui_g_Static2_2));
}

void _ui_remove_g_Static2_2() {
    ui_g_Static2_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static2_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_2, sizeof(ui_g_Static2_2));
}
ui_string_frame_t ui_g_Static2_3;
ui_interface_string_t* ui_g_Static2_Gimbal = &(ui_g_Static2_3.option);

void _ui_init_g_Static2_3() {
    ui_g_Static2_3.option.figure_name[0] = 0;
    ui_g_Static2_3.option.figure_name[1] = 4;
    ui_g_Static2_3.option.figure_name[2] = 3;
    ui_g_Static2_3.option.operate_type = 1;

    ui_g_Static2_Gimbal->figure_type = 7;
    ui_g_Static2_Gimbal->operate_type = 1;
    ui_g_Static2_Gimbal->layer = 0;
    ui_g_Static2_Gimbal->color = 1;
    ui_g_Static2_Gimbal->start_x = 13;
    ui_g_Static2_Gimbal->start_y = 378;
    ui_g_Static2_Gimbal->width = 2;
    ui_g_Static2_Gimbal->font_size = 24;
    ui_g_Static2_Gimbal->str_length = 7;
    strcpy(ui_g_Static2_Gimbal->string, "Gimbal:");


    ui_proc_string_frame(&ui_g_Static2_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_3, sizeof(ui_g_Static2_3));
}

void _ui_update_g_Static2_3() {
    ui_g_Static2_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static2_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_3, sizeof(ui_g_Static2_3));
}

void _ui_remove_g_Static2_3() {
    ui_g_Static2_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static2_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_3, sizeof(ui_g_Static2_3));
}
ui_string_frame_t ui_g_Static2_4;
ui_interface_string_t* ui_g_Static2_Ammo = &(ui_g_Static2_4.option);

void _ui_init_g_Static2_4() {
    ui_g_Static2_4.option.figure_name[0] = 0;
    ui_g_Static2_4.option.figure_name[1] = 4;
    ui_g_Static2_4.option.figure_name[2] = 4;
    ui_g_Static2_4.option.operate_type = 1;

    ui_g_Static2_Ammo->figure_type = 7;
    ui_g_Static2_Ammo->operate_type = 1;
    ui_g_Static2_Ammo->layer = 0;
    ui_g_Static2_Ammo->color = 1;
    ui_g_Static2_Ammo->start_x = 16;
    ui_g_Static2_Ammo->start_y = 317;
    ui_g_Static2_Ammo->width = 2;
    ui_g_Static2_Ammo->font_size = 24;
    ui_g_Static2_Ammo->str_length = 5;
    strcpy(ui_g_Static2_Ammo->string, "Ammo:");


    ui_proc_string_frame(&ui_g_Static2_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_4, sizeof(ui_g_Static2_4));
}

void _ui_update_g_Static2_4() {
    ui_g_Static2_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static2_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_4, sizeof(ui_g_Static2_4));
}

void _ui_remove_g_Static2_4() {
    ui_g_Static2_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static2_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static2_4, sizeof(ui_g_Static2_4));
}

void ui_init_g_Static2() {
    _ui_init_g_Static2_0();
    _ui_init_g_Static2_1();
    _ui_init_g_Static2_2();
    _ui_init_g_Static2_3();
    _ui_init_g_Static2_4();
}

void ui_update_g_Static2() {
    _ui_update_g_Static2_0();
    _ui_update_g_Static2_1();
    _ui_update_g_Static2_2();
    _ui_update_g_Static2_3();
    _ui_update_g_Static2_4();
}

void ui_remove_g_Static2() {
    _ui_remove_g_Static2_0();
    _ui_remove_g_Static2_1();
    _ui_remove_g_Static2_2();
    _ui_remove_g_Static2_3();
    _ui_remove_g_Static2_4();
}

ui_2_frame_t ui_g_Static3_0;

ui_interface_line_t *ui_g_Static3_left_car_line = (ui_interface_line_t*)&(ui_g_Static3_0.data[0]);
ui_interface_line_t *ui_g_Static3_right_car_line = (ui_interface_line_t*)&(ui_g_Static3_0.data[1]);

void _ui_init_g_Static3_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Static3_0.data[i].figure_name[0] = 0;
        ui_g_Static3_0.data[i].figure_name[1] = 5;
        ui_g_Static3_0.data[i].figure_name[2] = i + 0;
        ui_g_Static3_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_Static3_0.data[i].operate_type = 0;
    }

    ui_g_Static3_left_car_line->figure_type = 0;
    ui_g_Static3_left_car_line->operate_type = 1;
    ui_g_Static3_left_car_line->layer = 0;
    ui_g_Static3_left_car_line->color = 0;
    ui_g_Static3_left_car_line->start_x = 547;
    ui_g_Static3_left_car_line->start_y = 86;
    ui_g_Static3_left_car_line->width = 6;
    ui_g_Static3_left_car_line->end_x = 722;
    ui_g_Static3_left_car_line->end_y = 516;

    ui_g_Static3_right_car_line->figure_type = 0;
    ui_g_Static3_right_car_line->operate_type = 1;
    ui_g_Static3_right_car_line->layer = 0;
    ui_g_Static3_right_car_line->color = 0;
    ui_g_Static3_right_car_line->start_x = 1377;
    ui_g_Static3_right_car_line->start_y = 72;
    ui_g_Static3_right_car_line->width = 7;
    ui_g_Static3_right_car_line->end_x = 1173;
    ui_g_Static3_right_car_line->end_y = 509;


    ui_proc_2_frame(&ui_g_Static3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static3_0, sizeof(ui_g_Static3_0));
}

void _ui_update_g_Static3_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Static3_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_g_Static3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static3_0, sizeof(ui_g_Static3_0));
}

void _ui_remove_g_Static3_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Static3_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_g_Static3_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static3_0, sizeof(ui_g_Static3_0));
}


void ui_init_g_Static3() {
    _ui_init_g_Static3_0();
}

void ui_update_g_Static3() {
    _ui_update_g_Static3_0();
}

void ui_remove_g_Static3() {
    _ui_remove_g_Static3_0();
}

