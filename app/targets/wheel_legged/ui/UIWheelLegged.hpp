//
// Created by refactored from ui.hpp — drone_gb_new style
//

#ifndef UIWHEELLEGGED_HPP
#define UIWHEELLEGGED_HPP

// ── Static labels (one-shot add) ──
extern void UIWheelLeggedLabelPY_add();
extern void UIWheelLeggedLabelLeg_add();
extern void UIWheelLeggedLabelAD_add();
extern void UIWheelLeggedDecorativeRect_add();

// ── Crosshair (add + edit) ──
extern void UIWheelLeggedCrosshair_add();
extern void UIWheelLeggedCrosshair_edit();

// ── Gimbal data display (hero variant) ──
extern void UIWheelLeggedGimbalData_add();
extern void UIWheelLeggedGimbalData_edit();

// ── Supercap energy bar ──
extern void UIWheelLeggedSupercapBox_add();
extern void UIWheelLeggedSupercap_add();
extern void UIWheelLeggedSupercap_edit();

// ── Leg length indicator (L M H box) ──
extern void UIWheelLeggedLegBox_add();
extern void UIWheelLeggedLegBox_edit();

// ── Leg pose lines + yaw arc ──
extern void UIWheelLeggedLegPose_add();
extern void UIWheelLeggedLegPose_edit();

// ── Friction wheel RPM ──
extern void UIWheelLeggedFricRPM_add();
extern void UIWheelLeggedFricRPM_edit();

// ── Bullet speed + projectile allowance ──
extern void UIWheelLeggedBulletData_add();
extern void UIWheelLeggedBulletData_edit();

// ── Status labels (infantry variant) ──
extern void UIWheelLeggedStatusLabel_add_st1();
extern void UIWheelLeggedStatusLabel_add_st2();
extern void UIWheelLeggedStatusLabel_add_st3();

// ── State indicator (infantry variant) ──
extern void UIWheelLeggedStateIndicator_add();
extern void UIWheelLeggedStateIndicator_edit();

// ── Aimbot target box ──
extern void UIWheelLeggedAimbotBox_add();
extern void UIWheelLeggedAimbotBox_edit();

// ── Helper: five-bar linkage inverse kinematics ──
extern void calcPointC(double x1, double y1, double x2, double y2, double L1, double L2, int sel, float *x3_out,
                       float *y3_out);

#endif  // UIWHEELLEGGED_HPP
