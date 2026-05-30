//
// Created by refactored from ui.hpp — drone_gb_new style
//

#ifndef UIWHEELLEGGED_HPP
#define UIWHEELLEGGED_HPP

// ── Static labels (one-shot add) ──
extern void UIWheelLeggedLabelPY_add();
extern void UIWheelLeggedLabelLeg_add();
extern void UIWheelLeggedDecorativeRect_add();

// ── Crosshair (add + edit) ──
extern void UIWheelLeggedCrosshair_add();
extern void UIWheelLeggedCrosshair_edit();

// ── Gimbal data display (hero variant) ──
extern void UIWheelLeggedGimbalData_add();
extern void UIWheelLeggedGimbalData_edit();

// ── Leg kinematics + supercap energy bar ──
extern void UIWheelLeggedKinematics_add();
extern void UIWheelLeggedKinematics_edit();

// ── Friction wheel RPM ──
extern void UIWheelLeggedFricRPM_add();
extern void UIWheelLeggedFricRPM_edit();

// ── Bullet speed + projectile allowance ──
extern void UIWheelLeggedBulletData_add();
extern void UIWheelLeggedBulletData_edit();

// ── Status labels (infantry variant, rotating) ──
extern void UIWheelLeggedStatusLabel_add();

// ── State indicator (infantry variant) ──
extern void UIWheelLeggedStateIndicator_add();
extern void UIWheelLeggedStateIndicator_edit();

// ── Aimbot target box ──
extern void UIWheelLeggedAimbotBox_add();
extern void UIWheelLeggedAimbotBox_edit();

// ── Helper: five-bar linkage inverse kinematics ──
extern void calcPointC(double x1, double y1, double x2, double y2, double L1, double L2, int sel,
                       float *x3_out, float *y3_out);

#endif  // UIWHEELLEGGED_HPP
