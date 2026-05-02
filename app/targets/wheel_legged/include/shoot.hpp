#pragma once

#include <cstdint>

struct SharedResources;

/**
 * @brief 发射机构控制器
 * @note  封装摩擦轮与拨盘电机的 PID 初始化、使能/失能与周期更新。
 *        当云台进入 kCombat 模式时使能，设定摩擦轮目标转速，
 *        DR16 拨轮控制拨盘连发。
 */
class Shoot {
 public:
  void Init(SharedResources &g);
  void Enable();
  void Disable();
  void Update(SharedResources &g, float dt, int16_t dr16_dial, bool mouse_left);

  bool enabled() const { return enabled_; }

 private:
  bool enabled_{false};
};
