#ifndef BOOSTER_H
#define BOOSTER_H

#include "librm.hpp"
#include "Gimbal.h"
#include "DMtypedef.h"

using namespace rm;
using rm::hal::Can;
using namespace rm::device;
using namespace rm::modules::algorithm;

constexpr f32 ROTORZEROANGLE = 0.30;
constexpr u16 ROTATIONSPEED = 4420;
constexpr u16 SHOOTINGCOUNTDOWN = 360;

enum class ROTOR_STATE { STOP, Initialize, READY, SHOOTING, COOLING };

class RotorMotor {
  friend class Rotor;

 private:
  DmMotor<DmMotorControlMode::kMit> _rotormotor;
  RingPID<PIDType::kPosition> _rotorpositionpid;
  PID<PIDType::kPosition> _rotorspeedpid;

 public:
  RotorMotor(Can &motorcan);
  ~RotorMotor() = default;
  void act();
  void sendcommand();
};

class Rotor {
 private:
  ROTOR_STATE _state;
  float _rotorangle{0};
  float _rotorspeed{0};
  float _nowsetangle{ROTORZEROANGLE};
  float _nextsetangle{ROTORZEROANGLE};

  i16 initcountdown{0};
  i16 shootingcountdown{0};
  i32 heat_delta{0};

 public:
  i32 local_heat{0};
  Rotor();
  ~Rotor() = default;
  DMABLE _rotorable{false, false, false};
  void getRotormotor(RotorMotor &rotormotor);
  void act();
  [[nodiscard]] ROTOR_STATE getRotorState() const { return _state; };
  [[nodiscard]] float getnowsetangle() const { return _nowsetangle; };
  [[nodiscard]] float getnextsetangle() const { return _nextsetangle; };
};

class Shooter {
 private:
  u16 rotationspeed{0};
  M3508 _left;
  M3508 _right;
  M3508 _top;
  PID<PIDType::kPosition> _leftspeedpid;
  PID<PIDType::kPosition> _rightspeedpid;
  PID<PIDType::kPosition> _topspeedpid;

 public:
  i8 speedadd{0};
  Shooter(Can &shooterCan);
  ~Shooter() = default;
  void act();
  void sendcommand();
};

extern Rotor myrotor;

#endif /* BOOSTER_H */