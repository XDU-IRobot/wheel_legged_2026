#include "Booster.h"
#include "ControlSource.h"
#include "librm/modules/algorithm/utils.hpp"
#include "Communicate.h"
#include "Aimbot.h"

using rm::modules::algorithm::utils::LoopConstrain;

extern RefereeData refereedata;
extern uint32_t System_time;

Rotor myrotor;

// struct test1 {
//   u16 heatdelta;
//   ROTOR_STATE shootmode;
// };

// test1 test[1024];

// u16 a = 0;

RotorMotor::RotorMotor(Can &motorcan)
    : _rotormotor(
          motorcan,
          DmMotorSettings<DmMotorControlMode::kMit>{0x06, 0x07, DMPI180, 30, 10, std::make_pair((float)0, (float)500),
                                                    std::make_pair((float)0, (float)5)},

          true),
      _rotorpositionpid(60.0f, 0.00f, 560.0f, 24.0f, 3.0f, DMPI360),
      _rotorspeedpid(0.30f, 0.0f, 0.02f, 6.4f, 0.0f) {}

Rotor::Rotor() : _state(ROTOR_STATE::STOP) {}

void RotorMotor::act() {
  // 拨盘实际输出计算
  _rotorpositionpid.Update(myrotor.getnowsetangle(), this->_rotormotor.pos());
  _rotorspeedpid.Update(_rotorpositionpid.value(), this->_rotormotor.vel());
}

void RotorMotor::sendcommand() {
  if (myrotor._rotorable.enablekey == 1) {
    this->_rotormotor.SendInstruction(DmMotorInstructions::kEnable);
    myrotor._rotorable.enablekey = 0;
  } else if (myrotor._rotorable.disablekey == 1) {
    this->_rotormotor.SendInstruction(DmMotorInstructions::kDisable);
    myrotor._rotorable.disablekey = 0;
  } else if (myrotor._rotorable.cleankey == 1) {
    this->_rotormotor.SendInstruction(DmMotorInstructions::kClearError);
    myrotor._rotorable.cleankey = 0;
  } else if (myrotor.getRotorState() == ROTOR_STATE::READY || myrotor.getRotorState() == ROTOR_STATE::SHOOTING ||
             myrotor.getRotorState() == ROTOR_STATE::COOLING) {
    this->_rotormotor.SetPosition(0, 0, -_rotorspeedpid.value(), 0, 0);
  } else {
  }
}

void Rotor::getRotormotor(RotorMotor &rotormotor) {
  // 拨盘速度与位置
  _rotorspeed = rotormotor._rotormotor.vel();
  _rotorangle = rotormotor._rotormotor.pos();
}

void Rotor::act() {
  if (mygimbal.trustreferee) {
    heat_delta = refereedata.getmaxheat() - refereedata.getRealHeat();
  } else {
    heat_delta = local_heat;
  }

  if (refereedata.getPowerState(POWER::AMMOBOOSTER) == 0) {
    _state = ROTOR_STATE::STOP;
  }
  if (ctrl16.key(RcKey::kCtrl) && ctrl16.key_once(RcKey::kG)) {
    mygimbal._pitchable.cleankey = true;
    mygimbal.setGimbalState(GimbalState::NOFORCE);
    _rotorable.cleankey = true;
    _state = ROTOR_STATE::STOP;
  }

  // if (System_time > 15000) {
  //   a++;
  //   if (a % 10 == 0 && a / 10 < 1024) {
  //     test[a / 10].heatdelta = heat_delta;
  //     test[a / 10].shootmode = _state;
  //   }
  // }

  // 拨盘状态逻辑
  switch (_state) {
    case ROTOR_STATE::STOP:
      if (ctrl16.switch_r() == RcSwitchState::kUp) {
        _state = ROTOR_STATE::Initialize;
        _nowsetangle = ROTORZEROANGLE;
        _nextsetangle = ROTORZEROANGLE;
        initcountdown = 600;
      }
      break;
    case ROTOR_STATE::Initialize:
      if (initcountdown > 0) {
        initcountdown--;
        if (initcountdown == 50) {
          _rotorable.disablekey = true;
        }
      } else {
        while (_nowsetangle < _rotorangle) {
          _nowsetangle += DMPI60;
        }
        while (_nowsetangle > _rotorangle) {
          _nowsetangle -= DMPI60;
        }
        if (_rotorangle - _nowsetangle > DMPI60 / 2) {
          _nowsetangle += DMPI60;
          if (_nowsetangle > DMPI180) {
            _nowsetangle -= 2 * DMPI180;
          }
          _nextsetangle = _nowsetangle + DMPI60;
          if (_nextsetangle > DMPI180) {
            _nextsetangle -= 2 * DMPI180;
          }
        } else {
          _nextsetangle = _nowsetangle + DMPI60;
          if (_nextsetangle > DMPI180) {
            _nextsetangle -= 2 * DMPI180;
          }
          _nowsetangle = _rotorangle;
        }
        _rotorable.enablekey = true;
        Rotor::_state = ROTOR_STATE::READY;
      }
      break;
    case ROTOR_STATE::READY:
      if (ctrl16.switch_r() != RcSwitchState::kUp) {
        _rotorable.disablekey = true;
        Rotor::_state = ROTOR_STATE::STOP;
      } else {
        if (ctrl16.mouse_button_left() || ctrl16.dial() > 100) {
          if (mygimbal.getControlMode() == ControlMode::MANUAL || (mygimbal.getControlMode() == ControlMode::MAN)) {
            Rotor::_state = ROTOR_STATE::SHOOTING;
            shootingcountdown = LoopConstrain(SHOOTINGCOUNTDOWN - (refereedata.getmaxheat() - 100) / 2, 250, 800);
            _nowsetangle = _nextsetangle;
            local_heat -= 120;

          } else {
            if (aimbot.getfire()) {
              Rotor::_state = ROTOR_STATE::SHOOTING;
              shootingcountdown = LoopConstrain(SHOOTINGCOUNTDOWN - (refereedata.getmaxheat() - 100) / 2, 250, 800);
              _nowsetangle = _nextsetangle;
              local_heat -= 120;
            }
          }
        }
      }
      break;
    case ROTOR_STATE::SHOOTING:
      if (shootingcountdown > 0) {
        shootingcountdown--;
      } else {
        Rotor::_state = ROTOR_STATE::COOLING;
        // 堵转检测
        if (LoopConstrain(_nowsetangle - _rotorangle, -DMPI180, DMPI180) > DMPI60 / 6) {
          _nowsetangle = _rotorangle - DMPI60 / 30;
        } else {
          _nextsetangle = _nowsetangle + DMPI60;
          if (_nextsetangle > DMPI180) {
            _nextsetangle -= 2 * DMPI180;
          }
        }
      }
      break;
    case ROTOR_STATE::COOLING:
      // 热量闭环
      if (ctrl16.switch_r() != RcSwitchState::kUp) {
        _rotorable.disablekey = true;
        Rotor::_state = ROTOR_STATE::STOP;
        break;
      }
      if (heat_delta >= 100) {
        Rotor::_state = ROTOR_STATE::READY;
      } else {
        Rotor::_state = ROTOR_STATE::COOLING;
      }
      // Rotor::_state = ROTOR_STATE::READY;
      break;
    default:
      Rotor::_state = ROTOR_STATE::STOP;
      break;
  }
}

Shooter::Shooter(Can &shooterCan)
    : _left(shooterCan, 2, true),
      _right(shooterCan, 3),
      _top(shooterCan, 1, true),
      _leftspeedpid(30, 0.001, 0, 10000, 1600),
      _rightspeedpid(30, 0.001, 0, 10000, 1600),
      _topspeedpid(30, 0.001, 0, 10000, 1600) {}

void Shooter::act() {
  auto nowstate = myrotor.getRotorState();
  if (ctrl16.key(RcKey::kCtrl) && ctrl16.key_once(RcKey::kV)) {
    speedadd++;
  } else if (ctrl16.key(RcKey::kCtrl) && ctrl16.key_once(RcKey::kX)) {
    speedadd--;
  }
  if (nowstate == ROTOR_STATE::Initialize || nowstate == ROTOR_STATE::READY || nowstate == ROTOR_STATE::SHOOTING ||
      nowstate == ROTOR_STATE::COOLING) {
    this->rotationspeed = ROTATIONSPEED + speedadd * 30;
  } else {
    this->rotationspeed = 0;
  }
  _leftspeedpid.Update(this->rotationspeed, _left.rpm());
  _rightspeedpid.Update(this->rotationspeed, _right.rpm());
  _topspeedpid.Update(this->rotationspeed, _top.rpm());
  if ((nowstate == ROTOR_STATE::Initialize || nowstate == ROTOR_STATE::READY || nowstate == ROTOR_STATE::SHOOTING ||
       nowstate == ROTOR_STATE::COOLING) ||
      (_left.rpm() > 2000 || _right.rpm() > 2000 || _top.rpm() > 2000)) {
    _left.SetCurrent(_leftspeedpid.value());
    _right.SetCurrent(_rightspeedpid.value());
    _top.SetCurrent(_topspeedpid.value());
  } else {
    _left.SetCurrent(0);
    _right.SetCurrent(0);
    _top.SetCurrent(0);
  }
}

void Shooter::sendcommand() { _left.SendCommand(); }