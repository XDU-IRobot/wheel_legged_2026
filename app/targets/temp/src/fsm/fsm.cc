#include "fsm.hpp"

#include "etl/fsm.h"
#include "etl/message.h"

namespace {

// ETL 内部状态 ID，与 Fsm::State 一一对应。
enum EtlStateId : etl::fsm_state_id_t {
  kNoForceState = 0,
  kNormalLowLegState = 1,
  kNormalMidLegState = 2,
  kNormalHighLegState = 3,
  kSmallGyroState = 4,
  kJumpRetract1State = 5,
  kJumpExtendState = 6,
  kJumpRetract2State = 7,
  kStateCount = 8,
};

constexpr uint32_t kJumpRetract1Ms = 450U;
constexpr uint32_t kJumpExtendMs = 1000U;
constexpr uint32_t kJumpRetract2Ms = 450U;
constexpr float kJumpExtendTargetLegLengthM = 0.35f;
constexpr float kJumpExtendReachTolM = 0.01f;

constexpr EtlStateId ToEtlStateId(const Fsm::State mode) {
  switch (mode) {
    case Fsm::State::kNoForce:
      return kNoForceState;
    case Fsm::State::kNormalLowLeg:
      return kNormalLowLegState;
    case Fsm::State::kNormalMidLeg:
      return kNormalMidLegState;
    case Fsm::State::kNormalHighLeg:
      return kNormalHighLegState;
    case Fsm::State::kSmallGyro:
      return kSmallGyroState;
    case Fsm::State::kJumpRetract1:
      return kJumpRetract1State;
    case Fsm::State::kJumpExtend:
      return kJumpExtendState;
    case Fsm::State::kJumpRetract2:
      return kJumpRetract2State;
    default:
      return kNoForceState;
  }
}

Fsm::State ResolveForceState(const Fsm::LegLengthMode leg_length_mode, const bool small_gyro_enable) {
  if (small_gyro_enable) {
    return Fsm::State::kSmallGyro;
  }

  switch (leg_length_mode) {
    case Fsm::LegLengthMode::kLow:
      return Fsm::State::kNormalLowLeg;
    case Fsm::LegLengthMode::kMid:
      return Fsm::State::kNormalMidLeg;
    case Fsm::LegLengthMode::kHigh:
      return Fsm::State::kNormalHighLeg;
    default:
      return Fsm::State::kNormalLowLeg;
  }
}

bool IsJumpState(const Fsm::State mode) {
  return mode == Fsm::State::kJumpRetract1 || mode == Fsm::State::kJumpExtend || mode == Fsm::State::kJumpRetract2;
}

// 状态到控制动作的映射表。
Fsm::Output::ControlOutput BuildControlOutput(const Fsm::State mode) {
  Fsm::Output::ControlOutput control{};
  switch (mode) {
    case Fsm::State::kNoForce:
      control.enable_dm = false;
      control.run_chassis_update = false;
      control.small_gyro_enable = false;
      control.target_leg_length_m = 0.18f;
      control.jump_phase = 0U;
      break;

    case Fsm::State::kNormalLowLeg:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.small_gyro_enable = false;
      control.target_leg_length_m = 0.18f;
      control.jump_phase = 0U;
      break;

    case Fsm::State::kNormalMidLeg:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.small_gyro_enable = false;
      control.target_leg_length_m = 0.23f;
      control.jump_phase = 0U;
      break;

    case Fsm::State::kNormalHighLeg:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.small_gyro_enable = false;
      control.target_leg_length_m = 0.23f;
      control.jump_phase = 0U;
      break;

    case Fsm::State::kSmallGyro:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.small_gyro_enable = true;
      control.target_leg_length_m = 0.18f;
      control.jump_phase = 0U;
      break;

    case Fsm::State::kJumpRetract1:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.small_gyro_enable = false;
      control.target_leg_length_m = 0.14f;
      control.jump_phase = 1U;
      break;

    case Fsm::State::kJumpExtend:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.small_gyro_enable = false;
      control.target_leg_length_m = 0.38f;
      control.jump_phase = 2U;
      break;

    case Fsm::State::kJumpRetract2:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.small_gyro_enable = false;
      control.target_leg_length_m = 0.2f;
      control.jump_phase = 3U;
      break;

    default:
      control.enable_dm = false;
      control.run_chassis_update = false;
      control.small_gyro_enable = false;
      control.target_leg_length_m = 0.18f;
      control.jump_phase = 0U;
      break;
  }
  return control;
}

struct ModeRequestEvent : public etl::message<1> {
  bool force_enable;
  Fsm::LegLengthMode leg_length_mode;
  bool small_gyro_enable;
  bool jump_trigger_rise;
  float current_leg_length_m;
  uint32_t tick_ms;

  explicit ModeRequestEvent(const bool force_enable, const Fsm::LegLengthMode leg_length_mode,
                            const bool small_gyro_enable, const bool jump_trigger_rise,
                            const float current_leg_length_m, const uint32_t tick_ms)
      : force_enable(force_enable),
        leg_length_mode(leg_length_mode),
        small_gyro_enable(small_gyro_enable),
        jump_trigger_rise(jump_trigger_rise),
        current_leg_length_m(current_leg_length_m),
        tick_ms(tick_ms) {}
};

class FsmEtlContext;

// 无力态。
class NoForceState : public etl::fsm_state<FsmEtlContext, NoForceState, kNoForceState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class NormalLowLegState
    : public etl::fsm_state<FsmEtlContext, NormalLowLegState, kNormalLowLegState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class NormalMidLegState
    : public etl::fsm_state<FsmEtlContext, NormalMidLegState, kNormalMidLegState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class NormalHighLegState
    : public etl::fsm_state<FsmEtlContext, NormalHighLegState, kNormalHighLegState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class SmallGyroState : public etl::fsm_state<FsmEtlContext, SmallGyroState, kSmallGyroState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class JumpRetract1State
    : public etl::fsm_state<FsmEtlContext, JumpRetract1State, kJumpRetract1State, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class JumpExtendState : public etl::fsm_state<FsmEtlContext, JumpExtendState, kJumpExtendState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class JumpRetract2State
    : public etl::fsm_state<FsmEtlContext, JumpRetract2State, kJumpRetract2State, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class FsmEtlContext : public etl::fsm {
 public:
  explicit FsmEtlContext(Fsm &owner) : etl::fsm(0x41), owner_(owner) {
    states_[0] = &no_force_;
    states_[1] = &normal_low_leg_;
    states_[2] = &normal_mid_leg_;
    states_[3] = &normal_high_leg_;
    states_[4] = &small_gyro_;
    states_[5] = &jump_retract1_;
    states_[6] = &jump_extend_;
    states_[7] = &jump_retract2_;
    set_states(states_, kStateCount);
  }

  void ApplyMode(Fsm::State mode) { owner_.Transit(mode); }
  void Dispatch(const ModeRequestEvent &event) {
    tick_ms_ = event.tick_ms;
    receive(event);
  }
  void MarkJumpPhaseEnter() { jump_phase_enter_ms_ = tick_ms_; }
  uint32_t JumpPhaseElapsedMs() const { return tick_ms_ - jump_phase_enter_ms_; }
  etl::fsm_state_id_t ResolveRequestedState(const ModeRequestEvent &event) const {
    return ToEtlStateId(ResolveForceState(event.leg_length_mode, event.small_gyro_enable));
  }

 private:
  Fsm &owner_;
  NoForceState no_force_;
  NormalLowLegState normal_low_leg_;
  NormalMidLegState normal_mid_leg_;
  NormalHighLegState normal_high_leg_;
  SmallGyroState small_gyro_;
  JumpRetract1State jump_retract1_;
  JumpExtendState jump_extend_;
  JumpRetract2State jump_retract2_;
  etl::ifsm_state *states_[kStateCount]{};
  uint32_t tick_ms_{0};
  uint32_t jump_phase_enter_ms_{0};
};

etl::fsm_state_id_t NoForceState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return No_State_Change;
  }

  if (event.jump_trigger_rise) {
    return kJumpRetract1State;
  }

  return get_fsm_context().ResolveRequestedState(event);
}

etl::fsm_state_id_t NoForceState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t NoForceState::on_enter_state() {
  get_fsm_context().ApplyMode(Fsm::State::kNoForce);
  return No_State_Change;
}

etl::fsm_state_id_t NormalLowLegState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kNoForceState;
  }
  if (event.jump_trigger_rise) {
    return kJumpRetract1State;
  }
  return get_fsm_context().ResolveRequestedState(event);
}

etl::fsm_state_id_t NormalLowLegState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t NormalLowLegState::on_enter_state() {
  get_fsm_context().ApplyMode(Fsm::State::kNormalLowLeg);
  return No_State_Change;
}

etl::fsm_state_id_t NormalMidLegState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kNoForceState;
  }
  if (event.jump_trigger_rise) {
    return kJumpRetract1State;
  }
  return get_fsm_context().ResolveRequestedState(event);
}

etl::fsm_state_id_t NormalMidLegState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t NormalMidLegState::on_enter_state() {
  get_fsm_context().ApplyMode(Fsm::State::kNormalMidLeg);
  return No_State_Change;
}

etl::fsm_state_id_t NormalHighLegState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kNoForceState;
  }
  if (event.jump_trigger_rise) {
    return kJumpRetract1State;
  }
  return get_fsm_context().ResolveRequestedState(event);
}

etl::fsm_state_id_t NormalHighLegState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t NormalHighLegState::on_enter_state() {
  get_fsm_context().ApplyMode(Fsm::State::kNormalHighLeg);
  return No_State_Change;
}

etl::fsm_state_id_t SmallGyroState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kNoForceState;
  }
  if (event.jump_trigger_rise) {
    return kJumpRetract1State;
  }
  return get_fsm_context().ResolveRequestedState(event);
}

etl::fsm_state_id_t SmallGyroState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t SmallGyroState::on_enter_state() {
  get_fsm_context().ApplyMode(Fsm::State::kSmallGyro);
  return No_State_Change;
}

etl::fsm_state_id_t JumpRetract1State::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kNoForceState;
  }
  return get_fsm_context().JumpPhaseElapsedMs() >= kJumpRetract1Ms ? kJumpExtendState : No_State_Change;
}

etl::fsm_state_id_t JumpRetract1State::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t JumpRetract1State::on_enter_state() {
  get_fsm_context().MarkJumpPhaseEnter();
  get_fsm_context().ApplyMode(Fsm::State::kJumpRetract1);
  return No_State_Change;
}

etl::fsm_state_id_t JumpExtendState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kNoForceState;
  }
  const bool leg_reached = event.current_leg_length_m >= (kJumpExtendTargetLegLengthM - kJumpExtendReachTolM);
  if (leg_reached || get_fsm_context().JumpPhaseElapsedMs() >= kJumpExtendMs) {
    return kJumpRetract2State;
  }
  return No_State_Change;
}

etl::fsm_state_id_t JumpExtendState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t JumpExtendState::on_enter_state() {
  get_fsm_context().MarkJumpPhaseEnter();
  get_fsm_context().ApplyMode(Fsm::State::kJumpExtend);
  return No_State_Change;
}

etl::fsm_state_id_t JumpRetract2State::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kNoForceState;
  }
  if (get_fsm_context().JumpPhaseElapsedMs() < kJumpRetract2Ms) {
    return No_State_Change;
  }

  return get_fsm_context().ResolveRequestedState(event);
}

etl::fsm_state_id_t JumpRetract2State::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t JumpRetract2State::on_enter_state() {
  get_fsm_context().MarkJumpPhaseEnter();
  get_fsm_context().ApplyMode(Fsm::State::kJumpRetract2);
  return No_State_Change;
}

}  // namespace

struct Fsm::EtlImpl {
  explicit EtlImpl(Fsm &owner) : context(owner) {}

  FsmEtlContext context;
  bool last_jump_trigger{false};
};

Fsm::~Fsm() {
  delete etl_impl_;
  etl_impl_ = nullptr;
}

// 初始化 ETL 状态机，默认进入 kNoForce。
void Fsm::Init() {
  if (!etl_impl_) {
    etl_impl_ = new EtlImpl(*this);
  }

  if (etl_impl_ && !etl_impl_->context.is_started()) {
    etl_impl_->context.start(true);
  }
}

void Fsm::Transit(State new_mode) {
  // 统一在状态迁移时刷新输出，确保 mode/control 同步。
  output_.state_changed = (new_mode != mode_);
  mode_ = new_mode;
  output_.mode = mode_;
  output_.control = BuildControlOutput(mode_);
}

Fsm::Output Fsm::Update(const Input &input) {
  if (!etl_impl_) {
    Init();
  }

  output_.state_changed = false;

  if (!input.input_valid) {
    // 输入无效时强制回退无力态。
    Transit(State::kNoForce);
    etl_impl_->last_jump_trigger = false;
    return output_;
  }

  const bool jump_trigger_rise = input.jump_trigger && !etl_impl_->last_jump_trigger;
  etl_impl_->last_jump_trigger = input.jump_trigger;

  // 跳跃序列是时间驱动状态，处于跳跃阶段时每拍都需要投递事件推进子状态。
  const bool should_dispatch =
      jump_trigger_rise || IsJumpState(mode_) ||
      (ToEtlStateId(ResolveForceState(input.leg_length_mode, input.small_gyro_enable)) != ToEtlStateId(mode_)) ||
      (mode_ == State::kNoForce && input.force_enable) || (mode_ != State::kNoForce && !input.force_enable);

  if (!should_dispatch) {
    return output_;
  }

  etl_impl_->context.Dispatch(ModeRequestEvent{input.force_enable, input.leg_length_mode, input.small_gyro_enable,
                                               jump_trigger_rise, input.current_leg_length_m, input.tick_ms});
  return output_;
}
