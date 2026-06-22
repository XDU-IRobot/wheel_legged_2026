#pragma once

#include <cstdint>

#ifndef WHEEL_LEGGED_ENABLE_AI_POLICY_TEST
#define WHEEL_LEGGED_ENABLE_AI_POLICY_TEST 1
#endif

namespace wheel_legged::ai {

struct PolicyInput {
  float observations[27]{};
  float observation_history[135]{};
};

struct PolicyOutput {
  float actions[6]{};
  bool initialized{false};
  bool ok{false};
  std::uint32_t step_count{0};
  std::uint32_t fail_count{0};
  std::uint32_t last_infer_us{0};
  std::uint32_t max_infer_us{0};
};

struct PolicyDebug {
  float observations[27]{};
  float observation_history[135]{};
  float actions[6]{};
  bool request_pending{false};
  bool initialized{false};
  bool ok{false};
  std::uint32_t request_count{0};
  std::uint32_t poll_count{0};
  std::uint32_t step_count{0};
  std::uint32_t fail_count{0};
  std::uint32_t last_infer_us{0};
  std::uint32_t max_infer_us{0};
};

class PolicyRunner {
 public:
  bool Init();
  bool Step(const PolicyInput& input);

  [[nodiscard]] const PolicyOutput& output() const { return output_; }

 private:
  PolicyOutput output_{};
  bool initialized_{false};
};

PolicyInput BuildDummyPolicyInput(const PolicyOutput& previous_output);

void PolicyTestInit();
void PolicyTestRequest();
void PolicyTestRequest(const PolicyInput& input);
void PolicyTestPoll();

const PolicyOutput& GetPolicyTestOutput();

extern PolicyDebug ai_policy_debug;

}  // namespace wheel_legged::ai
