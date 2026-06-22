#include "../include/ai/policy_runner.hpp"

#if WHEEL_LEGGED_ENABLE_AI_POLICY_TEST

#include <algorithm>
#include <cstring>

#include "stm32h7xx_hal.h"

extern "C" {
#include "network.h"
#include "network_data.h"
}

namespace wheel_legged::ai {
namespace {

alignas(4) ai_u8 g_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
ai_handle g_network = AI_HANDLE_NULL;
PolicyRunner g_policy_runner;
std::uint32_t g_dummy_tick = 0;
volatile bool g_policy_request_pending = false;
PolicyInput g_pending_input{};

std::uint32_t CyclesToUs(std::uint32_t cycles) {
  const std::uint32_t hclk = HAL_RCC_GetHCLKFreq();
  return (hclk == 0u) ? 0u : static_cast<std::uint32_t>((static_cast<std::uint64_t>(cycles) * 1000000u) / hclk);
}

float Clamp(float value, float min_value, float max_value) { return std::max(min_value, std::min(max_value, value)); }

}  // namespace

PolicyDebug ai_policy_debug __attribute__((section(".sram4")));

bool PolicyRunner::Init() {
  if (initialized_) return true;

  const ai_handle activations[] = {AI_HANDLE_PTR(g_activations)};
  const ai_error err = ai_network_create_and_init(&g_network, activations, nullptr);

  initialized_ = (err.type == AI_ERROR_NONE);
  output_.initialized = initialized_;
  output_.ok = initialized_;
  if (!initialized_) {
    output_.fail_count++;
  }
  return initialized_;
}

bool PolicyRunner::Step(const PolicyInput& input) {
  if (!initialized_ && !Init()) return false;

  ai_buffer* inputs = ai_network_inputs_get(g_network, nullptr);
  ai_buffer* outputs = ai_network_outputs_get(g_network, nullptr);
  if (inputs == nullptr || outputs == nullptr) {
    output_.ok = false;
    output_.fail_count++;
    return false;
  }

  std::memcpy(inputs[0].data, input.observations, sizeof(input.observations));
  std::memcpy(inputs[1].data, input.observation_history, sizeof(input.observation_history));

  const std::uint32_t start_cycles = DWT->CYCCNT;
  const ai_i32 batch_count = ai_network_run(g_network, inputs, outputs);
  const std::uint32_t infer_us = CyclesToUs(DWT->CYCCNT - start_cycles);

  output_.last_infer_us = infer_us;
  output_.max_infer_us = std::max(output_.max_infer_us, infer_us);
  output_.step_count++;

  if (batch_count != 1) {
    (void)ai_network_get_error(g_network);
    output_.ok = false;
    output_.fail_count++;
    return false;
  }

  std::memcpy(output_.actions, outputs[0].data, sizeof(output_.actions));
  output_.ok = true;
  return true;
}

PolicyInput BuildDummyPolicyInput(const PolicyOutput& previous_output) {
  PolicyInput input{};

  const float phase = static_cast<float>(g_dummy_tick % 200u) * 0.005f;
  input.observations[0] = 0.02f * phase;
  input.observations[1] = -0.01f * phase;
  input.observations[2] = 0.03f * phase;
  input.observations[3] = 0.0f;
  input.observations[4] = 0.0f;
  input.observations[5] = -1.0f;
  input.observations[6] = 0.10f;
  input.observations[7] = 0.0f;
  input.observations[8] = 1.15f;
  input.observations[9] = 0.0f;
  input.observations[10] = 0.0f;
  input.observations[11] = 0.0f;
  input.observations[12] = 0.0f;
  input.observations[13] = 0.85f;
  input.observations[14] = 0.85f;
  input.observations[15] = 0.0f;
  input.observations[16] = 0.0f;
  input.observations[17] = 0.0f;
  input.observations[18] = 0.0f;
  input.observations[19] = 0.0f;
  input.observations[20] = 0.0f;
  for (std::uint32_t i = 0; i < 6u; ++i) {
    input.observations[21u + i] = Clamp(previous_output.actions[i], -3.0f, 3.0f);
  }

  for (std::uint32_t frame = 0; frame < 5u; ++frame) {
    std::memcpy(&input.observation_history[frame * 27u], input.observations, sizeof(input.observations));
  }
  g_dummy_tick++;
  return input;
}

void PolicyTestInit() {
  (void)g_policy_runner.Init();
  ai_policy_debug.initialized = g_policy_runner.output().initialized;
  ai_policy_debug.ok = g_policy_runner.output().ok;
}

void PolicyTestRequest() { PolicyTestRequest(BuildDummyPolicyInput(g_policy_runner.output())); }

void PolicyTestRequest(const PolicyInput& input) {
  g_pending_input = input;
  g_policy_request_pending = true;
  ai_policy_debug.request_pending = true;
  ai_policy_debug.request_count++;
}

void PolicyTestPoll() {
  if (!g_policy_request_pending) return;

  PolicyInput input{};
  __disable_irq();
  input = g_pending_input;
  g_policy_request_pending = false;
  __enable_irq();

  std::memcpy(ai_policy_debug.observations, input.observations, sizeof(ai_policy_debug.observations));
  std::memcpy(ai_policy_debug.observation_history, input.observation_history,
              sizeof(ai_policy_debug.observation_history));
  ai_policy_debug.request_pending = false;
  ai_policy_debug.poll_count++;

  (void)g_policy_runner.Step(input);

  const PolicyOutput& output = g_policy_runner.output();
  std::memcpy(ai_policy_debug.actions, output.actions, sizeof(ai_policy_debug.actions));
  ai_policy_debug.initialized = output.initialized;
  ai_policy_debug.ok = output.ok;
  ai_policy_debug.step_count = output.step_count;
  ai_policy_debug.fail_count = output.fail_count;
  ai_policy_debug.last_infer_us = output.last_infer_us;
  ai_policy_debug.max_infer_us = output.max_infer_us;
}

const PolicyOutput& GetPolicyTestOutput() { return g_policy_runner.output(); }

}  // namespace wheel_legged::ai

#else

namespace wheel_legged::ai {

namespace {
PolicyOutput g_disabled_output{};
}

PolicyDebug ai_policy_debug{};

PolicyInput BuildDummyPolicyInput(const PolicyOutput&) { return {}; }

void PolicyTestInit() {}

void PolicyTestRequest() {}

void PolicyTestRequest(const PolicyInput&) {}

void PolicyTestPoll() {}

const PolicyOutput& GetPolicyTestOutput() { return g_disabled_output; }

}  // namespace wheel_legged::ai

#endif
