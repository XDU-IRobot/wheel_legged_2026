#include "common/device/vl53l4cd.hpp"

#include <array>

#include "i2c.h"

namespace device {
namespace {

constexpr std::uint16_t kVhvTimeout = 0x0008;
constexpr std::uint16_t kGpioHvMuxCtrl = 0x0030;
constexpr std::uint16_t kGpioStatus = 0x0031;
constexpr std::uint16_t kRangeConfigA = 0x005E;
constexpr std::uint16_t kRangeConfigB = 0x0061;
constexpr std::uint16_t kInterMeasurement = 0x006C;
constexpr std::uint16_t kInterruptClear = 0x0086;
constexpr std::uint16_t kSystemStart = 0x0087;
constexpr std::uint16_t kResultRangeStatus = 0x0089;
constexpr std::uint16_t kResultSpadCount = 0x008C;
constexpr std::uint16_t kResultSignalRate = 0x008E;
constexpr std::uint16_t kResultAmbientRate = 0x0090;
constexpr std::uint16_t kResultSigma = 0x0092;
constexpr std::uint16_t kResultDistance = 0x0096;
constexpr std::uint16_t kOscCalibrateValue = 0x00DE;
constexpr std::uint16_t kFirmwareStatus = 0x00E5;
constexpr std::uint16_t kModelId = 0x010F;

constexpr std::uint32_t kI2cTimeoutMs = 100;
constexpr std::uint32_t kBootTimeoutMs = 1000;
constexpr std::uint32_t kReceiverPollPeriodMs = 5;

// ST VL53L4CD ULD v1.0.0 default configuration, registers 0x002D-0x0087.
constexpr std::array<std::uint8_t, 91> kDefaultConfiguration{
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00,
    0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xCC, 0x07, 0x01, 0xF1, 0x05, 0x00, 0xA0,
    0x00, 0x80, 0x08, 0x38, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07,
    0x05, 0x06, 0x06, 0x00, 0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

constexpr std::array<std::uint8_t, 24> kRangeStatusMap{255, 255, 255, 5,   2,   4,   1,  7, 3,   0,   255, 255,
                                                       9,   13,  255, 255, 255, 255, 10, 6, 255, 255, 11,  12};

Vl53l4cd g_receiver_sensor{hi2c2};
Vl53l4cd::Result g_receiver_result{};
std::uint32_t g_receiver_last_poll_tick{0};
bool g_receiver_started{false};

void SetReceiverDriverStatus(Vl53l4cd::Status status) {
  g_vl53l4cd_state.driver_status = static_cast<std::uint8_t>(status);
}

}  // namespace

volatile Vl53l4cdReceiverState g_vl53l4cd_state{};

Vl53l4cd::Vl53l4cd(I2C_HandleTypeDef &i2c, std::uint8_t address) : i2c_(&i2c), address_(address) {}

Vl53l4cd::Status Vl53l4cd::Init() {
  if (i2c_ == nullptr || (address_ & 0x01U) != 0U) {
    return Status::kInvalidArgument;
  }

  HAL_Delay(10);
  Status status = WaitForBoot();
  if (status != Status::kOk) {
    return status;
  }

  status = ReadWord(kModelId, model_id_);
  if (status != Status::kOk) {
    return status;
  }
  if (model_id_ != kExpectedModelId) {
    return Status::kWrongDevice;
  }

  status = Write(0x002D, kDefaultConfiguration.data(), static_cast<std::uint16_t>(kDefaultConfiguration.size()));
  if (status != Status::kOk) {
    return status;
  }

  status = WriteByte(kSystemStart, 0x40);
  if (status != Status::kOk) {
    return status;
  }
  status = WaitForData(kBootTimeoutMs);
  if (status != Status::kOk) {
    return status;
  }

  if ((status = ClearInterrupt()) != Status::kOk || (status = Stop()) != Status::kOk ||
      (status = WriteByte(kVhvTimeout, 0x09)) != Status::kOk || (status = WriteByte(0x000B, 0x00)) != Status::kOk ||
      (status = WriteWord(0x0024, 0x0500)) != Status::kOk) {
    return status;
  }

  return SetRangeTiming(50, 0);
}

Vl53l4cd::Status Vl53l4cd::Start() {
  std::uint32_t inter_measurement = 0;
  Status status = ReadDword(kInterMeasurement, inter_measurement);
  if (status != Status::kOk) {
    return status;
  }

  status = WriteByte(kSystemStart, inter_measurement == 0 ? 0x21 : 0x40);
  if (status != Status::kOk) {
    return status;
  }
  status = WaitForData(kBootTimeoutMs);
  return status == Status::kOk ? ClearInterrupt() : status;
}

Vl53l4cd::Status Vl53l4cd::Stop() { return WriteByte(kSystemStart, 0x00); }

Vl53l4cd::Status Vl53l4cd::SetRangeTiming(std::uint32_t timing_budget_ms, std::uint32_t inter_measurement_ms) {
  if (timing_budget_ms < 10 || timing_budget_ms > 200 ||
      (inter_measurement_ms != 0 && inter_measurement_ms <= timing_budget_ms)) {
    return Status::kInvalidArgument;
  }

  std::uint16_t osc_frequency = 0;
  Status status = ReadWord(0x0006, osc_frequency);
  if (status != Status::kOk) {
    return status;
  }
  if (osc_frequency == 0) {
    return Status::kInvalidArgument;
  }

  std::uint32_t timing_budget_us = timing_budget_ms * 1000U;
  const std::uint32_t macro_period_us = (2304U * (0x40000000U / osc_frequency)) >> 6U;

  if (inter_measurement_ms == 0) {
    if ((status = WriteDword(kInterMeasurement, 0)) != Status::kOk) {
      return status;
    }
    timing_budget_us -= 2500U;
  } else {
    std::uint16_t clock_pll = 0;
    if ((status = ReadWord(kOscCalibrateValue, clock_pll)) != Status::kOk) {
      return status;
    }
    clock_pll &= 0x03FFU;
    const auto period =
        static_cast<std::uint32_t>(1.055F * static_cast<float>(inter_measurement_ms) * static_cast<float>(clock_pll));
    if ((status = WriteDword(kInterMeasurement, period)) != Status::kOk) {
      return status;
    }
    timing_budget_us = (timing_budget_us - 4300U) / 2U;
  }

  const auto encode_timeout = [timing_budget_us](std::uint32_t macro_period) {
    std::uint16_t ms_byte = 0;
    const std::uint32_t shifted_budget = timing_budget_us << 12U;
    const std::uint32_t tmp = macro_period * 16U;
    std::uint32_t ls_byte = ((shifted_budget + ((tmp >> 6U) >> 1U)) / (tmp >> 6U)) - 1U;
    while ((ls_byte & 0xFFFFFF00U) != 0U) {
      ls_byte >>= 1U;
      ++ms_byte;
    }
    return static_cast<std::uint16_t>((ms_byte << 8U) | (ls_byte & 0xFFU));
  };

  status = WriteWord(kRangeConfigA, encode_timeout(macro_period_us));
  if (status != Status::kOk) {
    return status;
  }

  const auto encode_timeout_b = [timing_budget_us](std::uint32_t macro_period) {
    std::uint16_t ms_byte = 0;
    const std::uint32_t shifted_budget = timing_budget_us << 12U;
    const std::uint32_t tmp = macro_period * 12U;
    std::uint32_t ls_byte = ((shifted_budget + ((tmp >> 6U) >> 1U)) / (tmp >> 6U)) - 1U;
    while ((ls_byte & 0xFFFFFF00U) != 0U) {
      ls_byte >>= 1U;
      ++ms_byte;
    }
    return static_cast<std::uint16_t>((ms_byte << 8U) | (ls_byte & 0xFFU));
  };
  return WriteWord(kRangeConfigB, encode_timeout_b(macro_period_us));
}

Vl53l4cd::Status Vl53l4cd::IsDataReady(bool &ready) {
  std::uint8_t mux = 0;
  std::uint8_t gpio = 0;
  Status status = ReadByte(kGpioHvMuxCtrl, mux);
  if (status != Status::kOk) {
    return status;
  }
  status = ReadByte(kGpioStatus, gpio);
  if (status != Status::kOk) {
    return status;
  }

  const std::uint8_t interrupt_polarity = (mux & 0x10U) != 0U ? 0U : 1U;
  ready = (gpio & 0x01U) == interrupt_polarity;
  return Status::kOk;
}

Vl53l4cd::Status Vl53l4cd::Read(Result &result) {
  std::uint8_t raw_status = 0;
  Status status = ReadByte(kResultRangeStatus, raw_status);
  if (status != Status::kOk) {
    return status;
  }
  raw_status &= 0x1FU;
  result.range_status = raw_status < kRangeStatusMap.size() ? kRangeStatusMap[raw_status] : 255;

  std::uint16_t raw = 0;
  if ((status = ReadWord(kResultSpadCount, raw)) != Status::kOk) {
    return status;
  }
  result.number_of_spads = raw / 256U;

  if ((status = ReadWord(kResultSignalRate, raw)) != Status::kOk) {
    return status;
  }
  result.signal_rate_kcps = raw * 8U;

  if ((status = ReadWord(kResultAmbientRate, raw)) != Status::kOk) {
    return status;
  }
  result.ambient_rate_kcps = raw * 8U;

  if ((status = ReadWord(kResultSigma, raw)) != Status::kOk) {
    return status;
  }
  result.sigma_mm = raw / 4U;

  if ((status = ReadWord(kResultDistance, result.distance_mm)) != Status::kOk) {
    return status;
  }
  return ClearInterrupt();
}

Vl53l4cd::Status Vl53l4cd::Poll(Result &result) {
  bool ready = false;
  const Status status = IsDataReady(ready);
  if (status != Status::kOk) {
    return status;
  }
  return ready ? Read(result) : Status::kNotReady;
}

Vl53l4cd::Status Vl53l4cd::WaitForBoot() {
  const std::uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < kBootTimeoutMs) {
    std::uint8_t firmware_status = 0;
    const Status status = ReadByte(kFirmwareStatus, firmware_status);
    if (status != Status::kOk) {
      return status;
    }
    if (firmware_status == 0x03) {
      return Status::kOk;
    }
    HAL_Delay(1);
  }
  return Status::kTimeout;
}

Vl53l4cd::Status Vl53l4cd::WaitForData(std::uint32_t timeout_ms) {
  const std::uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeout_ms) {
    bool ready = false;
    const Status status = IsDataReady(ready);
    if (status != Status::kOk) {
      return status;
    }
    if (ready) {
      return Status::kOk;
    }
    HAL_Delay(1);
  }
  return Status::kTimeout;
}

Vl53l4cd::Status Vl53l4cd::ClearInterrupt() { return WriteByte(kInterruptClear, 0x01); }

Vl53l4cd::Status Vl53l4cd::Read(std::uint16_t reg, std::uint8_t *data, std::uint16_t size) {
  return HAL_I2C_Mem_Read(i2c_, address_, reg, I2C_MEMADD_SIZE_16BIT, data, size, kI2cTimeoutMs) == HAL_OK
             ? Status::kOk
             : Status::kI2cError;
}

Vl53l4cd::Status Vl53l4cd::Write(std::uint16_t reg, const std::uint8_t *data, std::uint16_t size) {
  return HAL_I2C_Mem_Write(i2c_, address_, reg, I2C_MEMADD_SIZE_16BIT, const_cast<std::uint8_t *>(data), size,
                           kI2cTimeoutMs) == HAL_OK
             ? Status::kOk
             : Status::kI2cError;
}

Vl53l4cd::Status Vl53l4cd::ReadByte(std::uint16_t reg, std::uint8_t &value) { return Read(reg, &value, 1); }

Vl53l4cd::Status Vl53l4cd::ReadWord(std::uint16_t reg, std::uint16_t &value) {
  std::uint8_t data[2]{};
  const Status status = Read(reg, data, sizeof(data));
  if (status == Status::kOk) {
    value = static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[0]) << 8U) | data[1]);
  }
  return status;
}

Vl53l4cd::Status Vl53l4cd::ReadDword(std::uint16_t reg, std::uint32_t &value) {
  std::uint8_t data[4]{};
  const Status status = Read(reg, data, sizeof(data));
  if (status == Status::kOk) {
    value = (static_cast<std::uint32_t>(data[0]) << 24U) | (static_cast<std::uint32_t>(data[1]) << 16U) |
            (static_cast<std::uint32_t>(data[2]) << 8U) | data[3];
  }
  return status;
}

Vl53l4cd::Status Vl53l4cd::WriteByte(std::uint16_t reg, std::uint8_t value) { return Write(reg, &value, 1); }

Vl53l4cd::Status Vl53l4cd::WriteWord(std::uint16_t reg, std::uint16_t value) {
  const std::uint8_t data[]{
      static_cast<std::uint8_t>(value >> 8U),
      static_cast<std::uint8_t>(value & 0xFFU),
  };
  return Write(reg, data, sizeof(data));
}

Vl53l4cd::Status Vl53l4cd::WriteDword(std::uint16_t reg, std::uint32_t value) {
  const std::uint8_t data[]{
      static_cast<std::uint8_t>(value >> 24U),
      static_cast<std::uint8_t>(value >> 16U),
      static_cast<std::uint8_t>(value >> 8U),
      static_cast<std::uint8_t>(value),
  };
  return Write(reg, data, sizeof(data));
}

void Vl53l4cdReceiverInit() {
  Vl53l4cd::Status status = g_receiver_sensor.Init();
  g_vl53l4cd_state.model_id = g_receiver_sensor.model_id();
  SetReceiverDriverStatus(status);
  if (status != Vl53l4cd::Status::kOk) {
    return;
  }

  status = g_receiver_sensor.Start();
  SetReceiverDriverStatus(status);
  g_receiver_started = status == Vl53l4cd::Status::kOk;
  g_receiver_last_poll_tick = HAL_GetTick();
}

void Vl53l4cdReceiverPoll() {
  if (!g_receiver_started) {
    return;
  }

  const std::uint32_t now = HAL_GetTick();
  if ((now - g_receiver_last_poll_tick) < kReceiverPollPeriodMs) {
    return;
  }
  g_receiver_last_poll_tick = now;

  const Vl53l4cd::Status status = g_receiver_sensor.Poll(g_receiver_result);
  if (status == Vl53l4cd::Status::kNotReady) {
    return;
  }
  SetReceiverDriverStatus(status);
  if (status != Vl53l4cd::Status::kOk) {
    return;
  }

  g_vl53l4cd_state.range_status = g_receiver_result.range_status;
  g_vl53l4cd_state.distance_mm = g_receiver_result.distance_mm;
  g_vl53l4cd_state.signal_rate_kcps = g_receiver_result.signal_rate_kcps;
  g_vl53l4cd_state.ambient_rate_kcps = g_receiver_result.ambient_rate_kcps;
  g_vl53l4cd_state.sigma_mm = g_receiver_result.sigma_mm;
  g_vl53l4cd_state.sample_count = g_vl53l4cd_state.sample_count + 1U;
}

}  // namespace device
