#include "common/device/vl53l4cd.hpp"

#include <array>

namespace rm::device {
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

// ST VL53L4CD ULD v1.0.0 default configuration, registers 0x002D-0x0087.
constexpr std::array<std::uint8_t, 91> kDefaultConfiguration{
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00,
    0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
    0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xCC, 0x07, 0x01, 0xF1, 0x05, 0x00, 0xA0,
    0x00, 0x80, 0x08, 0x38, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07,
    0x05, 0x06, 0x06, 0x00, 0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

constexpr std::array<std::uint8_t, 24> kRangeStatusMap{255, 255, 255, 5,   2,   4,   1,  7, 3,   0,   255, 255,
                                                       9,   13,  255, 255, 255, 255, 10, 6, 255, 255, 11,  12};

// Result registers are contiguous from RESULT_RANGE_STATUS (0x0089) through
// RESULT_DISTANCE (0x0097). Read them in one transaction instead of six.
constexpr std::size_t kResultBlockSize = kResultDistance + 2U - kResultRangeStatus;
constexpr std::size_t kRangeStatusOffset = 0U;
constexpr std::size_t kSpadCountOffset = kResultSpadCount - kResultRangeStatus;
constexpr std::size_t kSignalRateOffset = kResultSignalRate - kResultRangeStatus;
constexpr std::size_t kAmbientRateOffset = kResultAmbientRate - kResultRangeStatus;
constexpr std::size_t kSigmaOffset = kResultSigma - kResultRangeStatus;
constexpr std::size_t kDistanceOffset = kResultDistance - kResultRangeStatus;

u16 ReadBigEndianWord(const std::array<u8, kResultBlockSize> &data, const std::size_t offset) {
  return static_cast<u16>((static_cast<u16>(data[offset]) << 8U) | data[offset + 1U]);
}

}  // namespace

Vl53l4cd::Vl53l4cd(I2C_HandleTypeDef &i2c) : Vl53l4cd(i2c, Config{}) {}

Vl53l4cd::Vl53l4cd(I2C_HandleTypeDef &i2c, const Config &config) : i2c_(&i2c), config_(config) {}

Vl53l4cd::Error Vl53l4cd::Begin() {
  measurement_ = {};
  model_id_ = 0;
  sample_count_ = 0;
  last_sample_tick_ms_ = 0;
  poll_count_ = 0;
  i2c_error_count_ = 0;
  interrupt_polarity_valid_ = false;
  ranging_ = false;

  Error error = Init();
  if (error == Error::kOk) {
    error = Start();
  }
  return RecordError(error);
}

Vl53l4cd::Error Vl53l4cd::Init() {
  if (i2c_ == nullptr || config_.address > 0x7FU || config_.i2c_timeout_ms == 0 || config_.boot_timeout_ms == 0) {
    return Error::kInvalidArgument;
  }

  HAL_Delay(10);
  Error error = WaitForBoot();
  if (error != Error::kOk) {
    return error;
  }

  error = ReadWord(kModelId, model_id_);
  if (error != Error::kOk) {
    return error;
  }
  if (model_id_ != kExpectedModelId) {
    return Error::kWrongDevice;
  }

  error = Write(0x002D, kDefaultConfiguration.data(), static_cast<u16>(kDefaultConfiguration.size()));
  if (error != Error::kOk) {
    return error;
  }

  // GPIO interrupt polarity is configured by the default register block and
  // remains unchanged while ranging. Cache it so normal polling only needs to
  // read GPIO_STATUS.
  u8 mux = 0;
  error = ReadByte(kGpioHvMuxCtrl, mux);
  if (error != Error::kOk) {
    return error;
  }
  interrupt_polarity_ = (mux & 0x10U) != 0U ? 0U : 1U;
  interrupt_polarity_valid_ = true;

  error = WriteByte(kSystemStart, 0x40);
  if (error != Error::kOk) {
    return error;
  }
  error = WaitForData(config_.boot_timeout_ms);
  if (error != Error::kOk) {
    return error;
  }

  if ((error = ClearInterrupt()) != Error::kOk || (error = Stop()) != Error::kOk ||
      (error = WriteByte(kVhvTimeout, 0x09)) != Error::kOk || (error = WriteByte(0x000B, 0x00)) != Error::kOk ||
      (error = WriteWord(0x0024, 0x0500)) != Error::kOk) {
    return error;
  }

  return SetRangeTiming(config_.timing_budget_ms, config_.inter_measurement_ms);
}

Vl53l4cd::Error Vl53l4cd::Start() {
  std::uint32_t inter_measurement = 0;
  Error error = ReadDword(kInterMeasurement, inter_measurement);
  if (error != Error::kOk) {
    return error;
  }

  error = WriteByte(kSystemStart, inter_measurement == 0 ? 0x21 : 0x40);
  if (error != Error::kOk) {
    return error;
  }
  error = WaitForData(config_.boot_timeout_ms);
  if (error == Error::kOk) {
    error = ClearInterrupt();
  }
  ranging_ = error == Error::kOk;
  last_poll_tick_ = HAL_GetTick();
  return error;
}

Vl53l4cd::Error Vl53l4cd::Stop() {
  const Error error = WriteByte(kSystemStart, 0x00);
  if (error == Error::kOk) {
    ranging_ = false;
  }
  return RecordError(error);
}

Vl53l4cd::Error Vl53l4cd::SetRangeTiming(std::uint32_t timing_budget_ms, std::uint32_t inter_measurement_ms) {
  if (timing_budget_ms < 10 || timing_budget_ms > 200 ||
      (inter_measurement_ms != 0 && inter_measurement_ms <= timing_budget_ms)) {
    return Error::kInvalidArgument;
  }

  std::uint16_t osc_frequency = 0;
  Error error = ReadWord(0x0006, osc_frequency);
  if (error != Error::kOk) {
    return error;
  }
  if (osc_frequency == 0) {
    return Error::kInvalidArgument;
  }

  std::uint32_t timing_budget_us = timing_budget_ms * 1000U;
  const std::uint32_t macro_period_us = (2304U * (0x40000000U / osc_frequency)) >> 6U;

  if (inter_measurement_ms == 0) {
    if ((error = WriteDword(kInterMeasurement, 0)) != Error::kOk) {
      return error;
    }
    timing_budget_us -= 2500U;
  } else {
    std::uint16_t clock_pll = 0;
    if ((error = ReadWord(kOscCalibrateValue, clock_pll)) != Error::kOk) {
      return error;
    }
    clock_pll &= 0x03FFU;
    const auto period =
        static_cast<std::uint32_t>(1.055F * static_cast<float>(inter_measurement_ms) * static_cast<float>(clock_pll));
    if ((error = WriteDword(kInterMeasurement, period)) != Error::kOk) {
      return error;
    }
    timing_budget_us = (timing_budget_us - 4300U) / 2U;
  }

  error = WriteWord(kRangeConfigA, EncodeTimeout(timing_budget_us, macro_period_us, 16U));
  if (error != Error::kOk) {
    return error;
  }
  error = WriteWord(kRangeConfigB, EncodeTimeout(timing_budget_us, macro_period_us, 12U));
  if (error == Error::kOk) {
    config_.timing_budget_ms = timing_budget_ms;
    config_.inter_measurement_ms = inter_measurement_ms;
  }
  return error;
}

u16 Vl53l4cd::EncodeTimeout(u32 timing_budget_us, u32 macro_period_us, u32 multiplier) {
  u16 exponent = 0;
  const u32 shifted_budget = timing_budget_us << 12U;
  const u32 period = macro_period_us * multiplier;
  u32 mantissa = ((shifted_budget + ((period >> 6U) >> 1U)) / (period >> 6U)) - 1U;
  while ((mantissa & 0xFFFFFF00U) != 0U) {
    mantissa >>= 1U;
    ++exponent;
  }
  return static_cast<u16>((exponent << 8U) | (mantissa & 0xFFU));
}

Vl53l4cd::Error Vl53l4cd::IsDataReady(bool &ready) {
  if (!interrupt_polarity_valid_) {
    u8 mux = 0;
    const Error error = ReadByte(kGpioHvMuxCtrl, mux);
    if (error != Error::kOk) {
      return error;
    }
    interrupt_polarity_ = (mux & 0x10U) != 0U ? 0U : 1U;
    interrupt_polarity_valid_ = true;
  }

  u8 gpio = 0;
  const Error error = ReadByte(kGpioStatus, gpio);
  if (error != Error::kOk) {
    return error;
  }

  ready = (gpio & 0x01U) == interrupt_polarity_;
  return Error::kOk;
}

Vl53l4cd::Error Vl53l4cd::ReadMeasurement() {
  Measurement next{};
  std::array<u8, kResultBlockSize> result{};
  Error error = Read(kResultRangeStatus, result.data(), static_cast<u16>(result.size()));
  if (error != Error::kOk) {
    return error;
  }

  u8 raw_status = result[kRangeStatusOffset];
  raw_status &= 0x1FU;
  next.range_status = raw_status < kRangeStatusMap.size() ? kRangeStatusMap[raw_status] : 255;
  u16 raw = ReadBigEndianWord(result, kSpadCountOffset);
  next.number_of_spads = raw / 256U;
  raw = ReadBigEndianWord(result, kSignalRateOffset);
  next.signal_rate_kcps = raw * 8U;
  raw = ReadBigEndianWord(result, kAmbientRateOffset);
  next.ambient_rate_kcps = raw * 8U;
  raw = ReadBigEndianWord(result, kSigmaOffset);
  next.sigma_mm = raw / 4U;
  next.distance_mm = ReadBigEndianWord(result, kDistanceOffset);
  if ((error = ClearInterrupt()) != Error::kOk) {
    return error;
  }

  measurement_ = next;
  last_sample_tick_ms_ = HAL_GetTick();
  ++sample_count_;
  return Error::kOk;
}

Vl53l4cd::Error Vl53l4cd::Poll() {
  if (!ranging_) {
    return RecordError(Error::kNotStarted);
  }

  const u32 now = HAL_GetTick();
  if ((now - last_poll_tick_) < config_.poll_period_ms) {
    return Error::kNotReady;
  }
  last_poll_tick_ = now;
  ++poll_count_;

  bool ready = false;
  Error error = IsDataReady(ready);
  if (error == Error::kOk && ready) {
    error = ReadMeasurement();
  } else if (error == Error::kOk) {
    return Error::kNotReady;
  }
  return RecordError(error);
}

Vl53l4cd::Error Vl53l4cd::WaitForBoot() {
  const u32 start = HAL_GetTick();
  while ((HAL_GetTick() - start) < config_.boot_timeout_ms) {
    u8 firmware_status = 0;
    const Error error = ReadByte(kFirmwareStatus, firmware_status);
    if (error != Error::kOk) {
      return error;
    }
    if (firmware_status == 0x03) {
      return Error::kOk;
    }
    HAL_Delay(1);
  }
  return Error::kTimeout;
}

Vl53l4cd::Error Vl53l4cd::WaitForData(u32 timeout_ms) {
  const u32 start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeout_ms) {
    bool ready = false;
    const Error error = IsDataReady(ready);
    if (error != Error::kOk) {
      return error;
    }
    if (ready) {
      return Error::kOk;
    }
    HAL_Delay(1);
  }
  return Error::kTimeout;
}

Vl53l4cd::Error Vl53l4cd::ClearInterrupt() { return WriteByte(kInterruptClear, 0x01); }

Vl53l4cd::Error Vl53l4cd::RecordError(Error error) {
  if (error == Error::kNotReady) {
    return error;
  }
  last_error_ = error;
  ReportStatus(error == Error::kOk ? Device::kOk : Device::kFault);
  return error;
}

Vl53l4cd::Error Vl53l4cd::Read(u16 reg, u8 *data, u16 size) {
  const u16 hal_address = static_cast<u16>(config_.address) << 1U;
  if (HAL_I2C_Mem_Read(i2c_, hal_address, reg, I2C_MEMADD_SIZE_16BIT, data, size, config_.i2c_timeout_ms) == HAL_OK) {
    return Error::kOk;
  }
  ++i2c_error_count_;
  return Error::kI2cError;
}

Vl53l4cd::Error Vl53l4cd::Write(u16 reg, const u8 *data, u16 size) {
  const u16 hal_address = static_cast<u16>(config_.address) << 1U;
  if (HAL_I2C_Mem_Write(i2c_, hal_address, reg, I2C_MEMADD_SIZE_16BIT, const_cast<u8 *>(data), size,
                        config_.i2c_timeout_ms) == HAL_OK) {
    return Error::kOk;
  }
  ++i2c_error_count_;
  return Error::kI2cError;
}

Vl53l4cd::Error Vl53l4cd::ReadByte(u16 reg, u8 &value) { return Read(reg, &value, 1); }

Vl53l4cd::Error Vl53l4cd::ReadWord(u16 reg, u16 &value) {
  u8 data[2]{};
  const Error error = Read(reg, data, sizeof(data));
  if (error == Error::kOk) {
    value = static_cast<u16>((static_cast<u16>(data[0]) << 8U) | data[1]);
  }
  return error;
}

Vl53l4cd::Error Vl53l4cd::ReadDword(u16 reg, u32 &value) {
  u8 data[4]{};
  const Error error = Read(reg, data, sizeof(data));
  if (error == Error::kOk) {
    value = (static_cast<u32>(data[0]) << 24U) | (static_cast<u32>(data[1]) << 16U) |
            (static_cast<u32>(data[2]) << 8U) | data[3];
  }
  return error;
}

Vl53l4cd::Error Vl53l4cd::WriteByte(u16 reg, u8 value) { return Write(reg, &value, 1); }

Vl53l4cd::Error Vl53l4cd::WriteWord(u16 reg, u16 value) {
  const u8 data[]{static_cast<u8>(value >> 8U), static_cast<u8>(value)};
  return Write(reg, data, sizeof(data));
}

Vl53l4cd::Error Vl53l4cd::WriteDword(u16 reg, u32 value) {
  const u8 data[]{static_cast<u8>(value >> 24U), static_cast<u8>(value >> 16U), static_cast<u8>(value >> 8U),
                  static_cast<u8>(value)};
  return Write(reg, data, sizeof(data));
}

}  // namespace rm::device
