/*
 * Minimal STM32 HAL driver for the ST VL53L4CD Ultra Lite Driver flow.
 *
 * The sensor's LPn pin must be held high before Init() is called. The default
 * 8-bit I2C address used by ST documentation is 0x52 (7-bit address 0x29).
 */

#ifndef APP_COMMON_DEVICE_VL53L4CD_HPP_
#define APP_COMMON_DEVICE_VL53L4CD_HPP_

#include <cstdint>

#include "stm32h7xx_hal.h"

namespace device {

class Vl53l4cd {
 public:
  static constexpr std::uint8_t kDefaultAddress = 0x52;
  static constexpr std::uint16_t kExpectedModelId = 0xEBAA;

  enum class Status : std::uint8_t {
    kOk = 0,
    kNotReady,
    kI2cError,
    kTimeout,
    kWrongDevice,
    kInvalidArgument,
  };

  struct Result {
    std::uint8_t range_status{255};
    std::uint16_t distance_mm{0};
    std::uint16_t ambient_rate_kcps{0};
    std::uint16_t signal_rate_kcps{0};
    std::uint16_t number_of_spads{0};
    std::uint16_t sigma_mm{0};

    [[nodiscard]] bool valid() const { return range_status == 0; }
  };

  explicit Vl53l4cd(I2C_HandleTypeDef &i2c, std::uint8_t address = kDefaultAddress);

  Status Init();
  Status Start();
  Status Stop();
  Status SetRangeTiming(std::uint32_t timing_budget_ms, std::uint32_t inter_measurement_ms = 0);

  Status IsDataReady(bool &ready);
  Status Read(Result &result);
  Status Poll(Result &result);

  [[nodiscard]] std::uint16_t model_id() const { return model_id_; }

 private:
  Status WaitForBoot();
  Status WaitForData(std::uint32_t timeout_ms);
  Status ClearInterrupt();

  Status Read(std::uint16_t reg, std::uint8_t *data, std::uint16_t size);
  Status Write(std::uint16_t reg, const std::uint8_t *data, std::uint16_t size);
  Status ReadByte(std::uint16_t reg, std::uint8_t &value);
  Status ReadWord(std::uint16_t reg, std::uint16_t &value);
  Status ReadDword(std::uint16_t reg, std::uint32_t &value);
  Status WriteByte(std::uint16_t reg, std::uint8_t value);
  Status WriteWord(std::uint16_t reg, std::uint16_t value);
  Status WriteDword(std::uint16_t reg, std::uint32_t value);

  I2C_HandleTypeDef *i2c_;
  std::uint8_t address_;
  std::uint16_t model_id_{0};
};

struct Vl53l4cdReceiverState {
  std::uint8_t driver_status{0};
  std::uint8_t range_status{255};
  std::uint16_t model_id{0};
  std::uint16_t distance_mm{0};
  std::uint16_t signal_rate_kcps{0};
  std::uint16_t ambient_rate_kcps{0};
  std::uint16_t sigma_mm{0};
  std::uint32_t sample_count{0};
};

extern volatile Vl53l4cdReceiverState g_vl53l4cd_state;

void Vl53l4cdReceiverInit();
void Vl53l4cdReceiverPoll();

}  // namespace device

#endif  // APP_COMMON_DEVICE_VL53L4CD_HPP_
