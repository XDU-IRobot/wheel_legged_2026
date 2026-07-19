/*
 * Minimal STM32 HAL driver for the ST VL53L4CD Ultra Lite Driver flow.
 *
 * The sensor uses 16-bit register addresses. Public I2C addresses are 7-bit;
 * the address is shifted internally when calling STM32 HAL.
 */

#ifndef APP_COMMON_DEVICE_VL53L4CD_HPP_
#define APP_COMMON_DEVICE_VL53L4CD_HPP_

#include "librm/core/typedefs.hpp"
#include "librm/device/device.hpp"
#include "stm32h7xx_hal.h"

namespace rm::device {

class Vl53l4cd : public Device {
 public:
  static constexpr u8 kDefaultAddress = 0x29;
  static constexpr u16 kExpectedModelId = 0xEBAA;

  enum class Error : u8 {
    kOk = 0,
    kNotReady,
    kI2cError,
    kTimeout,
    kWrongDevice,
    kInvalidArgument,
    kNotStarted,
  };

  struct Config {
    u8 address{kDefaultAddress};
    u32 timing_budget_ms{10};
    u32 inter_measurement_ms{0};
    // The application schedules Poll() at 100 Hz. Keep this guard shorter than
    // 10 ms so millisecond tick quantization cannot accidentally skip a whole
    // scheduled sample.
    u32 poll_period_ms{5};
    u32 i2c_timeout_ms{100};
    u32 boot_timeout_ms{1000};
  };

  struct Measurement {
    u8 range_status{255};
    u16 distance_mm{0};
    u16 ambient_rate_kcps{0};
    u16 signal_rate_kcps{0};
    u16 number_of_spads{0};
    u16 sigma_mm{0};

    [[nodiscard]] bool valid() const { return range_status == 0; }
  };

  Vl53l4cd() = delete;
  explicit Vl53l4cd(I2C_HandleTypeDef &i2c);
  Vl53l4cd(I2C_HandleTypeDef &i2c, const Config &config);

  /**
   * Initialize the sensor and start ranging.
   *
   * AVDD must already be present and XSHUT must be high. Pulling XSHUT low
   * resets the sensor, so Begin() must be called again after it is raised.
   */
  Error Begin();
  Error Stop();

  /** Poll for one new sample. Calls faster than Config::poll_period_ms are ignored. */
  Error Poll();

  Error SetRangeTiming(u32 timing_budget_ms, u32 inter_measurement_ms = 0);

  [[nodiscard]] const Measurement &measurement() const { return measurement_; }
  [[nodiscard]] bool data_valid() const { return measurement_.valid(); }
  [[nodiscard]] Error last_error() const { return last_error_; }
  [[nodiscard]] u16 model_id() const { return model_id_; }
  [[nodiscard]] u32 sample_count() const { return sample_count_; }
  [[nodiscard]] u32 last_sample_tick_ms() const { return last_sample_tick_ms_; }
  [[nodiscard]] u32 poll_count() const { return poll_count_; }
  [[nodiscard]] u32 i2c_error_count() const { return i2c_error_count_; }
  [[nodiscard]] bool ranging() const { return ranging_; }
  [[nodiscard]] u8 address() const { return config_.address; }

 private:
  Error Init();
  Error Start();
  Error ReadMeasurement();
  Error IsDataReady(bool &ready);
  Error WaitForBoot();
  Error WaitForData(u32 timeout_ms);
  Error ClearInterrupt();
  Error RecordError(Error error);

  Error Read(u16 reg, u8 *data, u16 size);
  Error Write(u16 reg, const u8 *data, u16 size);
  Error ReadByte(u16 reg, u8 &value);
  Error ReadWord(u16 reg, u16 &value);
  Error ReadDword(u16 reg, u32 &value);
  Error WriteByte(u16 reg, u8 value);
  Error WriteWord(u16 reg, u16 value);
  Error WriteDword(u16 reg, u32 value);

  static u16 EncodeTimeout(u32 timing_budget_us, u32 macro_period_us, u32 multiplier);

  I2C_HandleTypeDef *i2c_;
  Config config_;
  Measurement measurement_{};
  Error last_error_{Error::kNotStarted};
  u16 model_id_{0};
  u32 sample_count_{0};
  u32 last_sample_tick_ms_{0};
  u32 last_poll_tick_{0};
  u32 poll_count_{0};
  u32 i2c_error_count_{0};
  u8 interrupt_polarity_{0};
  bool interrupt_polarity_valid_{false};
  bool ranging_{false};
};

}  // namespace rm::device

#endif  // APP_COMMON_DEVICE_VL53L4CD_HPP_
