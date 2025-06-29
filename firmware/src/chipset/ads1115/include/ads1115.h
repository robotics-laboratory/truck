#ifndef TRUCK_HW_CHIPSET_ADS1115_INCLUDE_ADS1115_H_
#define TRUCK_HW_CHIPSET_ADS1115_INCLUDE_ADS1115_H_

#include <cstdint>

#include "i2c2.h"

enum class ADCServoType : uint8_t {
  ADC_SERVO_1 = 1,
  ADC_SERVO_2
};

class ADS1115 {
 private:

  I2C &i2c_handle = I2C::getInstance();
  uint8_t i2c_address = 0x00;
  const uint8_t i2c_timeout = 10;
  bool is_initialized = false;

  uint32_t config(void);

  explicit ADS1115(ADCServoType id);

  ADS1115(const ADS1115 &obj) = delete;
  ADS1115 &operator=(const ADS1115 &obj) = delete;
 public:
  ADS1115(ADS1115 &&obj) noexcept = default;
  ADS1115 &operator=(ADS1115 &&obj) noexcept = default;

  ~ADS1115() = default;

  uint32_t init();
  static ADS1115 &get_instance(ADCServoType type);
  uint32_t read_value(uint16_t& value);
};

#endif /* TRUCK_HW_CHIPSET_ADS1115_INCLUDE_ADS1115_H_ */