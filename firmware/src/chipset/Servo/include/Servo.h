#ifndef TRUCK_HW_CHIPSET_SERVO_INCLUDE_SERVO_H_
#define TRUCK_HW_CHIPSET_SERVO_INCLUDE_SERVO_H_

#include <cstdint>

#include "pwm_servo.h"
#include "ads1115.h"

enum class ServoType{
  SERVO_LEFT,
  SERVO_RIGHT
};

class Servo {
 private:
  ServoType type_;
  PWMServo &pwm;
  ADS1115 &adc;
  ADCServoType adc_type;
  bool is_initialized = false;

  float max_angle = 270.0f;
  float home_angle = 135.0f;
  float min_angle = 0.0f;

  float adc_to_ang_poly1;
  float adc_to_ang_poly0;

  static PWMServo &get_pwm_instance(ServoType servo_type_) {
      switch (servo_type_) {
          case ServoType::SERVO_LEFT: return PWMServo::get_instance(PWMServoType::PWM_SERVO_1);
          case ServoType::SERVO_RIGHT: return PWMServo::get_instance(PWMServoType::PWM_SERVO_2);
      }
  }

  static ADS1115 &get_adc_instance(ServoType servo_type_) {
      switch (servo_type_) {
          case ServoType::SERVO_LEFT: return ADS1115::get_instance(ADCServoType::ADC_SERVO_1);
          case ServoType::SERVO_RIGHT: return ADS1115::get_instance(ADCServoType::ADC_SERVO_2);
      }
  }

  explicit Servo(ServoType type) : pwm(get_pwm_instance(type)), adc(get_adc_instance(type)), type_(type) {};

  Servo(const Servo &obj) = delete;
  Servo &operator=(const Servo &obj) = delete;

 public:
  Servo(Servo &&obj) noexcept = default;
  Servo &operator=(Servo &&obj) noexcept = default;

  ~Servo() = default;

  static Servo &get_instance(ServoType type);

  uint32_t init();
  uint32_t calibrate(void);
  uint32_t set_angle(float angle);
  float get_angle(void);
  void get_home_angles(float &ret_home_angle);
  uint32_t stop();
};

#endif //TRUCK_HW_CHIPSET_SERVO_INCLUDE_SERVO_H_
