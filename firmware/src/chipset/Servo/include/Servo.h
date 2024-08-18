//
// Created by Георгий Сенин on 05.04.2024.
//

#ifndef TRUCK_HW_CHIPSET_SERVO_INCLUDE_SERVO_H_
#define TRUCK_HW_CHIPSET_SERVO_INCLUDE_SERVO_H_

#include <cstdint>
#include "pwm_servo.h"

enum class ServoType{
  SERVO_LEFT,
  SERVO_RIGHT
};

class Servo {
 private:
  float servo_angle = 0.0f;
  ServoType type_;
  PWMServo &pwm;
  bool is_initialized = false;

  float max_angle = 180;
  float min_angle = 0;
  int32_t init();
  static PWMServo &get_pwm_instance(ServoType servo_type_) {
      switch (servo_type_) {
          case ServoType::SERVO_LEFT: return PWMServo::get_instance(PWMServoType::PWM_SERVO_1);
          case ServoType::SERVO_RIGHT: return PWMServo::get_instance(PWMServoType::PWM_SERVO_2);
      }
  }
  Servo(ServoType type) : pwm(get_pwm_instance(type)) {
      type_ = type;
      init();
  };

  ~Servo() {};
  Servo(const Servo &obj) = delete;
  Servo &operator=(const Servo &obj) = delete;

 public:
  static Servo &get_instance(ServoType type) {
      static std::unordered_map<ServoType, Servo *> instances;
      auto it = instances.find(type);
      if (it == instances.end()) {
          instances[type] = new Servo(type);
      }
      return *instances[type];
  }

  int32_t set_angle(float angle);
  float get_angle();
  int32_t stop();
};

#endif //TRUCK_HW_CHIPSET_SERVO_INCLUDE_SERVO_H_
