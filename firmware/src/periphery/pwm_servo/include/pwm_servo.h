#ifndef TRUCK_HW_PERIPHERY_PWM_SERVO_INCLUDE_PWM_SERVO_H_
#define TRUCK_HW_PERIPHERY_PWM_SERVO_INCLUDE_PWM_SERVO_H_

#include <unordered_map>
#include "stm32g4xx_ll_tim.h"


enum class PWMServoType : uint8_t {
  PWM_SERVO_1,
  PWM_SERVO_2
};

class PWMServo {
 private:
  PWMServoType type_;
  bool is_initialized = false;
  static bool is_common_tim_initialized;
  const TIM_TypeDef *common_timer_handle = TIM3;
  static constexpr uint32_t TIM_PRESCALER = 143;
  static constexpr uint32_t TIM_AUTORELOAD = 20000;
  uint32_t timer_channel;
  int32_t init();

  PWMServo(PWMServoType init_type);
  ~PWMServo() {};
  PWMServo(const PWMServo &obj) = delete;
  PWMServo &operator=(const PWMServo &obj) = delete;

 public:
  static PWMServo &get_instance(PWMServoType type) {
      static std::unordered_map<PWMServoType, PWMServo *> instances;
      auto it = instances.find(type);
      if (it == instances.end()) {
          instances[type] = new PWMServo(type);
      }
      return *instances[type];
  }

  int32_t set_us_impulse(uint32_t us);
  int32_t stop();
  bool initialized(){
      return is_initialized;
  }
};
#endif //TRUCK_HW_PERIPHERY_PWM_SERVO_INCLUDE_PWM_SERVO_H_
