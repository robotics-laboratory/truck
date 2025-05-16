#ifndef TRUCK_HW_APPLICATION_SERVOCONTROLLER_INCLUDE_SERVOCONTROLLER_H_
#define TRUCK_HW_APPLICATION_SERVOCONTROLLER_INCLUDE_SERVOCONTROLLER_H_

#include <array>

#include "FreeRTOS.h"
#include "task.h"

#include "Servo.h"


class ServoController {
 private:
  static constexpr float correction_bias = 0.1f;
  static constexpr float low_pass_speed_filter_coef = 0.4f;

  struct ServoContext {
    Servo &servo;
    float speed = 0.0f;
    float correction = 0.0f;
    float prev_position = 0.0f;
    float target_angle = 0.0f;
    uint32_t last_correction_timeout = 0;
  };

  bool is_initialized = false;
  bool is_enabled = false;

  TaskHandle_t task_handle;
  SemaphoreHandle_t mutex;

  std::array<std::pair<ServoType, ServoContext>, 2> servos_;

  ServoController() : servos_
      ({std::make_pair(ServoType::SERVO_LEFT, ServoContext{Servo::get_instance(ServoType::SERVO_LEFT), {}}),
        std::make_pair(ServoType::SERVO_RIGHT, ServoContext{Servo::get_instance(ServoType::SERVO_RIGHT), {}})}) {
  };
  ~ServoController() {};
  ServoController(const ServoController &obj) = delete;
  ServoController &operator=(const ServoController &obj) = delete;

  void task();

  static void startTaskImpl(void *_this) {
      ((ServoController *) _this)->task();
  }

  ServoContext& get_ctx(ServoType type);
  void process_servo(ServoType type);
  void update_target_angle(ServoType type, float target_angle);

 public:
  static ServoController &getInstance();

  uint32_t init();

  uint32_t set_angle(float left_servo_angle, float right_servo_angle);
  uint32_t enable(bool is_enable);

  uint32_t get_angle(float &left_servo_angle, float &right_servo_angle);
};
#endif //TRUCK_HW_APPLICATION_SERVOCONTROLLER_INCLUDE_SERVOCONTROLLER_H_
