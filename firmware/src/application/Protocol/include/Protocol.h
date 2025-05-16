#ifndef TRUCK_HW_APPLICATION_PROTOCOL_INCLUDE_PROTOCOL_H_
#define TRUCK_HW_APPLICATION_PROTOCOL_INCLUDE_PROTOCOL_H_

#include <cstdint>
#include <cstring>
#include <array>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "fdcan.h"

#define TASK_NOTIFY_RX_DATA_SIGNAL                                         (uint32_t)(0x01)

#pragma pack(push, 1)
typedef struct {
  float accel_x_axis;
  float accel_y_axis;
  float accel_z_axis;
  float gyro_x_axis;
  float gyro_y_axis;
  float gyro_z_axis;
  float magn_x_axis;
  float magn_y_axis;
  float magn_z_axis;
  bool data_updated;
} protocol_inform_imu_data;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  float motor_velocity;
  float front_left_wheel_velocity;
  float front_right_wheel_velocity;
  bool data_updated;
} protocol_inform_motion_data;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  float target_speed;
  bool is_enable;
} protocol_request_motor;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  int32_t status;
} protocol_response_motor;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  float left_servo_angle;
  float right_servo_angle;
  bool is_enable;
} protocol_request_servo;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
  int32_t status;
} protocol_response_servo;
#pragma pack(pop)

#define CALLBACKS_MAX_NUM 20
#define REQUEST_BASE_ID_OFFSET 0
#define INFORM_BASE_ID_OFFSET CALLBACKS_MAX_NUM

#define SERVO_TARGET_ANGLES_MSG_TAG      (REQUEST_BASE_ID_OFFSET + 1)

#define ENCODERS_SPEED_MSG_TAG           (INFORM_BASE_ID_OFFSET + 1)
#define SERVO_MEASURED_ANGLES_MSG_TAG    (INFORM_BASE_ID_OFFSET + 2)
#define MAGN_Z_MSG_TAG                   (INFORM_BASE_ID_OFFSET + 3)

typedef void(*request_callback_t)(const uint8_t *request_data, uint32_t request_data_size,
                                  uint8_t *response_data, uint32_t &response_data_size);

class Protocol {
 private:
  #pragma pack(push, 1)
  typedef struct {
    uint8_t left_angle;
    uint8_t right_angle;
  } request_target_servo_angles;
  #pragma pack(pop)

  #pragma pack(push, 1)
  typedef struct {
    float left_angle;
    float right_angle;
  } telemetry_servo_measured_angles;
  #pragma pack(pop)

  #pragma pack(push, 1)
  typedef struct {
    float left_front_speed;
    float right_front_speed;
  } telemetry_encoders_speed;
  #pragma pack(pop)

  #pragma pack(push, 1)
  typedef struct {
    uint16_t magn_x;
    uint16_t magn_y;
    uint16_t magn_z;
  } telemetry_magnitometer_data;
  #pragma pack(pop)

  FDCAN& communication_unit;
  static TaskHandle_t task_handle;

  void task();
  static void startTaskImpl(void *_this) {
      ((Protocol *) _this)->task();
  }

  std::array<request_callback_t, CALLBACKS_MAX_NUM> request_callback;
  void bind(uint32_t tag, request_callback_t callback);

  static void on_new_request();

  Protocol() : communication_unit(FDCAN::getInstance()) {
      request_callback.fill(nullptr);
  };
  ~Protocol() {};
  Protocol(const Protocol &obj) = delete;
  Protocol &operator=(const Protocol &obj) = delete;

 public:
  static Protocol &getInstance();
  void init();
  void start();
  void transmit_encoders_speed(float left_speed, float right_speed);
  void transmit_servo_angles(float left_angle, float right_angle);
  void transmit_magn_data(uint16_t x_axis, uint16_t y_axis, uint16_t z_axis);
};
#endif //TRUCK_HW_APPLICATION_PROTOCOL_INCLUDE_PROTOCOL_H_
