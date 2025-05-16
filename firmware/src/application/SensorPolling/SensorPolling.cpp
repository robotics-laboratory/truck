#include "SensorPolling.h"

#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "COBS.h"

#include "system_clock.h"
#include "mag3110.h"
#include "hal_gpio.h"

#include "wheel_encoder.h"
#include "ServoController.h"
#include "Protocol.h"

SensorPolling& SensorPolling::getInstance() {
    static SensorPolling _instance;
    return _instance;
}

void SensorPolling::task() {
    static WheelEncoder& enc_left_front = WheelEncoder::get_instance(WheelType::LEFT_FRONT);
    static WheelEncoder& enc_right_front = WheelEncoder::get_instance(WheelType::RIGHT_FRONT);
    static Protocol& protocol = Protocol::getInstance();
    static MAG3110& magnitometr = MAG3110::getInstance();

    magnitometr.init();
    enc_left_front.init();
    enc_right_front.init();

    static ServoController& SC = ServoController::getInstance();
    uint16_t magn_x = 0;
    uint16_t magn_y = 0;
    uint16_t magn_z = 0;


    while (true) {
        float left_angle = 0;
        float right_angle = 0;
        SC.get_angle(left_angle, right_angle);
        protocol.transmit_servo_angles(left_angle, right_angle);

        float left_speed = enc_left_front.get_ticks_per_sec();
        float right_speed = enc_right_front.get_ticks_per_sec();
        protocol.transmit_servo_angles(left_speed, right_speed);

        magnitometr.get_magn_z(magn_x, magn_y, magn_z);
        protocol.transmit_magn_data(magn_x, magn_y, magn_z);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int32_t SensorPolling::init() {
    int32_t status = 0;
    if (!is_initialized) {
        BaseType_t task_status =
            xTaskCreate(startTaskImpl, "sensor_task", 1024, this, 1, &task_handle);
        if (task_status == pdTRUE) {
            vTaskSuspend(task_handle);
            //            imu.init();
            is_initialized = true;
        } else {
            printf("Error сreating sensor polling task\n");
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}

void SensorPolling::start() { vTaskResume(task_handle); }

int32_t SensorPolling::get_magn_data(float& x_axis, float& y_axis, float& z_axis) {
    int32_t status = 0;
    if (is_initialized) {
        x_axis = magn_x;
        y_axis = magn_y;
        z_axis = magn_z;
        if (!magn_updated) {
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}
