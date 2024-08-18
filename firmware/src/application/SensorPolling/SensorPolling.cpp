#include "FreeRTOS.h"
#include "task.h"

#include "SensorPolling.h"
#include <cstdio>
#include "fdcan.h"
#include "encoder_timer.h"
#include "pwm_servo.h"

void SensorPolling::task() {
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    static EncoderTimer &enc = EncoderTimer::get_instance(EncoderType::ID_0);
    enc.init();
    int prev_len = 0;
    static std::vector<int> enc_data;
    enc_data.reserve(50);

    static PWMServo& pwm = PWMServo::get_instance(PWMServoType::PWM_SERVO_1);
    float stepper_speed = 0.1;
    uint32_t timeout = HAL_GetTick() + 2000;
    while (true) {
       imu.update_values();
       imu.get_gyro_values(gyro_x, gyro_y, gyro_z);
       imu.get_accel_values(accel_x, accel_y, accel_z);
       gyro_updated = true;
       accel_updated = true;
       printf("Accel %d %d %d\n", (int)(accel_x*1000), (int)(accel_y*1000), (int)(accel_z*1000));
       printf("Gyro %d %d %d\n", (int)(gyro_x*100), (int)(gyro_y*100), (int)(gyro_z*100));
       uint8_t gyro_x_can = (uint8_t)(accel_x*1000);
    //    fdcan.transmit(1, &gyro_x_can, 4);
        if (HAL_GetTick() > timeout) {
            timeout = HAL_GetTick() + 2000;
            stepper_speed += 0.1;
            if (stepper_speed >= 1.0) {
                stepper_speed = 0.1;
            }
        }
        int us = 1000000 / (200 * stepper_speed * 4);
        pwm.set_us_impulse(us);
        enc.get_data(enc_data);
        if (enc_data.size()) {
            uint32_t sum = 0;
            for (int t : enc_data) {
                printf("%d ", t);
                sum += t;
            }
            printf("-- %d\r\n", sum);
            int speed = 1000000 * enc_data.size() / (20.0 * sum) * 100.0;
            printf("enc - %d - size %d\r\n", (int)(speed), enc_data.size());
            prev_len = (prev_len + enc_data.size()) % 50;
        } else {
//            printf("skip\r\n");
        }
        printf("%d\r\n", enc.get_data_size());
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

int32_t SensorPolling::init() {
    int32_t status = 0;
    if (!is_initialized) {
        BaseType_t task_status = xTaskCreate(startTaskImpl, "sensor_task", 1024, this, 1, &task_handle);
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

void SensorPolling::start() {
    vTaskResume(task_handle);
}

int32_t SensorPolling::get_gyro_data(float &x_axis, float &y_axis, float &z_axis) {
    int32_t status = 0;
    if (is_initialized) {
        x_axis = gyro_x;
        y_axis = gyro_y;
        z_axis = gyro_z;
        if (!gyro_updated) {
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}

int32_t SensorPolling::get_accel_data(float &x_axis, float &y_axis, float &z_axis) {
    int32_t status = 0;
    if (is_initialized) {
        x_axis = accel_x;
        y_axis = accel_y;
        z_axis = accel_z;
        if (!accel_updated) {
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}

int32_t SensorPolling::get_magn_data(float &x_axis, float &y_axis, float &z_axis) {
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
