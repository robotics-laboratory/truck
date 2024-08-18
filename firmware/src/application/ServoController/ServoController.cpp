#include <cstdio>
#include "ServoController.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEFAULT_SERVO_ANGLE     90.0f

void ServoController::task() {
    static int angle = 60;
    int value = 6;
    while (true){
        set_angle((float) angle, (float) (angle));
        angle += value;
        if (angle == 120 || angle == 60) {
            value = -value;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        //TODO Some logic to implement servo feedback
    }
}

int32_t ServoController::init() {
    // TODO Task creation
    BaseType_t status = xTaskCreate(startTaskImpl, "servo_task", 512, this, 2, &task_handle);
    if (status == pdTRUE) {
        is_initialized = true;
        vTaskResume(task_handle);
    } else {
        printf("Error сreating servo task\n");
    }
    return 0;
}

int32_t ServoController::set_angle(float left_servo_angle, float right_servo_angle) {
    left_servo.set_angle(left_servo_angle);
    right_servo.set_angle(right_servo_angle);
    return 0;
}

int32_t ServoController::enable(bool enable_servo) {
    int32_t status = 0;
    if (is_initialized){
        if ((enable_servo) && (!is_enabled)) {
            status |= left_servo.set_angle(DEFAULT_SERVO_ANGLE);
            status |= right_servo.set_angle(DEFAULT_SERVO_ANGLE);
            is_enabled = true;
        } else {
            status |= left_servo.stop();
            status |= right_servo.stop();
            is_enabled = false;
        }
    }
    return status;
}

