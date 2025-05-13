#include <cstdio>
#include "ServoController.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system_clock.h"
#include <vector>
#include <tuple>

ServoController& ServoController::getInstance() {
    static ServoController _instance;
    return _instance;
}

ServoController::ServoContext& ServoController::get_ctx(ServoType type) {
    for (auto& p : servos_) {
        if (p.first == type) {
            return p.second;
        }
    }
}

void ServoController::process_servo(ServoType type) {
    auto &servo_ctx = get_ctx(type);

    float current_angle = servo_ctx.servo.get_angle();
    servo_ctx.speed = servo_ctx.speed * (1.0f - low_pass_speed_filter_coef) +
                      low_pass_speed_filter_coef * (current_angle - servo_ctx.prev_position) / 0.005f;
    servo_ctx.prev_position = current_angle;

    if (system_clock_get_tick() > servo_ctx.last_correction_timeout) { // timeout because servo has high latency
        if (std::abs(servo_ctx.speed) < 5.0f) { // if servo still moving fastly, dont try to correct it's position
            if (((servo_ctx.correction > 0) && ((servo_ctx.target_angle - current_angle) > 0.2)) || // undershooting when angle increasing
                ((servo_ctx.correction < 0) && ((current_angle - servo_ctx.target_angle) > 0.2))) { // undershooting when angle decreasing
                if (std::abs(servo_ctx.correction) < 10) {
                    servo_ctx.correction += 0.5f*(servo_ctx.target_angle - current_angle);
                }
            } else if (std::abs(servo_ctx.target_angle - current_angle) > 0.3) { //overshooting
                servo_ctx.correction = (servo_ctx.correction > 0) ? (-1 * correction_bias) : (correction_bias); // TODO: IDK solution for overshooting
                // printf("\n\novershoot -%s\n", (type == ServoType::SERVO_LEFT) ? "L" : "R");
            }
            servo_ctx.servo.set_angle(servo_ctx.target_angle + servo_ctx.correction);
            servo_ctx.last_correction_timeout = system_clock_get_tick() + 40; // 20 ms pause to wait for correction
        }
    }
    // printf("%d-%s-%s-%04d-%04d-%04d-%02d--",
    //         system_clock_get_tick(),
    //         (type == ServoType::SERVO_LEFT) ? "L" : "R",
    //         (system_clock_get_tick() > servo_ctx.last_correction_timeout) ? "work" : "skip",
    //         (int) std::abs(servo_ctx.speed * 10),
    //         (int) (current_angle * 10),
    //         (int) (servo_ctx.target_angle * 10),
    //         (int) (servo_ctx.correction * 10));
}

void ServoController::task() {

    while (true) {
        if (xSemaphoreTake(mutex, portMAX_DELAY) != pdFALSE) {
            process_servo(ServoType::SERVO_LEFT);
            process_servo(ServoType::SERVO_RIGHT);
            // printf("\n");
            // printf("%d --- %d\n", (int) (10 * left_servo->speed), (int) (10 * right_servo->speed));
        }
        xSemaphoreGive(mutex);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


uint32_t ServoController::init() {
    uint32_t status = 0;

    mutex = xSemaphoreCreateMutex();
    BaseType_t returned = xTaskCreate(startTaskImpl, "servo_task", 512, this, 2, &task_handle);

    if ((mutex != NULL) && (returned == pdTRUE)) {
        auto left_servo = get_ctx(ServoType::SERVO_LEFT);
        auto right_servo = get_ctx(ServoType::SERVO_RIGHT);
        if ((left_servo.servo.init() == 0) && (right_servo.servo.init() == 0)) {
            is_initialized = true;

            left_servo.prev_position = left_servo.servo.get_angle();
            right_servo.prev_position = right_servo.servo.get_angle();

            left_servo.servo.get_home_angles(left_servo.target_angle);
            right_servo.servo.get_home_angles(right_servo.target_angle);
            vTaskResume(task_handle);
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}


void ServoController::update_target_angle(ServoType type, float target_angle) {
    auto &servo_ctx = get_ctx(type);
    if (target_angle != servo_ctx.target_angle) {
        if (target_angle > servo_ctx.target_angle) {
            servo_ctx.correction = correction_bias;
        } else if (target_angle < servo_ctx.target_angle) {
            servo_ctx.correction = -correction_bias;
        }
        servo_ctx.target_angle = target_angle;
        servo_ctx.servo.set_angle(servo_ctx.target_angle + servo_ctx.correction);
        servo_ctx.last_correction_timeout = system_clock_get_tick() + 60;
        servo_ctx.speed = 0.0f;
    }
}

uint32_t ServoController::set_angle(float left_servo_angle, float right_servo_angle) {
    uint32_t status = 0;

    // printf("\nset_angle\n\n");

    if (is_initialized) {
        if (xSemaphoreTake(mutex, portMAX_DELAY) != pdFALSE) {
            update_target_angle(ServoType::SERVO_LEFT, left_servo_angle);
            update_target_angle(ServoType::SERVO_RIGHT, right_servo_angle);
            xSemaphoreGive(mutex);
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint32_t ServoController::enable(bool enable_servo) {
    int32_t status = 0;

    return status;
}

uint32_t ServoController::get_angle(uint16_t &left_servo_angle, uint16_t &right_servo_angle) {
    uint32_t status = 0;

    if (is_initialized) {
        left_servo_angle = get_ctx(ServoType::SERVO_LEFT).prev_position;
        right_servo_angle =  get_ctx(ServoType::SERVO_RIGHT).prev_position;
    } else {
        status = 1;
    }

    return status;
}