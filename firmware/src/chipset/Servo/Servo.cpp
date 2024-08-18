#include <cstdio>
#include "Servo.h"

int32_t Servo::init() {
    switch (type_) {
        case ServoType::SERVO_LEFT:
            max_angle = 110;
            min_angle = 70;
            break;
        case ServoType::SERVO_RIGHT:
            max_angle = 110;
            min_angle = 70;
            break;
    }
    is_initialized = true;
    // TODO Self-test??
    return 0;
}

int32_t Servo::set_angle(float angle) {
    int32_t status = 0;
    if (is_initialized) {
        if (pwm.initialized()) {
            if ((angle <= max_angle) && (angle >= min_angle)) {
                uint32_t us = 500 + (uint32_t) ((2500.0 - 500.0) / 180.0 * angle);
                pwm.set_us_impulse(us);
            } else {
                status = 3;
            }
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }
    return status;
}

float Servo::get_angle() {
    // TODO return measured from potentiometr angle
    return servo_angle;
}

int32_t Servo::stop() {
    int32_t status = 0;
    if (is_initialized) {
        status = pwm.stop();
    } else {
        status = 1;
    }

    return status;
}
