#include "Servo.h"

#include <array>
#include <memory>

#include "system_clock.h"

Servo& Servo::get_instance(ServoType type) {
    static std::array<std::pair<ServoType, std::unique_ptr<Servo>>, 2> instances{
        std::make_pair(ServoType::SERVO_LEFT,  std::make_unique<Servo>(Servo(ServoType::SERVO_LEFT))),
        std::make_pair(ServoType::SERVO_RIGHT, std::make_unique<Servo>(Servo(ServoType::SERVO_RIGHT)))
    };

    for (auto& p : instances) {
        if (p.first == type) {
            return *p.second;
        }
    }
}

uint32_t Servo::init() {
    uint32_t status = 0;

    switch (type_) {
        case ServoType::SERVO_LEFT:
            max_angle = 99.0f + 45.0f; //98.0f + 48.0f; //135.0f + 48.0f + 1.0f;
            home_angle = 99.0f; // 98.0f - 2.0f;
            min_angle = 99.0f - 42.0f; //98.0f - 39.0f; //135.0f - 62.0f - 1.0f;
            adc_to_ang_poly1 = -0.01793f;
            adc_to_ang_poly0 = 293.7f;
            break;
        case ServoType::SERVO_RIGHT:
            max_angle = 172.0f + 36.0f; //135.0f + 62.0f + 1.0f;
            home_angle = 172.0f - 5.0f; //172.0f - 4.0f;
            min_angle = 172.0f - 51.0f; // 135.0f - 48.0f - 1.0f;
            adc_to_ang_poly1 = -0.01802f;
            adc_to_ang_poly0 = 294.2f;
            break;
    }

    if (pwm.init() == 0) {
        if (adc.init() == 0) {
            is_initialized = true;
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint32_t Servo::calibrate(void) {
    set_angle(home_angle);
    return 0;
}

uint32_t Servo::set_angle(float angle) {
    uint32_t status = 0;

    if (is_initialized == true) {
        if ((angle <= max_angle) && (angle >= min_angle)) {
            uint32_t us = 500 + (uint32_t) ((2500.0f - 500.0f) / 270.0f * angle);
            pwm.set_us_impulse(us);
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

float Servo::get_angle() {
    float angle = 0.0f;
    uint16_t raw_value = 0;

    if (adc.read_value(raw_value) == 0) {
        angle = raw_value * adc_to_ang_poly1 + adc_to_ang_poly0;

        if ((angle < -10.0f) || (angle > 280.0f)) {
            angle = 0.0f;
        }
    }

    return angle;
}

void Servo::get_home_angles(float &ret_home_angle) {
    ret_home_angle = home_angle;
}

uint32_t Servo::stop() {
    uint32_t status = 0;

    if (is_initialized) {
        pwm.stop();
    } else {
        status = 1;
    }

    return status;
}
