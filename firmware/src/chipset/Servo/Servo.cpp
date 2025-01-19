#include "Servo.h"

#include "board.h"

Servo& Servo::get_instance(ServoType type) {
    static std::unordered_map<ServoType, Servo *> instances;
    auto it = instances.find(type);
    if (it == instances.end()) {
        instances[type] = new Servo(type);
    }
    return *instances[type];
}

uint32_t Servo::init() {
    uint32_t status = 0;

    switch (type_) {
        case ServoType::SERVO_LEFT:
            max_angle = 150.0f;
            min_angle = 35.0f;
            break;
        case ServoType::SERVO_RIGHT:
            max_angle = 150.0f;
            min_angle = 35.0f;
            break;
    }

    if (pwm.init() == 0) {
        if (adc.init() == 0) {
            adc.start_measurment();
            switch (type_) {
                case ServoType::SERVO_LEFT:
                    adc_type = ADCServoType::ADC_SERVO_1;
                    break;
                case ServoType::SERVO_RIGHT:
                    adc_type = ADCServoType::ADC_SERVO_2;
                    break;
                default:
                    status = 3;
                    break;
            }
            if (status == 0) {
                is_initialized = true;
            }
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint32_t Servo::calibrate(void) {
    if (is_initialized == true) {
        for (int a = 90; a > min_angle; a--) {
            set_angle(a);
            board_delay_ms(4);
        }
        board_delay_ms(100);
        for (int i = 0; i < 5; ++i) {
            // printf("%d\n", get_angle());
        }
        // printf("---------\n");
        for (int a = min_angle; a <= max_angle; a++) {
            set_angle(a);
            // printf("%d\n", get_angle());
            board_delay_ms(3);
        }
    }
    return 0;
}

uint32_t Servo::set_angle(float angle) {
    uint32_t status = 0;

    if (is_initialized == true) {
        if ((angle <= max_angle) && (angle >= min_angle)) {
            uint32_t us = 500 + (uint32_t) ((2500.0f - 500.0f) / 180.0f * angle);
            pwm.set_us_impulse(us);
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

    return status;
}

uint16_t Servo::get_angle() {
    uint16_t raw_value = 0.0f;

    if (adc.get_value(adc_type, raw_value) != 0) {
        raw_value = 0;
    }

    return raw_value;
}

uint32_t Servo::stop() {
    uint32_t status = 0;

    if (is_initialized) {
        pwm.stop();
        adc.stop_measurment();
    } else {
        status = 1;
    }

    return status;
}
