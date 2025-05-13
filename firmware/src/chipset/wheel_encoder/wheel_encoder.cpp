#include "wheel_encoder.h"

#include <array>
#include <memory>

#include "system_clock.h"

WheelEncoder& WheelEncoder::get_instance(WheelType type) {
    static std::array<std::pair<WheelType, std::unique_ptr<WheelEncoder>>, 4> instances{
        std::make_pair(WheelType::LEFT_FRONT,  std::make_unique<WheelEncoder>(WheelEncoder(WheelType::LEFT_FRONT))),
        std::make_pair(WheelType::RIGHT_FRONT, std::make_unique<WheelEncoder>(WheelEncoder(WheelType::RIGHT_FRONT))),
        std::make_pair(WheelType::LEFT_REAR,   std::make_unique<WheelEncoder>(WheelEncoder(WheelType::LEFT_REAR))),
        std::make_pair(WheelType::RIGHT_REAR,  std::make_unique<WheelEncoder>(WheelEncoder(WheelType::RIGHT_REAR))),
    };

    for (auto& p : instances) {
        if (p.first == type) {
            return *p.second;
        }
    }
}

uint32_t WheelEncoder::init(void) {
    uint32_t status = 0;

    encoder_ticks.resize(max_data_size);

    if (encoder_ticks.capacity() >= max_data_size) {
        if (encoder_timer_handle.init() == 0) {
            is_initialized = true;
        } else {
            status = 1;
        }
    } else {
        status = 2;
    }
    return status;
}

void WheelEncoder::low_pas_filter_apply(float raw_speed) {
    current_speed = low_pass_coef * raw_speed + (1 - low_pass_coef) * current_speed;
}

float WheelEncoder::get_ticks_per_sec(void) {
    if (is_initialized) {
        encoder_timer_handle.get_data(encoder_ticks);
        if (encoder_ticks.empty() && ((system_clock_get_tick() - last_tick_ts) > idle_timeout_ms)) {
            current_speed = 0.0f;
        } else if (encoder_ticks.empty() == false){
            for (auto val : encoder_ticks) {
                if (val > 2000) { // 10 rps of wheel with cpr == 50
                    low_pas_filter_apply(1000000.0f / val / encoder_timer_handle.get_tick_len_micros());
                }
            }
            last_tick_ts = system_clock_get_tick();
        }
    } else {
        current_speed = 0.0f;
    }

    return current_speed;
}
