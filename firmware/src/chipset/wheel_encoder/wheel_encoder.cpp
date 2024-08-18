#include "wheel_encoder.h"

int WheelEncoder::calculate_speed() {
    if (!is_initialized) {
        return 1;
    }

    encoder_timer_handle.get_data(encoder_ticks);
    if (encoder_ticks.empty()) {
        last_speed = 0.0f;
        return 0;
    }
    uint32_t sum = 0;
    for (auto val : encoder_ticks) {
        sum += val;
    }
    float encoder_step_duration_micros = encoder_timer_handle.get_tick_len_micros() * sum / encoder_ticks.size();
    last_speed =  (2 * pi * wheel_radius_mm) /
        (steps_per_turn * encoder_step_duration_micros);

    return 0;
}
