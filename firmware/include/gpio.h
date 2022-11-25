#pragma once

#include <stm32f1xx.h>

namespace gpio {

inline void write_pin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state) {
    if (state == GPIO_PIN_RESET) {
        port->BSRR = (uint32_t)pin << 16u;
    } else {
        port->BSRR = pin;
    }
}

}  // namespace gpio