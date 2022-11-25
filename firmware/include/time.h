#include <stm32f1xx.h>

namespace time {

uint32_t get_us() {
    uint32_t m = HAL_GetTick();
    uint32_t tms = SysTick->LOAD + 1;
    uint32_t u = tms - SysTick->VAL;
    return (m * 1000 + (u * 1000) / tms);
}

inline uint32_t get_ms() { return HAL_GetTick(); }

}  // namespace time