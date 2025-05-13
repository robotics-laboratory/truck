#ifndef TRUCK_HW_BOARD_BOARD_H_
#define TRUCK_HW_BOARD_BOARD_H_

#include "stdint.h"

void system_clock_init();
uint32_t system_clock_get_tick(void);
void system_clock_delay_ticks(uint32_t delay_ticks);

#endif //TRUCK_HW_BOARD_BOARD_H_
