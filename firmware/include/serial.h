#pragma once

#include <stm32f1xx.h>

#include <cstdio>

namespace serial {

template <typename... T>
int print(USART_HandleTypeDef* husart, const char* format, T... args) {
    const int max_msg_size = 100;
    uint8_t msg[max_msg_size];
    int msg_size = snprintf((char*)msg, max_msg_size, format, args...);
    HAL_USART_Transmit(husart, msg, msg_size, 100);

    return msg_size;
}

}  // namespace serial