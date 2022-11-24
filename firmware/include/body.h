#pragma once

#include <as5047p.h>
#include <stm32f1xx.h>

extern USART_HandleTypeDef husart1;
extern SPI_HandleTypeDef hspi2;

namespace body {

class Body {
 public:
  Body() : as5047p(hspi2) {
  }

  void init();

  void loop();

 private:
  as5047p::AS5047p as5047p;
  bool led_pin_state = false;
  uint32_t iteration = 0;
};

}  // namespace body