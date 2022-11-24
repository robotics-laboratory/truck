#include <as5047p.h>
#include <body.h>
#include <serial.h>
#include <stm32f1xx.h>

namespace body {

void Body::init() {
}

void Body::loop() {
  uint16_t angle = as5047p.get_uncompressed_angle();
  uint16_t is_good_magnet = as5047p.is_magnetic_field_good();

  serial::print(&husart1, "Angle %d, is good magnet %d\n", angle, is_good_magnet);

  if (++iteration % 30 == 0) {
    serial::print(&husart1, "reset angle\n");
    as5047p.set_zero_angle();
  }

  led_pin_state = !led_pin_state;
  if (led_pin_state) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PinState::GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PinState::GPIO_PIN_RESET);
  }
  HAL_Delay(100);
}

}  // namespace body

body::Body body_ins;
