#include <body.h>
#include <serial.h>
#include <time.h>

namespace body {

void Body::init() {}

void Body::loop() {
    as5047p.update();

    uint16_t angle = as5047p.get_angle();
    uint16_t is_good_magnet = as5047p.is_magnetic_field_good();
    int64_t number_of_rotations = as5047p.get_number_of_rotations();

    serial::print(
        &husart1,
        "Angle %d, is good magnet %d, rotations %d\n",
        angle,
        is_good_magnet,
        number_of_rotations);

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
