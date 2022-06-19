from pyPS4Controller.controller import Controller
from backend.hardware import Config, State, Hardware


class Gamepad(Controller):
    def __init__(self, hardware: Hardware, **kwargs):
        super().__init__(**kwargs)
        self.hardware = hardware

    def on_R2_press(self, value):
        self.hardware.soft_reset()

    def on_L1_press(self):
        self.hardware.enable()

    def on_R1_press(self):
        self.hardware.set_speed(0)
        self.hardware.disable()

    def on_L3_left(self, value):
        self.hardware.set_steering(-value / 32768)

    def on_L3_x_at_rest(self):
        self.hardware.set_steering(0)

    def on_L3_right(self, value):
        self.hardware.set_steering(-value / 32768)

    def on_R3_up(self, value):
        ratio = -value / 32768
        self.hardware.set_speed(ratio)

    def on_R3_y_at_rest(self):
        self.hardware.set_speed(0)

    def on_R3_down(self, value):
        ratio = -value / 32768
        self.hardware.set_speed(ratio)


if __name__ == "__main__":
    import logging
    logging.basicConfig(level="INFO")
    config = Config.load_from_yaml()
    state = State(config)
    hardware = Hardware(config, state)
    
    try:
        gamepad = Gamepad(
            hardware,
            interface="/dev/input/js0",
            connecting_using_ds4drv=False,
        )
        gamepad.listen()
    except KeyboardInterrupt:
        hardware.disable()
    finally:
        hardware.disable()
