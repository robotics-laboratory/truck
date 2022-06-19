import dataclasses as dc
from pathlib import Path
import math
import yaml


class Const:
    MAIN_CONFIG_PATH = "config.yaml"
    STEERING_MAP_PATH = "steering.csv"
    ODRIVE_MAIN_AXIS = "axis1"
    ZERO_CURVATURE_EPS = 1e-3
    TEENSY_SERIAL_PORT = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_9670820-if00"
    TEENSY_SERIAL_SPEED = 500000
    TEENSY_SERIAL_TIMEOUT = 1
    MSGPACK_RECV_INDEX = 2
    MSGPACK_SEND_INDEX = 1
    RATE_LIMIT_FREQUENCY = 10


@dc.dataclass
class Config:
    wheel_base_width: float
    wheel_base_length: float
    linear_speed_to_rpm: float
    max_curvature: float
    max_linear_speed: float
    linear_acceleration: float
    left_servo_home: float
    right_servo_home: float

    @classmethod
    def load_from_yaml(cls, path=Const.MAIN_CONFIG_PATH):
        with open(path) as file:
            data = yaml.load(file, yaml.Loader)
            valid_keys = set(x.name for x in dc.fields(cls))
            valid_data = {k: v for k, v in data.items() if k in valid_keys}
            extra_keys = data.keys() - valid_data.keys()
            if extra_keys:
                print("Extra keys ignored:", list(extra_keys))
            return cls(**valid_data)

    def save_to_yaml(self, path):
        prev_data = {}
        if Path(path).exists():
            with open(path) as file:
                prev_data = yaml.load(file, yaml.Loader)
                assert isinstance(prev_data, dict)
        with open(path, "w") as file:
            data = {**prev_data, **dc.asdict(self)}
            yaml.dump(data, file, sort_keys=False)


@dc.dataclass
class SpeedCommand:
    ratio: float
    linear: float
    rpm: float

    def __init__(self, config: Config, ratio: float = None, linear: float = None, rpm: float = None):
        if ratio is not None:
            self.ratio = max(-1, min(1, ratio))
            self.linear = config.max_linear_speed * self.ratio
            self.rpm = self.linear * config.linear_speed_to_rpm
        elif linear is not None:
            self.linear = max(-config.max_linear_speed, min(config.max_linear_speed, linear))
            self.ratio = self.linear / config.max_linear_speed
            self.rpm = self.linear * config.linear_speed_to_rpm
        elif rpm is not None:
            max_rpm = config.max_linear_speed * config.linear_speed_to_rpm
            self.rpm = max(-max_rpm, min(max_rpm, rpm))
            self.linear = self.rpm / config.linear_speed_to_rpm
            self.ratio = self.linear / config.max_linear_speed
        else:
            raise ValueError

    def __str__(self):
        return f"{self.ratio:.1%}% | {self.linear:.1f} m/s | {self.rpm:.1f} rpm"
    

class SteeringCommand:
    ratio: float
    radius: float
    curvature: float

    def __init__(self, config: Config, ratio: float = None, radius: float = None, curvature: float = None):
        if ratio is not None:
            self.ratio = max(-1, min(1, ratio))
            self.curvature = self.ratio * config.max_curvature
            self.radius = 1 / self.curvature if abs(self.curvature) > Const.ZERO_CURVATURE_EPS else float("+inf")
        elif radius is not None:
            min_radius = 1 / config.max_curvature
            self.radius = math.copysign(max(min_radius, abs(radius)), radius)
            if abs(self.radius) > 1 / Const.ZERO_CURVATURE_EPS:
                self.radius = float("+inf")
            self.curvature = 1 / self.radius
            self.ratio = self.curvature / config.max_curvature
        elif curvature is not None:
            raise NotImplementedError
        else:
            raise ValueError

    def __str__(self):
        return f"{self.ratio:.1%} | R={self.radius:.1f} | C={self.curvature:.3f}"


@dc.dataclass
class State:
    motor_enabled: bool = False
    error_state: bool = False
    error_description: str = ""
    target_speed: SpeedCommand = None
    current_speed: SpeedCommand = None
    target_steering: SteeringCommand = None
    current_steering: SteeringCommand = None

    def __init__(self, config: Config):
        self.target_speed = self.current_speed = SpeedCommand(config, ratio=0)
        self.target_steering = self.target_steering = SteeringCommand(config, ratio=0)
