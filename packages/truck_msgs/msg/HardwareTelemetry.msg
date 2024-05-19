std_msgs/Header header

# CORE
float64 current_rps                # RPS
float64 target_rps                 # RPS
float64 battery_voltage            # Volts
float64 battery_current            # Amps

# WHEELS
float64 target_left_steering       # rad
float64 current_left_steering      # rad
float64 target_right_steering      # rad
float64 current_right_steering     # rad

float64 rear_left_wheel_velocity   # rad/s
float64 rear_right_wheel_velocity  # rad/s
float64 front_left_wheel_velocity  # rad/s
float64 front_right_wheel_velocity # rad/s
