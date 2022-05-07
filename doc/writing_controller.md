## Writing controller

reference: https://github.com/ros-controls/ros2_controllers/blob/master/doc/writing_new_controller.rst

1. Create class, inheriting from `ControllerInterface`

2. Provide following 7 overriding methods:
    - `return_type init(const std::string & controller_name) override;`
    - `InterfaceConfiguration command_interface_configuration() const override;`
    - `InterfaceConfiguration state_interface_configuration() const override;`
    - `CallbackReturn on_configure(const State & previous_state) override;`
    - `CallbackReturn on_activate(const State & previous_state) override;`
    - `CallbackReturn on_deactivate(const State & previous_state) override;`
    - `return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;`

3. 