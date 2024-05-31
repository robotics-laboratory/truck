#include "simulator_2d/simulator_engine.h"

#include <boost/format.hpp>
#include <pybind11/pybind11.h>
#include <sstream>

namespace py = pybind11;
using namespace truck;

template<typename T>
std::string to_string(const T& obj);

template<>
std::string to_string<float>(const float& obj) {
    return boost::str(boost::format("%.5f") % obj);
}

template<>
std::string to_string<double>(const double& obj) {
    return boost::str(boost::format("%.5f") % obj);
}

template<>
std::string to_string<rclcpp::Time>(const rclcpp::Time& obj) {
    return boost::str(boost::format("Time(%1% ns)") % obj.nanoseconds());
}

template<>
std::string to_string<geom::Vec2>(const geom::Vec2& obj) {
    return boost::str(boost::format("Vec2(x=%s, y=%s)") % to_string(obj.x) % to_string(obj.y));
}

template<>
std::string to_string<geom::Vec3>(const geom::Vec3& obj) {
    return boost::str(
        boost::format("Vec3(x=%s, y=%s, z=%s)") % to_string(obj.x) % to_string(obj.y)
        % to_string(obj.z));
}

template<>
std::string to_string<geom::Angle>(const geom::Angle& obj) {
    return boost::str(boost::format("Angle(%.5f deg)") % obj.degrees());
}

template<>
std::string to_string<geom::AngleVec2>(const geom::AngleVec2& obj) {
    return boost::str(boost::format("AngleVec2(%s)") % to_string(obj.angle()));
}

template<>
std::string to_string<geom::Pose>(const geom::Pose& obj) {
    return boost::str(
        boost::format("Pose(pos=%s, dir=%s)") % to_string(obj.pos) % to_string(obj.dir));
}

template<>
std::string to_string<model::Steering>(const model::Steering& obj) {
    return boost::str(
        boost::format("Steering(left=%s, right=%s)") % to_string(obj.left) % to_string(obj.right));
}

template<>
std::string to_string<model::WheelVelocity>(const model::WheelVelocity& obj) {
    return boost::str(
        boost::format("WheelVelocity(rear_left=%s, rear_right=%s, front_left=%s, front_right=%s)")
        % to_string(obj.rear_left) % to_string(obj.rear_right) % to_string(obj.front_left)
        % to_string(obj.front_right));
}

template<>
std::string to_string<model::Twist>(const model::Twist& obj) {
    return boost::str(
        boost::format("Twist(curvature=%.5f, velocity=%.5f)") % obj.curvature % obj.velocity);
}

template<>
std::string to_string<std::vector<float>>(const std::vector<float>& vec) {
    std::ostringstream oss;
    for (const auto& val : vec) {
        oss << to_string(val) << " ";
    }

    std::string result = oss.str();
    if (!result.empty()) {
        result.pop_back();
    }

    return result;
}

template<>
std::string to_string<simulator::TruckState>(const simulator::TruckState& obj) {
    return boost::str(
        boost::format(
            "TruckState(time=%s, base_odom_pose=%s, current_steering=%s, "
            "target_steering=%s, base_odom_twist=%s, base_odom_linear_velocity=%s, "
            "base_odom_angular_velocity=%s, lidar_ranges=%s, current_motor_rps=%s, "
            "target_motor_rps=%s, gyro_angular_velocity=%s, accel_linear_acceleration=%s)")
        % to_string(obj.time()) % to_string(obj.odomBasePose()) % to_string(obj.currentSteering())
        % to_string(obj.targetSteering()) % to_string(obj.baseTwist())
        % to_string(obj.odomBaseLinearVelocity()) % to_string(obj.baseAngularVelocity())
        % to_string(obj.lidarRanges()) % to_string(obj.currentMotorRps())
        % to_string(obj.targetMotorRps()) % to_string(obj.gyroAngularVelocity())
        % to_string(obj.accelLinearAcceleration()));
}

std::unique_ptr<simulator::SimulatorEngine> create_simulator_engine(
    const model::Model& model, double integration_step, double precision) {
    return std::make_unique<simulator::SimulatorEngine>(
        std::make_unique<model::Model>(model), integration_step, precision);
}

PYBIND11_MODULE(pymodel, m) {
    m.doc() = "truck simulator_2d bindings";
    py::class_<rclcpp::Time>(m, "Time")
        .def_property_readonly("nanoseconds", &rclcpp::Time::nanoseconds)
        .def("__repr__", &to_string<rclcpp::Time>);
    py::class_<geom::Vec2>(m, "Vec2")
        .def_readwrite("x", &geom::Vec2::x)
        .def_readwrite("y", &geom::Vec2::y)
        .def("__repr__", &to_string<geom::Vec2>);
    py::class_<geom::Vec3>(m, "Vec3")
        .def_readwrite("x", &geom::Vec3::x)
        .def_readwrite("y", &geom::Vec3::y)
        .def_readwrite("z", &geom::Vec3::z)
        .def("__repr__", &to_string<geom::Vec2>);
    py::class_<geom::Angle>(m, "Angle")
        .def_property_readonly("radians", &geom::Angle::radians)
        .def_property_readonly("degrees", &geom::Angle::degrees)
        .def("__repr__", &to_string<geom::Angle>);
    py::class_<geom::AngleVec2>(m, "AngleVec2")
        .def_property_readonly("vec", &geom::AngleVec2::vec)
        .def_property_readonly("angle", &geom::AngleVec2::angle)
        .def("__repr__", &to_string<geom::AngleVec2>);
    py::class_<geom::Pose>(m, "Pose")
        .def_readonly("pos", &geom::Pose::pos)
        .def_readonly("dir", &geom::Pose::dir)
        .def("__repr__", &to_string<geom::Pose>);
    py::class_<model::Steering>(m, "Steering")
        .def_readonly("left", &model::Steering::left)
        .def_readonly("right", &model::Steering::right)
        .def("__repr__", &to_string<model::Steering>);
    py::class_<model::WheelVelocity>(m, "WheelVelocity")
        .def_readonly("rear_left", &model::WheelVelocity::rear_left)
        .def_readonly("rear_right", &model::WheelVelocity::rear_right)
        .def_readonly("front_left", &model::WheelVelocity::front_left)
        .def_readonly("front_right", &model::WheelVelocity::front_right)
        .def("__repr__", &to_string<model::WheelVelocity>);
    py::class_<model::Twist>(m, "Twist")
        .def(py::init<double, double>())
        .def_readonly("curvature", &model::Twist::curvature)
        .def_readonly("velocity", &model::Twist::velocity)
        .def("__repr__", &to_string<model::Twist>);
    py::class_<simulator::TruckState>(m, "TruckState")
        .def(
            "time",
            static_cast<rclcpp::Time (simulator::TruckState::*)() const>(
                &simulator::TruckState::time))
        .def(
            "odom_base_pose",
            static_cast<geom::Pose (simulator::TruckState::*)() const>(
                &simulator::TruckState::odomBasePose))
        .def(
            "current_steering",
            static_cast<model::Steering (simulator::TruckState::*)() const>(
                &simulator::TruckState::currentSteering))
        .def(
            "target_steering",
            static_cast<model::Steering (simulator::TruckState::*)() const>(
                &simulator::TruckState::targetSteering))
        .def(
            "base_twist",
            static_cast<model::Twist (simulator::TruckState::*)() const>(
                &simulator::TruckState::baseTwist))
        .def(
            "odom_base_linear_velocity",
            static_cast<geom::Vec2 (simulator::TruckState::*)() const>(
                &simulator::TruckState::odomBaseLinearVelocity))
        .def(
            "base_angular_velocity",
            static_cast<double (simulator::TruckState::*)() const>(
                &simulator::TruckState::baseAngularVelocity))
        .def(
            "lidar_ranges",
            static_cast<const std::vector<float>& (simulator::TruckState::*)() const>(
                &simulator::TruckState::lidarRanges))
        .def(
            "current_motor_rps",
            static_cast<double (simulator::TruckState::*)() const>(
                &simulator::TruckState::currentMotorRps))
        .def(
            "target_motor_rps",
            static_cast<double (simulator::TruckState::*)() const>(
                &simulator::TruckState::targetMotorRps))
        .def(
            "gyro_angular_velocity",
            static_cast<geom::Vec3 (simulator::TruckState::*)() const>(
                &simulator::TruckState::gyroAngularVelocity))
        .def(
            "accel_linear_acceleration",
            static_cast<geom::Vec3 (simulator::TruckState::*)() const>(
                &simulator::TruckState::accelLinearAcceleration))
        .def("time", py::overload_cast<const rclcpp::Time&>(&simulator::TruckState::time))
        .def(
            "odom_base_pose",
            py::overload_cast<const geom::Pose&>(&simulator::TruckState::odomBasePose))
        .def(
            "current_steering",
            py::overload_cast<const model::Steering&>(&simulator::TruckState::currentSteering))
        .def(
            "target_steering",
            py::overload_cast<const model::Steering&>(&simulator::TruckState::targetSteering))
        .def(
            "base_twist", py::overload_cast<const model::Twist&>(&simulator::TruckState::baseTwist))
        .def(
            "odom_base_linear_velocity",
            py::overload_cast<const geom::Vec2&>(&simulator::TruckState::odomBaseLinearVelocity))
        .def(
            "base_angular_velocity",
            py::overload_cast<double>(&simulator::TruckState::baseAngularVelocity))
        .def(
            "lidar_ranges",
            py::overload_cast<std::vector<float>>(&simulator::TruckState::lidarRanges))
        .def(
            "current_motor_rps", py::overload_cast<double>(&simulator::TruckState::currentMotorRps))
        .def("target_motor_rps", py::overload_cast<double>(&simulator::TruckState::targetMotorRps))
        .def(
            "gyro_angular_velocity",
            py::overload_cast<const geom::Vec3&>(&simulator::TruckState::gyroAngularVelocity))
        .def(
            "accel_linear_acceleration",
            py::overload_cast<const geom::Vec3&>(&simulator::TruckState::accelLinearAcceleration))
        .def("__repr__", &to_string<simulator::TruckState>);
    py::class_<simulator::SimulatorEngine>(m, "SimulatorEngine")
        .def_property_readonly("get_truck_state", &simulator::SimulatorEngine::getTruckState)
        .def("reset_base", &simulator::SimulatorEngine::resetBase)
        .def("reset_map", &simulator::SimulatorEngine::resetMap)
        .def("reset_map", &simulator::SimulatorEngine::eraseMap)
        .def(
            "set_base_control",
            py::overload_cast<double, double, double>(&simulator::SimulatorEngine::setBaseControl))
        .def(
            "set_base_control",
            py::overload_cast<double, double>(&simulator::SimulatorEngine::setBaseControl))
        .def("advance", &simulator::SimulatorEngine::advance);
    m.def(
        "create_simulator_engine",
        &create_simulator_engine,
        py::arg("model"),
        py::arg("integration_step"),
        py::arg("precision"));
}