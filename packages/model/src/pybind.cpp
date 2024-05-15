#include <pybind11/pybind11.h>
#include "model/model.h"
#include <boost/format.hpp>

namespace py = pybind11;
using namespace truck;

template<typename T>
std::string to_string(const T& obj);

template<>
std::string to_string<double>(const double& obj) {
    return boost::str(boost::format("%.5d") % obj);
}

template<>
std::string to_string<geom::Angle>(const geom::Angle& obj) {
    return boost::str(boost::format("Angle(%.1d deg)") % obj.degrees());
}

template<>
std::string to_string<model::Steering>(const model::Steering& obj) {
    return boost::str(
        boost::format("Steering(left=%s, right=%s)") % to_string(obj.left) % to_string(obj.right));
}

template<>
std::string to_string<model::WheelVelocity>(const model::WheelVelocity& obj) {
    return boost::str(
        boost::format("WheelVelocity(left=%s, right=%s)") % to_string(obj.left) %
        to_string(obj.right));
}

template<>
std::string to_string<model::Twist>(const model::Twist& obj) {
    return boost::str(
        boost::format("Twist(curvature=%.5d, velocity=%.5d)") % obj.curvature % obj.velocity);
}

template<>
std::string to_string<model::ServoAngles>(const model::ServoAngles& obj) {
    return boost::str(
        boost::format("ServoAngles(left=%s, right=%s)") % to_string(obj.left) %
        to_string(obj.right));
}

template<typename T>
void bind_limits_class(py::module& m, const std::string& name) {
    using Class = Limits<T>;
    py::class_<Class>(m, name.c_str())
        .def_readonly("min", &Class::min)
        .def_readonly("max", &Class::max)
        .def("__repr__", [name](const Class& obj) {
            return boost::str(
                boost::format("%s(min=%s, max=%s)") % name % to_string<T>(obj.min) %
                to_string<T>(obj.max));
        });
}

PYBIND11_MODULE(pymodel, m) {
    m.doc() = "truck model bindings";
    py::class_<geom::Angle>(m, "Angle")
        .def_property_readonly("radians", &geom::Angle::radians)
        .def_property_readonly("degrees", &geom::Angle::degrees)
        .def("__repr__", &to_string<geom::Angle>);
    // TODO: Expose more methods if needed in the future
    py::class_<model::Steering>(m, "Steering")
        .def_readonly("left", &model::Steering::left)
        .def_readonly("right", &model::Steering::right)
        .def("__repr__", &to_string<model::Steering>);
    py::class_<model::WheelVelocity>(m, "WheelVelocity")
        .def_readonly("left", &model::WheelVelocity::left)
        .def_readonly("right", &model::WheelVelocity::right)
        .def("__repr__", &to_string<model::WheelVelocity>);
    py::class_<model::Twist>(m, "Twist")
        .def(py::init<double, double>())
        .def_readonly("curvature", &model::Twist::curvature)
        .def_readonly("velocity", &model::Twist::velocity)
        .def("__repr__", &to_string<model::Twist>);
    py::class_<model::ServoAngles>(m, "ServoAngles")
        .def_readonly("left", &model::ServoAngles::left)
        .def_readonly("right", &model::ServoAngles::right)
        .def("__repr__", &to_string<model::ServoAngles>);
    bind_limits_class<double>(m, "FloatLimits");
    bind_limits_class<geom::Angle>(m, "AngleLimits");
    py::class_<model::Model>(m, "Model")
        .def(py::init<const std::string&>())
        .def_property_readonly("base_max_abs_curvature", &model::Model::baseMaxAbsCurvature)
        .def_property_readonly("left_steering_limits", &model::Model::leftSteeringLimits)
        .def_property_readonly("right_steering_limits", &model::Model::rightSteeringLimits)
        .def_property_readonly("base_velocity_limits", &model::Model::baseVelocityLimits)
        .def_property_readonly("max_acceleration", &model::Model::baseMaxAcceleration)
        .def_property_readonly("max_acceleration", &model::Model::baseMaxDeceleration)
        .def_property_readonly("servo_home_angles", &model::Model::servoHomeAngles)
        .def_property_readonly("gear_ratio", &model::Model::gearRatio)
        .def("base_to_rear_twist", &model::Model::baseToRearTwist)
        .def("rear_twist_to_steering", &model::Model::rearTwistToSteering)
        .def("rear_twist_to_wheel_velocity", &model::Model::rearTwistToWheelVelocity)
        .def("linear_velocity_to_motor_rps", &model::Model::linearVelocityToMotorRPS)
        .def("motor_rps_to_linear_velocity", &model::Model::motorRPStoLinearVelocity);
}
