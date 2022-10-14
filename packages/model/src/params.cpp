#include <model/params.h>

namespace truck::model {

namespace {

SteeringLimit toSteeringLimits(const YAML::Node& node) {
    return {
        geom::Angle::fromDegrees(node["inner"].as<double>()),
        geom::Angle::fromDegrees(node["outer"].as<double>())};
}

template <typename T>
Limits<double> toLimits(const YAML::Node& node) {
    return {node["min"].as<double>(), node["max"].as<double>()};
}

}  // namespace

using namespace geom::literals;

WheelBase::WheelBase(const YAML::Node& node)
    : width(node["width"].as<double>())
    , length(node["length"].as<double>())
    , base_to_rear(node["base_to_rear"].as<double>()) {
    BOOST_VERIFY(width > 0);
    BOOST_VERIFY(length > 0);
    BOOST_VERIFY(base_to_rear > 0);
    BOOST_VERIFY(length > base_to_rear);
}

VehicleLimits::VehicleLimits(const YAML::Node& node)
    : max_abs_curvature(node["max_abs_curvature"].as<double>())
    , steering{toSteeringLimits(node["steering"])}
    , velocity{toLimits<double>(node["velocity"])}
    , acceleration{toLimits<double>(node["acceleration"])} {
    BOOST_VERIFY(max_abs_curvature >= 0);
    BOOST_VERIFY(0_deg <= steering.inner && steering.inner < 90_deg);
    BOOST_VERIFY(0_deg <= steering.outer && steering.outer < 90_deg);
}

ServoAngles::ServoAngles(const YAML::Node& node)
    : left(geom::Angle::fromDegrees(node["left"].as<double>()))
    , right(geom::Angle::fromDegrees(node["right"].as<double>())) {
    BOOST_VERIFY(0_deg <= left && left < 180_deg);
    BOOST_VERIFY(0_deg <= right && right < 180_deg);
}

Params::Params(const YAML::Node& node)
    : wheel_base(node["wheel_base"])
    , limits(node["limits"])
    , wheel_radius(node["wheel_radius"].as<double>())
    , gear_ratio(node["gear_ratio"].as<double>())
    , servo_home_angles(node["servo_home_angles"]) {
    BOOST_VERIFY(wheel_radius > 0);
    BOOST_VERIFY(gear_ratio > 0);
}

Params::Params(const std::string& config_path) : Params(YAML::LoadFile(config_path)) {}

}  // namespace truck::model