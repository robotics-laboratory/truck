#pragma once

#include "util/yaml_serializer.hpp"

#include <string>

namespace model {

struct Model {
    YAML_SERIALIZABLE_STRUCT(Model)

    YAML_DECLARE_FIELD(double, max_velocity);
    YAML_DECLARE_FIELD(double, max_acceleration);
    YAML_DECLARE_FIELD(double, max_decceleration);
    YAML_DECLARE_FIELD(double, truck_length);
    YAML_DECLARE_FIELD(double, truck_width);
    YAML_DECLARE_FIELD(double, wheel_radius);
    YAML_DECLARE_FIELD(double, max_wheels_angle);
public:
    Model() = default;
    Model(const std::string &config_file) {
        YAML::Node config = YAML::LoadFile(config_file);
        deserialize(config);
    }

};

};
