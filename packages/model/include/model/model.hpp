#pragma once

#include "yaml-cpp/yaml.h"

#include <string>

namespace truck::model {

struct WheelBase {
    double width;
    double length;
};

struct Model {




    DECLARE_MODEL_FIELD(double, max_velocity);
    DECLARE_MODEL_FIELD(double, max_acceleration);
    DECLARE_MODEL_FIELD(double, max_decceleration);
    DECLARE_MODEL_FIELD(double, lookahead_distance);
    DECLARE_MODEL_FIELD(double, truck_length);
    DECLARE_MODEL_FIELD(double, truck_width);
    DECLARE_MODEL_FIELD(double, wheel_radius);
public:
    Model(const std::string &config_file) {
        YAML::Node config = YAML::LoadFile(config_file);
        SerializeHelper<sizeof(Model)>::deserialize(*this, config);
    }

#undef DECLARE_MODEL_FIELD
};

}  // namespace truck::model
