#pragma once

#include "util/yaml_serializer.hpp"

namespace pure_pursuit {

struct ControllerConfig {
    YAML_SERIALIZABLE_STRUCT(ControllerConfig)

    YAML_DECLARE_FIELD(bool, smooth_curvature);
    YAML_DECLARE_FIELD(size_t, integrator_steps);
    YAML_DECLARE_FIELD(double, lookahead_distance);
public:
    ControllerConfig() = default;
    ControllerConfig(const std::string &config_file) {
        YAML::Node config = YAML::LoadFile(config_file);
        deserialize(config);
    }
};

}
