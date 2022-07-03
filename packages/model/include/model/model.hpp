#pragma once

#include "yaml-cpp/yaml.h"

#include <string>

namespace model {

struct Model {
  private:
    template <size_t offset, class = void>
    struct SerializeHelper {
        static void deserialize(Model &model, YAML::Node &config) {
            if constexpr (offset > 0) SerializeHelper<offset - 1>::deserialize(model, config);
        }
    };
#define DECLARE_MODEL_FIELD(type, name)                                         \
  public:                                                                       \
    type name;                                                                  \
                                                                                \
  private:                                                                      \
    template <class Dummy>                                                      \
    struct SerializeHelper<offsetof(Model, name) + sizeof(type), Dummy> {       \
        static void deserialize(Model &model, YAML::Node &config) {             \
            model.name = config[#name].as<type>();                              \
            SerializeHelper<offsetof(Model, name)>::deserialize(model, config); \
        }                                                                       \
    };

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

};  // namespace model