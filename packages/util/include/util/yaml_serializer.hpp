#pragma once

#include "yaml-cpp/yaml.h"

namespace util {

#define YAML_SERIALIZABLE_STRUCT(name) \
    private: \
        using _SelfType = name; \
        template<size_t offset, class = void> \
        struct SerializeHelper { \
            static void deserialize(_SelfType& object, const YAML::Node& config) { \
                if constexpr (offset > 0) \
                    SerializeHelper<offset - 1>::deserialize(object, config); \
            } \
        }; \
    public: \
        void deserialize(const YAML::Node &config) { \
            SerializeHelper<sizeof(_SelfType)>::deserialize(*this, config); \
        }
#define YAML_DECLARE_FIELD(type, name) public: \
    type name; \
    private: \
    template<class Dummy> \
    struct SerializeHelper<offsetof(_SelfType, name) + sizeof(type), Dummy> { \
        static void deserialize(_SelfType& object, const YAML::Node& config) { \
            object.name = config[#name].as<type>(); \
            SerializeHelper<offsetof(_SelfType, name)>::deserialize(object, config); \
        } \
    };
}
