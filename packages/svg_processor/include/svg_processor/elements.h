#pragma once

#include "svg_processor/attributes.h"

#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include <memory>

namespace truck::svg_processor::elements {

using SerializedElement = std::string;

namespace serializers {

struct Serializer;

}  // namespace serializers

namespace deserializers {

struct Deserializer;

}  // namespace deserializers

class Element {
  public:
    using NameType = std::string;
    using AttributesStorageType = std::unordered_map<std::string, attributes::SerializedAttribute>;

    friend struct serializers::Serializer;
    friend struct deserializers::Deserializer;

    Element() = delete;

    explicit Element(std::string_view name);

    Element(std::string_view name, AttributesStorageType attributes);

    Element(const Element& other);

    Element(Element&& other);

    Element& operator=(Element other) &;

    NameType& GetName();

    const NameType& GetName() const;

    template<typename T, typename = std::enable_if_t<attributes::IsAttributeType<T>>>
    std::optional<T> GetAttribute(const std::string& key) const {
        auto it = attributes_.find(key);
        if (it == attributes_.end()) {
            return std::nullopt;
        }
        return attributes::deserializers::Deserializer<T>()(it->second);
    }

    const AttributesStorageType& GetAttributesStorage() const;

    Element& SetName(std::string_view name);

    template<typename T, typename = std::enable_if_t<attributes::IsAttributeType<T>>>
    Element& SetAttribute(const T& attribute) {
        attributes_[attribute.GetTag()] = attributes::serializers::Serializer<T>()(attribute);
        return *this;
    }

    template<typename T>
    Element& SetAttribute(const std::string& tag, const T& object) {
        return SetAttribute<attributes::Attribute<T>>(attributes::Attribute<T>(tag, object));
    }

    ~Element() = default;

  private:
    NameType name_;
    AttributesStorageType attributes_;
};

namespace serializers {

struct Serializer {
    elements::SerializedElement operator()(const Element& element) const;
};

}  // namespace serializers

namespace deserializers {

struct Deserializer {
    Element operator()(const elements::SerializedElement& data) const;
};

}  // namespace deserializers

}  // namespace truck::svg_processor::elements