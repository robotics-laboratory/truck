#pragma once

#include "geom/vector.h"

#include <algorithm>
#include <numeric>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace truck::svg_processor::object::serializers {

template<typename T>
struct ObjectSerializer {
    virtual std::string operator()(const T& object) const = 0;
};

template<typename T, typename Enable = void>
struct Serializer final : public ObjectSerializer<T> {
    std::string operator()(const T& object) const final override = delete;
};

template<>
struct Serializer<std::string> final : public ObjectSerializer<std::string> {
    std::string operator()(const std::string& object) const final override {
        return std::string(object);
    }
};

template<typename T>
struct Serializer<T, std::enable_if_t<std::is_arithmetic_v<T>>> final : public ObjectSerializer<T> {
    std::string operator()(const T& object) const final override {
        std::stringstream sstream;
        sstream << object;
        return sstream.str();
    }
};

template<>
struct Serializer<std::vector<geom::Vec2>> final
    : public ObjectSerializer<std::vector<geom::Vec2>> {
    std::string operator()(const std::vector<geom::Vec2>& object) const final override {
        std::stringstream sstream;
        for (auto it = object.begin(); it != object.end(); ++it) {
            sstream << it->x << ',' << it->y;
            if (it + 1 != object.end()) {
                sstream << ' ';
            }
        }
        return sstream.str();
    }
};

template<>
struct Serializer<std::vector<std::string>> final
    : public ObjectSerializer<std::vector<std::string>> {
    std::string operator()(const std::vector<std::string>& object) const final override {
        std::string data = std::accumulate(
            object.begin(),
            object.end(),
            std::string(),
            [](std::string in, std::string str) -> std::string {
                return std::move(in) + std::move(str) + ' ';
            });
        return data.substr(0, (data.empty() ? 0 : data.size() - 1));
    }
};

}  // namespace truck::svg_processor::object::serializers

namespace truck::svg_processor::object::deserializers {

template<typename T>
struct ObjectDeserializer {
    virtual T operator()(std::string_view data) const = 0;
};

template<typename T, typename Enable = void>
struct Deserializer final : public ObjectDeserializer<T> {
    T operator()(std::string_view data) const final override = delete;
};

template<>
struct Deserializer<std::string> final : public ObjectDeserializer<std::string> {
    std::string operator()(std::string_view data) const final override {
        auto object_begin = std::find_if(
            data.begin(), data.end(), [](const char ch) -> bool { return !std::isspace(ch); });
        auto object_end = std::find_if(data.rbegin(), data.rend(), [](const char ch) -> bool {
                              return !std::isspace(ch);
                          }).base();
        data = data.substr(object_begin - data.begin(), object_end - object_begin);
        return std::string(data);
    }
};

template<typename T>
struct Deserializer<T, std::enable_if_t<std::is_arithmetic_v<T>>> final
    : public ObjectDeserializer<T> {
    T operator()(std::string_view data) const final override {
        T object;
        std::stringstream(data.data()) >> object;
        return object;
    }
};

template<>
struct Deserializer<std::vector<geom::Vec2>> final
    : public ObjectDeserializer<std::vector<geom::Vec2>> {
    std::vector<geom::Vec2> operator()(std::string_view data) const final override {
        std::vector<geom::Vec2> object;
        auto comma_it = std::find(data.begin(), data.end(), ',');
        while (comma_it != data.end()) {
            auto coord_x_rbegin = std::find_if(
                std::make_reverse_iterator(comma_it + 1), data.rend(), [](const char ch) -> bool {
                    return std::isdigit(ch);
                });
            auto coord_x_rend =
                std::find_if(coord_x_rbegin, data.rend(), [](const char ch) -> bool {
                    return std::isspace(ch);
                });
            auto coord_y_begin = std::find_if(comma_it, data.end(), [](const char ch) -> bool {
                return std::isdigit(ch) || (ch == '-');
            });
            auto coord_y_end = std::find_if(
                coord_y_begin, data.end(), [](const char ch) -> bool { return std::isspace(ch); });
            geom::Vec2 point;
            std::stringstream(
                data.substr(coord_x_rend.base() - data.begin(), coord_x_rend - coord_x_rbegin)
                    .data()) >>
                point.x;
            std::stringstream(
                data.substr(coord_y_begin - data.begin(), coord_y_end - coord_y_begin).data()) >>
                point.y;
            object.push_back(point);
            comma_it = std::find(comma_it + 1, data.end(), ',');
        }
        return object;
    }
};

template<>
struct Deserializer<std::vector<std::string>> final
    : public ObjectDeserializer<std::vector<std::string>> {
    std::vector<std::string> operator()(std::string_view data) const final override {
        std::vector<std::string> object;
        auto object_begin = std::find_if(
            data.begin(), data.end(), [](const char ch) -> bool { return !std::isspace(ch); });
        while (object_begin != data.end()) {
            auto object_end = std::find_if(
                object_begin, data.end(), [](const char ch) -> bool { return std::isspace(ch); });
            object.push_back(std::string(object_begin, object_end));
            object_begin = std::find_if(
                object_end, data.end(), [](const char ch) -> bool { return !std::isspace(ch); });
        }
        return object;
    }
};

}  // namespace truck::svg_processor::object::deserializers

namespace truck::svg_processor::attributes {

using SerializedAttribute = std::string;

struct BaseAttribute {};

template<typename T>
inline constexpr bool IsAttributeType = std::is_base_of_v<BaseAttribute, T>&&
    std::is_convertible_v<const volatile T*, const volatile BaseAttribute*>;

namespace serializers {

template<typename T, typename Enable>
struct Serializer;

}  // namespace serializers

namespace deserializers {

template<typename T, typename Enable>
struct Deserializer;

}  // namespace deserializers

template<typename T>
class Attribute final : public BaseAttribute {
  public:
    friend struct serializers::Serializer<Attribute<T>, void>;
    friend struct deserializers::Deserializer<Attribute<T>, void>;

    using TagType = std::string;
    using ObjectType = T;

    Attribute() = default;

    Attribute(const TagType& tag, const ObjectType& object) : tag_(tag), object_(object) {}

    Attribute(const Attribute& other) : tag_(other.tag_), object_(other.object_) {}

    Attribute(Attribute&& other) : Attribute() {
        std::swap(tag_, other.tag_);
        std::swap(object_, other.object_);
    }

    Attribute& operator=(Attribute other) & {
        std::swap(tag_, other.tag_);
        std::swap(object_, other.object_);
    }

    TagType& GetTag() { return tag_; }

    const TagType& GetTag() const { return tag_; }

    ObjectType& GetObject() { return object_; }

    const ObjectType& GetObject() const { return object_; }

    ObjectType& operator*() { return object_; }

    const ObjectType& operator*() const { return object_; }

    Attribute& SetTag(const TagType& tag) {
        tag_ = tag;
        return *this;
    }

    Attribute& SetObject(const ObjectType& object) {
        object_ = object;
        return *this;
    }

    ~Attribute() = default;

  private:
    TagType tag_;
    ObjectType object_;
};

using StringAttribute = Attribute<std::string>;
using CharAttribute = Attribute<char>;
using IntAttribute = Attribute<int>;
using FloatAttribute = Attribute<float>;
using DoubleAttribute = Attribute<double>;
using PointsAttriubte = Attribute<std::vector<geom::Vec2>>;
using MultistringAttribute = Attribute<std::vector<std::string>>;

namespace serializers {

template<typename T, typename Enable = std::enable_if_t<attributes::IsAttributeType<T>>>
struct Serializer {
    attributes::SerializedAttribute operator()(const T& attribute) const {
        return std::string(attribute.tag_) + "=\"" +
               object::serializers::Serializer<typename T::ObjectType>()(attribute.object_) + "\"";
    }
};

}  // namespace serializers

namespace deserializers {

template<typename T, typename Enable = std::enable_if_t<attributes::IsAttributeType<T>>>
struct Deserializer {
    T operator()(const attributes::SerializedAttribute& data) const {
        auto equals_it = std::find(data.begin(), data.end(), '=');
        auto object_begin = std::find(equals_it + 1, data.end(), '\"') + 1;
        auto object_end = std::find(object_begin, data.end(), '\"');
        return T(
            object::deserializers::Deserializer<typename T::TagType>()(
                data.substr(0, equals_it - data.begin())),
            object::deserializers::Deserializer<typename T::ObjectType>()(
                data.substr(object_begin - data.begin(), object_end - object_begin)));
    }
};

}  // namespace deserializers

}  // namespace truck::svg_processor::attributes