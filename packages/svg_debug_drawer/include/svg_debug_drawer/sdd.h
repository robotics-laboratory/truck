#pragma once

#include <pugixml.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "geom/vector.h"
#include "geom/polyline.h"
#include "geom/polygon.h"
#include "geom/complex_polygon.h"

namespace truck::sdd {

enum class Color {
    Black,
    Silver,
    Gray,
    White,
    Marron,
    Red,
    Purple,
    Fuchsia,
    Green,
    Lime,
    Olive,
    Yellow,
    Navy,
    Blue,
    Teal,
    Aqua
};

struct Size {
    size_t width = 512;
    size_t height = 512;
};

struct Description {
    std::string id = "";
    Color color = Color::Black;
};

struct MarkerDescription final : public Description {
    enum class Type { Circle, Square };
    Type type = Type::Circle;
};

class SDD {
  public:
    SDD() = delete;

    SDD(std::string_view input_path, std::string_view output_path);

    SDD(std::string_view path);

    SDD(const Size& size, std::string_view output_path);

    void DrawMarker(
        const geom::Vec2& point,
        const MarkerDescription& description = MarkerDescription()) noexcept;

    void DrawPose(const geom::Pose& pose, const Description& description = Description()) noexcept;

    void DrawPolyline(
        const geom::Polyline& polyline, const Description& description = Description()) noexcept;

    void DrawPolygon(
        const geom::Polygon& polygon, const Description& description = Description()) noexcept;

    void DrawComplexPolygon(
        const geom::ComplexPolygon& polygon,
        const Description& description = Description()) noexcept;

    geom::Vec2 FindMarker(std::string_view id) const;

    geom::Pose FindPose(std::string_view id) const;

    geom::Polyline FindPolyline(std::string_view id) const;

    geom::Polygon FindPolygon(std::string_view id) const;

    geom::ComplexPolygon FindComplexPolygon(std::string_view id) const;

    Size GetSize() const noexcept;

    template<typename T>
    static std::enable_if_t<std::is_arithmetic_v<T>, std::string> ToString(const T& value) {
        std::stringstream sstream;
        sstream << value;
        return sstream.str();
    }

    static std::string ToString(const Color& value);

    template<typename T>
    static std::enable_if_t<std::is_base_of_v<std::vector<geom::Vec2>, T>, std::string> ToString(
        const T& value) {
        std::stringstream sstream;
        for (auto it = value.begin(); it != value.end(); ++it) {
            sstream << it->x << ',' << it->y;
            if (it + 1 != value.end()) {
                sstream << ' ';
            }
        }
        return sstream.str();
    }

    static std::string ToString(geom::ComplexPolygon value);

    template<typename T>
    static std::enable_if_t<std::is_base_of_v<std::vector<geom::Vec2>, T>, T> FromString(
        std::string_view data) {
        T object;
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

    static geom::ComplexPolygon FromString(std::string_view data);

    ~SDD();

  private:
    double ScaleUnit() const noexcept;

    std::string output_path_;

    pugi::xml_document doc_;
    pugi::xml_node root_;
};

}  // namespace truck::sdd