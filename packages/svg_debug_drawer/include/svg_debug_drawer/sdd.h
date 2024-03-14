#pragma once

#include <pugixml.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "geom/vector.h"
#include "geom/polyline.h"
#include "geom/polygon.h"
#include "geom/complex_polygon.h"

namespace truck::sdd {

namespace color {

struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

constexpr auto black = RGB(0, 0, 0);
constexpr auto silver = RGB(192, 192, 192);
constexpr auto gray = RGB(128, 128, 128);
constexpr auto white = RGB(255, 255, 255);
constexpr auto maroon = RGB(128, 0, 0);
constexpr auto red = RGB(255, 0, 0);
constexpr auto purple = RGB(128, 0, 128);
constexpr auto fuchsia = RGB(255, 0, 255);
constexpr auto green = RGB(0, 128, 0);
constexpr auto lime = RGB(0, 255, 0);
constexpr auto olive = RGB(128, 128, 0);
constexpr auto yellow = RGB(255, 255, 0);
constexpr auto navy = RGB(0, 0, 128);
constexpr auto blue = RGB(0, 0, 255);
constexpr auto teal = RGB(0, 128, 128);
constexpr auto aqua = RGB(0, 255, 255);

}  // namespace color

struct Size {
    size_t width = 512;
    size_t height = 512;
};

struct Marker {
    enum class Shape : uint8_t { Circle, Square };

    geom::Vec2 point;
    Shape shape = Shape::Circle;
    double scale = 1.0;
    color::RGB color = color::black;
    std::string label = "";
};

struct Pose {
    geom::Pose pose;
    double scale = 1.0;
    double length = 1.0;
    color::RGB color = color::black;
    std::string label = "";
};

struct Polyline {
    geom::Polyline polyline;
    double thickness = 1.0;
    color::RGB color = color::black;
    std::string label = "";
};

struct Polygon {
    geom::Polygon polygon;
    double border_thickness = 0.0;
    bool fill = true;
    color::RGB color = color::black;
    std::string label = "";
};

struct ComplexPolygon {
    geom::ComplexPolygon complex_polygon;
    double border_thickness = 0.0;
    bool fill = true;
    color::RGB color = color::black;
    std::string label = "";
};

class SDD {
  public:
    SDD() = delete;

    SDD(const std::string& input_path, const std::string& output_path);

    SDD(const std::string& path);

    SDD(const Size& size, const std::string& output_path);

    SDD& Add(const Marker& marker) noexcept;

    SDD& Add(const geom::Vec2& point) noexcept;

    SDD& Add(const Pose& pose) noexcept;

    SDD& Add(const geom::Pose& pose) noexcept;

    SDD& Add(const Polyline& polyline) noexcept;

    SDD& Add(const geom::Polyline& polyline) noexcept;

    SDD& Add(const Polygon& polygon) noexcept;

    SDD& Add(const geom::Polygon& polygon) noexcept;

    SDD& Add(const ComplexPolygon& polygon) noexcept;

    SDD& Add(const geom::ComplexPolygon& polygon) noexcept;

    Size GetSize() const noexcept;

    ~SDD();

  private:
    double ScaleUnit() const noexcept;

    std::string output_path_;

    pugi::xml_document doc_;
    pugi::xml_node root_;
};

template<typename T>
SDD& operator<<(SDD& img, const T& obj) {
    return img.Add(obj);
}

}  // namespace truck::sdd