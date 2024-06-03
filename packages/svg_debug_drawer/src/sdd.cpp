#include "svg_debug_drawer/sdd.h"

#include <fmt/format.h>

#include <fstream>
#include <utility>

#include "common/exception.h"

namespace {

std::string toString(const truck::sdd::color::RGB& value) {
    return fmt::format("rgb({}, {}, {})", value.r, value.g, value.b);
}

template<typename T>
std::enable_if_t<std::is_arithmetic_v<T>, std::string> toString(const T& value) {
    return fmt::to_string(value);
}

std::string toString(const truck::geom::Vec2& value) {
    return fmt::format("{},{}", value.x, value.y);
}

template<typename T>
std::enable_if_t<std::is_base_of_v<std::vector<truck::geom::Vec2>, T>, std::string> toString(
    const T& value) {
    std::vector<std::string> points;
    points.reserve(value.size());
    std::transform(value.begin(), value.end(), std::back_inserter(points), [](const auto& point) {
        return toString(point);
    });
    return fmt::format("{}", fmt::join(points, " "));
}

std::string toString(const truck::geom::ComplexPolygon& value) {
    std::vector<std::string> store;
    store.reserve(value.inners.size() + 1);
    {
        std::vector<std::string> points;
        points.reserve(value.outer.size());
        switch (value.outer.orientation()) {
            case truck::geom::Orientation::CLOCKWISE:
                std::transform(
                    value.outer.begin(),
                    value.outer.end(),
                    std::back_inserter(points),
                    [](const auto& point) { return toString(point); });
                break;
            case truck::geom::Orientation::COUNTERCLOCKWISE:
                std::transform(
                    value.outer.rbegin(),
                    value.outer.rend(),
                    std::back_inserter(points),
                    [](const auto& point) { return toString(point); });
                break;
        }
        store.push_back(fmt::format("M {} Z", fmt::join(points, " L ")));
    }
    for (const auto& inner : value.inners) {
        std::vector<std::string> points;
        points.reserve(inner.size());
        switch (inner.orientation()) {
            case truck::geom::Orientation::CLOCKWISE:
                std::transform(
                    inner.rbegin(),
                    inner.rend(),
                    std::back_inserter(points),
                    [](const auto& point) { return toString(point); });
                break;
            case truck::geom::Orientation::COUNTERCLOCKWISE:
                std::transform(
                    inner.begin(), inner.end(), std::back_inserter(points), [](const auto& point) {
                        return toString(point);
                    });
                break;
        }
        store.push_back(fmt::format("M {} Z", fmt::join(points, " L ")));
    }
    return fmt::format("{}", fmt::join(store, " "));
}

}  // namespace

namespace truck::sdd {

SDD::SDD(const std::string& input_path, std::string output_path) :
    output_path_(std::move(output_path)) {
    std::ifstream file(input_path, std::ios::in);
    VERIFY(file.is_open());

    auto xml_parse_result = doc_.load(file);
    VERIFY(xml_parse_result);

    file.close();

    root_ = doc_.child("svg").child("g");
    VERIFY(root_);
}

SDD::SDD(const std::string& path) : SDD(path, path) {}

SDD::SDD(const Size& size, std::string output_path) : output_path_(std::move(output_path)) {
    auto svg = doc_.append_child("svg");
    svg.append_attribute("version").set_value("1.1");
    svg.append_attribute("width").set_value(toString(size.width).data());
    svg.append_attribute("height").set_value(toString(size.height).data());
    svg.append_attribute("xmlns").set_value("http://www.w3.org/2000/svg");
    root_ = svg.append_child("g");
    root_.append_attribute("transform")
        .set_value(("translate(0," + toString(size.height) + ") scale(1,-1)").data());
}

SDD& SDD::Add(const Marker& marker) noexcept {
    pugi::xml_node node;
    switch (marker.shape) {
        case Marker::Shape::Circle:
            node = root_.append_child("circle");
            node.append_attribute("cx").set_value(toString(marker.point.x).data());
            node.append_attribute("cy").set_value(toString(marker.point.y).data());
            node.append_attribute("r").set_value(toString(marker.scale / 2).data());
            break;
        case Marker::Shape::Square:
            node = root_.append_child("rect");
            node.append_attribute("x").set_value(
                toString(marker.point.x - marker.scale / 2).data());
            node.append_attribute("y").set_value(
                toString(marker.point.y - marker.scale / 2).data());
            node.append_attribute("width").set_value(toString(marker.scale).data());
            node.append_attribute("height").set_value(toString(marker.scale).data());
            break;
    }
    node.append_attribute("fill").set_value(toString(marker.color).data());
    node.append_attribute("id").set_value(marker.label.data());
    node.append_child("title").text().set(marker.label.data());
    return *this;
}

SDD& SDD::Add(const geom::Vec2& point) noexcept {
    return Add(Marker{.point = point, .scale = ScaleUnit() * 2});
}

SDD& SDD::Add(const Pose& pose) noexcept {
    auto defs_node = root_.child("defs");
    if (!defs_node) {
        defs_node = root_.append_child("defs");
    }

    const auto marker_id = fmt::format(
        "marker-{}-{:03}-{:03}-{:03}", pose.scale, pose.color.r, pose.color.g, pose.color.b);

    const auto pos_marker_id = fmt::format("pos-{}", marker_id);
    auto pos_marker_node = defs_node.find_child_by_attribute("id", pos_marker_id.data());
    if (!pos_marker_node) {
        pos_marker_node = defs_node.append_child("marker");
        pos_marker_node.append_attribute("id").set_value(pos_marker_id.data());
        pos_marker_node.append_attribute("orient").set_value("auto");
        pos_marker_node.append_attribute("markerWidth").set_value(toString(pose.scale).data());
        pos_marker_node.append_attribute("markerHeight").set_value(toString(pose.scale).data());
        pos_marker_node.append_attribute("markerUnits").set_value("userSpaceOnUse");
        pos_marker_node.append_attribute("refX").set_value(toString(pose.scale * 0.5).data());
        pos_marker_node.append_attribute("refY").set_value(toString(pose.scale * 0.5).data());
        auto circle_node = pos_marker_node.append_child("circle");
        circle_node.append_attribute("cx").set_value(toString(pose.scale * 0.5).data());
        circle_node.append_attribute("cy").set_value(toString(pose.scale * 0.5).data());
        circle_node.append_attribute("r").set_value(toString(pose.scale * 0.5).data());
        circle_node.append_attribute("fill").set_value(toString(pose.color).data());
    }

    const auto dir_marker_id = fmt::format("dir-{}", marker_id);
    auto dir_marker_node = defs_node.find_child_by_attribute("id", dir_marker_id.data());
    if (!dir_marker_node) {
        dir_marker_node = defs_node.append_child("marker");
        dir_marker_node.append_attribute("id").set_value(dir_marker_id.data());
        dir_marker_node.append_attribute("orient").set_value("auto");
        dir_marker_node.append_attribute("markerWidth").set_value(toString(pose.scale).data());
        dir_marker_node.append_attribute("markerHeight").set_value(toString(pose.scale).data());
        dir_marker_node.append_attribute("markerUnits").set_value("userSpaceOnUse");
        dir_marker_node.append_attribute("refX").set_value("0");
        dir_marker_node.append_attribute("refY").set_value(toString(pose.scale * 0.5).data());
        auto polygon_node = dir_marker_node.append_child("polygon");
        polygon_node.append_attribute("points").set_value(
            toString(geom::Polygon{
                         geom::Vec2(0, 0),
                         geom::Vec2(0, pose.scale),
                         geom::Vec2(pose.scale, pose.scale * 0.5)})
                .data());
        polygon_node.append_attribute("fill").set_value(toString(pose.color).data());
    }

    auto node = root_.append_child("line");
    node.append_attribute("x1").set_value(toString(pose.pose.pos.x).data());
    node.append_attribute("y1").set_value(toString(pose.pose.pos.y).data());
    node.prepend_attribute("x2").set_value(
        toString((pose.pose.pos + pose.pose.dir.vec().unit() * pose.length).x).data());
    node.prepend_attribute("y2").set_value(
        toString((pose.pose.pos + pose.pose.dir.vec().unit() * pose.length).y).data());
    node.append_attribute("marker-start").set_value(("url(#" + pos_marker_id + ")").data());
    node.append_attribute("marker-end").set_value(("url(#" + dir_marker_id + ")").data());
    node.append_attribute("stroke").set_value(toString(pose.color).data());
    node.append_attribute("stroke-width").set_value(toString(pose.scale * 0.5).data());
    node.append_attribute("id").set_value(pose.label.data());
    node.append_child("title").text().set(pose.label.data());

    return *this;
}

SDD& SDD::Add(const geom::Pose& pose) noexcept {
    return Add(Pose{.pose = pose, .scale = ScaleUnit() * 2, .length = ScaleUnit() * 2});
}

SDD& SDD::Add(const Polyline& polyline) noexcept {
    auto node = root_.append_child("polyline");
    node.append_attribute("points").set_value(toString(polyline.polyline).data());
    node.append_attribute("fill").set_value("none");
    node.append_attribute("stroke").set_value(toString(polyline.color).data());
    node.append_attribute("stroke-width").set_value(toString(polyline.thickness).data());
    node.append_attribute("id").set_value(polyline.label.data());
    node.append_child("title").text().set(polyline.label.data());

    return *this;
}

SDD& SDD::Add(const geom::Polyline& polyline) noexcept {
    return Add(Polyline{.polyline = polyline, .thickness = ScaleUnit()});
}

SDD& SDD::Add(const Polygon& polygon) noexcept {
    auto node = root_.append_child("polygon");
    node.append_attribute("points").set_value(toString(polygon.polygon).data());
    node.append_attribute("stroke").set_value(toString(polygon.color).data());
    node.append_attribute("stroke-width").set_value(toString(polygon.border_thickness).data());
    node.append_attribute("fill").set_value(
        (polygon.fill ? toString(polygon.color) : "none").data());
    node.append_attribute("id").set_value(polygon.label.data());
    node.append_child("title").text().set(polygon.label.data());

    return *this;
}

SDD& SDD::Add(const geom::Polygon& polygon) noexcept { return Add(Polygon{.polygon = polygon}); }

SDD& SDD::Add(const ComplexPolygon& polygon) noexcept {
    auto node = root_.append_child("path");
    node.append_attribute("d").set_value(toString(polygon.complex_polygon).data());
    node.append_attribute("stroke").set_value(toString(polygon.color).data());
    node.append_attribute("stroke-width").set_value(toString(polygon.border_thickness).data());
    node.append_attribute("fill").set_value(
        (polygon.fill ? toString(polygon.color) : "none").data());
    node.append_attribute("id").set_value(polygon.label.data());
    node.append_child("title").text().set(polygon.label.data());

    return *this;
}

SDD& SDD::Add(const geom::ComplexPolygon& polygon) noexcept {
    return Add(ComplexPolygon{.complex_polygon = polygon});
}

Size SDD::GetSize() const noexcept {
    auto svg = root_.parent();
    return {
        .width = svg.attribute("width").as_ullong(), .height = svg.attribute("height").as_ullong()};
}

SDD::~SDD() { doc_.save_file(output_path_.data()); }

double SDD::ScaleUnit() const noexcept {
    auto size = GetSize();
    return std::max(size.width, size.height) * 0.01;  // 1% of image longest side
}

}  // namespace truck::sdd
