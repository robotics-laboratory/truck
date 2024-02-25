#include "svg_debug_drawer/sdd.h"

#include "common/exception.h"

#include <fstream>

namespace truck::sdd {

SDD::SDD(std::string_view input_path, std::string_view output_path) : output_path_(output_path) {
    std::ifstream file(input_path.data(), std::ios::in);
    VERIFY(file.is_open());

    auto xml_parse_result = doc_.load(file);
    VERIFY(xml_parse_result);

    file.close();

    root_ = doc_.child("svg").child("g");
    VERIFY(root_);
    VERIFY(root_.attribute("transform"));
}

SDD::SDD(std::string_view path) : SDD(path, path) {}

SDD::SDD(const Size& size, std::string_view output_path) : output_path_(output_path) {
    auto svg = doc_.append_child("svg");
    svg.append_attribute("version").set_value("1.1");
    svg.append_attribute("width").set_value(ToString(size.width).data());
    svg.append_attribute("height").set_value(ToString(size.height).data());
    svg.append_attribute("xmlns").set_value("http://www.w3.org/2000/svg");
    root_ = svg.append_child("g");
    root_.append_attribute("transform")
        .set_value(("translate(0," + ToString(size.height) + ") scale(1,-1)").data());
}

void SDD::DrawMarker(const geom::Vec2& point, const MarkerDescription& description) noexcept {
    auto scale = ScaleUnit();

    pugi::xml_node node;
    switch (description.type) {
        case MarkerDescription::Type::Circle:
            node = root_.append_child("circle");
            node.append_attribute("cx").set_value(ToString(point.x).data());
            node.append_attribute("cy").set_value(ToString(point.y).data());
            node.append_attribute("r").set_value(ToString(scale).data());
            break;
        case MarkerDescription::Type::Square:
            node = root_.append_child("rect");
            node.append_attribute("x").set_value(ToString(point.x - scale).data());
            node.append_attribute("y").set_value(ToString(point.y - scale).data());
            node.append_attribute("width").set_value(ToString(scale * 2).data());
            node.append_attribute("height").set_value(ToString(scale * 2).data());
            break;
    }
    node.append_attribute("fill").set_value(ToString(description.color).data());
    node.append_attribute("id").set_value(description.id.data());
    node.append_child("title").text().set(description.id.data());
}

void SDD::DrawPose(const geom::Pose& pose, const Description& description) noexcept {
    auto scale = ScaleUnit();

    auto defs_node = root_.child("defs");
    if (!defs_node) {
        defs_node = root_.append_child("defs");
    }

    const auto pos_marker_id = "pos-marker-" + ToString(description.color);
    auto pos_marker_node = defs_node.find_child_by_attribute("id", pos_marker_id.data());
    if (!pos_marker_node) {
        pos_marker_node = defs_node.append_child("marker");
        pos_marker_node.append_attribute("id").set_value(pos_marker_id.data());
        pos_marker_node.append_attribute("orient").set_value("auto");
        pos_marker_node.append_attribute("markerWidth").set_value(ToString(scale * 2).data());
        pos_marker_node.append_attribute("markerHeight").set_value(ToString(scale * 2).data());
        pos_marker_node.append_attribute("markerUnits").set_value("userSpaceOnUse");
        pos_marker_node.append_attribute("refX").set_value(ToString(scale).data());
        pos_marker_node.append_attribute("refY").set_value(ToString(scale).data());
        auto circle_node = pos_marker_node.append_child("circle");
        circle_node.append_attribute("cx").set_value(ToString(scale).data());
        circle_node.append_attribute("cy").set_value(ToString(scale).data());
        circle_node.append_attribute("r").set_value(ToString(scale).data());
        circle_node.append_attribute("fill").set_value(ToString(description.color).data());
    }

    const auto dir_marker_id = "dir-marker-" + ToString(description.color);
    auto dir_marker_node = defs_node.find_child_by_attribute("id", dir_marker_id.data());
    if (!dir_marker_node) {
        dir_marker_node = defs_node.append_child("marker");
        dir_marker_node.append_attribute("id").set_value(dir_marker_id.data());
        dir_marker_node.append_attribute("orient").set_value("auto");
        dir_marker_node.append_attribute("markerWidth").set_value(ToString(scale * 2).data());
        dir_marker_node.append_attribute("markerHeight").set_value(ToString(scale * 2).data());
        dir_marker_node.append_attribute("markerUnits").set_value("userSpaceOnUse");
        dir_marker_node.append_attribute("refX").set_value("0");
        dir_marker_node.append_attribute("refY").set_value(ToString(scale).data());
        auto polygon_node = dir_marker_node.append_child("polygon");
        polygon_node.append_attribute("points").set_value(
            ToString(geom::Polygon{
                         geom::Vec2(0, 0), geom::Vec2(0, scale * 2), geom::Vec2(scale * 2, scale)})
                .data());
        polygon_node.append_attribute("fill").set_value(ToString(description.color).data());
    }

    auto node = root_.append_child("line");
    node.append_attribute("x1").set_value(ToString(pose.pos.x).data());
    node.append_attribute("y1").set_value(ToString(pose.pos.y).data());
    node.prepend_attribute("x2").set_value(
        ToString((pose.pos + pose.dir.vec().unit() * scale * 1.5).x).data());
    node.prepend_attribute("y2").set_value(
        ToString((pose.pos + pose.dir.vec().unit() * scale * 1.5).y).data());
    node.append_attribute("marker-start").set_value(("url(#" + pos_marker_id + ")").data());
    node.append_attribute("marker-end").set_value(("url(#" + dir_marker_id + ")").data());
    node.append_attribute("stroke").set_value(ToString(description.color).data());
    node.append_attribute("stroke-width").set_value(ToString(scale).data());
    node.append_attribute("id").set_value(description.id.data());
    node.append_child("title").text().set(description.id.data());
}

void SDD::DrawPolyline(const geom::Polyline& polyline, const Description& description) noexcept {
    auto scale = ScaleUnit();

    auto node = root_.append_child("polyline");
    node.append_attribute("points").set_value(ToString(polyline).data());
    node.append_attribute("fill").set_value("none");
    node.append_attribute("stroke").set_value(ToString(description.color).data());
    node.append_attribute("stroke-width").set_value(ToString(scale).data());
    node.append_attribute("id").set_value(description.id.data());
    node.append_child("title").text().set(description.id.data());
}

void SDD::DrawPolygon(const geom::Polygon& polygon, const Description& description) noexcept {
    auto node = root_.append_child("polygon");
    node.append_attribute("points").set_value(ToString(polygon).data());
    node.append_attribute("fill").set_value(ToString(description.color).data());
    node.append_attribute("id").set_value(description.id.data());
    node.append_child("title").text().set(description.id.data());
}

void SDD::DrawComplexPolygon(
    const geom::ComplexPolygon& polygon, const Description& description) noexcept {
    auto node = root_.append_child("path");
    node.append_attribute("d").set_value(ToString(polygon).data());
    node.append_attribute("fill").set_value(ToString(description.color).data());
    node.append_attribute("id").set_value(description.id.data());
    node.append_child("title").text().set(description.id.data());
}

geom::Vec2 SDD::FindMarker(std::string_view id) const {
    auto node = root_.find_child_by_attribute("id", id.data());
    if (std::string(node.name()) == "circle") {
        return geom::Vec2(node.attribute("cx").as_double(), node.attribute("cy").as_double());
    } else if (std::string(node.name()) == "rect") {
        return geom::Vec2(
            node.attribute("x").as_double() + node.attribute("width").as_double() * 0.5,
            node.attribute("y").as_double() + node.attribute("height").as_double() * 0.5);
    }
    THROW_EXCEPTION() << "Wrong attribute\n";
}

geom::Pose SDD::FindPose(std::string_view id) const {
    auto node = root_.find_child_by_attribute("id", id.data());
    VERIFY(std::string(node.name()) == "line");
    auto pos = geom::Vec2(node.attribute("x1").as_double(), node.attribute("y1").as_double());
    auto dir = geom::Vec2(node.attribute("x2").as_double(), node.attribute("y2").as_double()) - pos;
    return geom::Pose(pos, geom::AngleVec2::fromVector(dir));
}

geom::Polyline SDD::FindPolyline(std::string_view id) const {
    auto node = root_.find_child_by_attribute("id", id.data());
    VERIFY(std::string(node.name()) == "polyline");
    return FromString<geom::Polyline>(node.attribute("points").value());
}

geom::Polygon SDD::FindPolygon(std::string_view id) const {
    auto node = root_.find_child_by_attribute("id", id.data());
    VERIFY(std::string(node.name()) == "polygon");
    return FromString<geom::Polygon>(node.attribute("points").value());
}

geom::ComplexPolygon SDD::FindComplexPolygon(std::string_view id) const {
    auto node = root_.find_child_by_attribute("id", id.data());
    VERIFY(std::string(node.name()) == "path");
    return FromString(node.attribute("d").value());
}

Size SDD::GetSize() const noexcept {
    auto svg = root_.parent();
    return {
        .width = svg.attribute("width").as_ullong(), .height = svg.attribute("height").as_ullong()};
}

std::string SDD::ToString(const Color& value) {
    switch (value) {
        case Color::Black:
            return "black";
        case Color::Silver:
            return "silver";
        case Color::Gray:
            return "gray";
        case Color::White:
            return "white";
        case Color::Marron:
            return "marron";
        case Color::Red:
            return "red";
        case Color::Purple:
            return "purple";
        case Color::Fuchsia:
            return "fuchsia";
        case Color::Green:
            return "green";
        case Color::Lime:
            return "lime";
        case Color::Olive:
            return "olive";
        case Color::Yellow:
            return "yellow";
        case Color::Navy:
            return "navy";
        case Color::Blue:
            return "blue";
        case Color::Teal:
            return "teal";
        case Color::Aqua:
            return "aqua";
        default:
            return "black";
    }
}

std::string SDD::ToString(geom::ComplexPolygon value) {
    std::stringstream sstream;
    if (value.outer.orientation() == geom::Orientation::COUNTERCLOCKWISE) {
        std::reverse(value.outer.begin(), value.outer.end());
    }
    sstream << "M ";
    for (auto it = value.outer.begin(); it != value.outer.end(); ++it) {
        sstream << it->x << ',' << it->y;
        sstream << ' ' << (it + 1 == value.outer.end() ? 'Z' : 'L') << ' ';
    }
    for (auto& inner : value.inners) {
        if (inner.orientation() == geom::Orientation::CLOCKWISE) {
            std::reverse(inner.begin(), inner.end());
        }
        sstream << "M ";
        for (auto it = inner.begin(); it != inner.end(); ++it) {
            sstream << it->x << ',' << it->y;
            sstream << ' ' << (it + 1 == inner.end() ? 'Z' : 'L') << ' ';
        }
    }
    return sstream.str();
}

geom::ComplexPolygon SDD::FromString(std::string_view data) {
    geom::ComplexPolygon object;
    auto subpath_begin = data.begin();
    auto subpath_end = std::find_if(
        subpath_begin, data.end(), [](const char ch) -> bool { return ch == 'z' || ch == 'Z'; });
    while (subpath_end != data.end()) {
        geom::Polygon polygon = FromString<geom::Polygon>(
            data.substr(subpath_begin - data.begin(), subpath_end - subpath_begin));
        if (polygon.orientation() == geom::Orientation::CLOCKWISE) {
            object.outer = std::move(polygon);
        } else {
            object.inners.push_back(std::move(polygon));
        }
        subpath_begin = subpath_end + 1;
        subpath_end = std::find_if(subpath_begin, data.end(), [](const char ch) -> bool {
            return ch == 'z' || ch == 'Z';
        });
    }
    return object;
}

SDD::~SDD() { doc_.save_file(output_path_.data()); }

double SDD::ScaleUnit() const noexcept {
    auto size = GetSize();
    return std::max(size.width, size.height) * 0.01;  // 1% of image longest side
}

}  // namespace truck::sdd