#include "svg_debug_drawer/sdd.h"

#include "common/exception.h"

#include <fstream>
#include <stdexcept>

#include <algorithm>

namespace truck::sdd {

SDD::SDD(std::string_view input_path, std::string_view output_path) : output_path_(output_path) {
    std::ifstream file(input_path.data(), std::ios::in);
    if (!file.is_open()) {
        throw std::runtime_error("Couldn't open file\n");
    }

    auto xml_parse_result = doc_.load(file);
    if (!xml_parse_result) {
        throw std::runtime_error(
            "Parse failed with error: " + std::string(xml_parse_result.description()));
    }

    file.close();

    root_ = doc_.child("svg").child("g");
    VERIFY(root_);
    VERIFY(root_.attribute("transform"));
}

SDD::SDD(std::string_view path) : SDD(path, path) {}

SDD::SDD(const Size& size, std::string_view output_path) : output_path_(output_path) {
    auto svg = doc_.append_child("svg");
    svg.append_attribute("version").set_value("1.1");
    svg.append_attribute("width").set_value(std::to_string(size.width).data());
    svg.append_attribute("height").set_value(std::to_string(size.height).data());
    svg.append_attribute("xmlns").set_value("http://www.w3.org/2000/svg");
    root_ = svg.append_child("g");
    root_.append_attribute("transform")
        .set_value(("translate(0," + std::to_string(size.height) + ") scale(1,-1)").data());
}

void SDD::DrawMarker(const Marker& marker) noexcept {
    auto size = GetSize();
    double scale = 0.01 * std::max(size.width, size.height);  // 1% of image longest side

    pugi::xml_node node;
    switch (marker.type) {
        case Marker::Type::Circle:
            node = root_.append_child("circle");
            node.append_attribute("cx").set_value(std::to_string(marker.point.x).data());
            node.append_attribute("cy").set_value(std::to_string(marker.point.y).data());
            node.append_attribute("r").set_value(std::to_string(scale).data());
            break;
        case Marker::Type::Square:
            node = root_.append_child("rect");
            node.append_attribute("x").set_value(std::to_string(marker.point.x - scale).data());
            node.append_attribute("y").set_value(std::to_string(marker.point.y - scale).data());
            node.append_attribute("width").set_value(std::to_string(2 * scale).data());
            node.append_attribute("height").set_value(std::to_string(2 * scale).data());
            break;
    }
    node.append_attribute("id").set_value(marker.id.data());
    node.append_attribute("fill").set_value(ToString(marker.color).data());
    DrawTitle(node, marker.id);
}

void SDD::DrawPolyline(const Polyline& polyline) noexcept {
    pugi::xml_node node = root_.append_child("polyline");
    std::stringstream sstream;
    for (auto it = polyline.points.begin(); it != polyline.points.end(); ++it) {
        sstream << it->x << ',' << it->y;
        if (it + 1 != polyline.points.end()) {
            sstream << ' ';
        }
    }
    node.append_attribute("points").set_value(sstream.str().data());
    node.append_attribute("fill").set_value("none");
    node.append_attribute("stroke").set_value(ToString(polyline.color).data());
    auto size = GetSize();
    double scale = 0.005 * std::max(size.width, size.height);  // 0.5% of image longest side
    node.append_attribute("stroke-width").set_value(std::to_string(scale).data());
}

Size SDD::GetSize() const noexcept {
    auto svg = root_.parent();
    return {
        .width = svg.attribute("width").as_ullong(), .height = svg.attribute("height").as_ullong()};
}

std::string SDD::ToString(Color color) noexcept {
    switch (color) {
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

SDD::~SDD() { doc_.save_file(output_path_.data()); }

void SDD::DrawTitle(pugi::xml_node& node, const std::string& id) noexcept {
    node.append_child("title").text().set(id.data());
}

}  // namespace truck::sdd