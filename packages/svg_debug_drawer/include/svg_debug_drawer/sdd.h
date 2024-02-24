#include <pugixml.hpp>

#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>

#include "geom/vector.h"
#include "geom/polyline.h"

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

struct Element {
    std::string id = "";
    Color color = Color::Black;
};

struct Marker final : public Element {
    enum class Type { Circle, Square };

    geom::Vec2 point;
    Type type = Type::Circle;
};

struct Polyline final : public Element {
    geom::Polyline points;
};

class SDD {
  public:
    SDD() = delete;

    SDD(std::string_view input_path, std::string_view output_path);

    SDD(std::string_view path);

    SDD(const Size& size, std::string_view output_path);

    void DrawMarker(const Marker& marker) noexcept;

    void DrawPolyline(const Polyline& polyline) noexcept;

    Size GetSize() const noexcept;

    template<typename T, std::enable_if_t<std::is_arithmetic_v<T>>>
    static std::string ToString(T value) noexcept {
        std::stringstream sstream;
        sstream << value;
        return sstream.str();
    }

    static std::string ToString(Color color) noexcept;

    ~SDD();

  private:
    void DrawTitle(pugi::xml_node& node, const std::string& id) noexcept;

    std::string output_path_;

    pugi::xml_document doc_;
    pugi::xml_node root_;
};

}  // namespace truck::sdd