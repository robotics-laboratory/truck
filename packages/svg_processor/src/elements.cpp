#include "svg_processor/elements.h"

#include <algorithm>

namespace truck::svg_processor::elements {

Element::Element(std::string_view name) : name_(name) {}

Element::Element(std::string_view name, AttributesStorageType attributes)
    : name_(name), attributes_(std::move(attributes)) {}

Element::Element(const Element& other) : name_(other.name_), attributes_(other.attributes_) {}

Element::Element(Element&& other) : Element("") {
    std::swap(name_, other.name_);
    std::swap(attributes_, other.attributes_);
}

Element& Element::operator=(Element other) & {
    std::swap(name_, other.name_);
    std::swap(attributes_, other.attributes_);
    return *this;
}

Element::NameType& Element::GetName() { return name_; }

const Element::NameType& Element::GetName() const { return name_; }

const Element::AttributesStorageType& Element::GetAttributesStorage() const { return attributes_; }

Element& Element::SetName(std::string_view name) {
    name_ = name;
    return *this;
}

namespace serializers {

elements::SerializedElement Serializer::operator()(const Element& element) const {
    elements::SerializedElement result;
    result = "<" + element.name_;
    for (const auto& [attribute_tag, serialized_attribute] : element.attributes_) {
        result += " " + serialized_attribute;
    }
    result += "\\>";
    return result;
}

}  // namespace serializers

namespace deserializers {

Element Deserializer::operator()(const elements::SerializedElement& data) const {
    auto name_begin = std::find_if(
        data.begin(), data.end(), [](const char ch) -> bool { return std::isalpha(ch); });
    auto name_end = std::find_if(
        name_begin, data.end(), [](const char ch) -> bool { return std::isspace(ch); });
    Element element(std::string(name_begin, name_end));
    auto attribute_begin =
        std::find_if(name_end, data.end(), [](const char ch) -> bool { return std::isalpha(ch); });
    while (attribute_begin != data.end()) {
        auto attribute_end =
            std::find(std::find(attribute_begin, data.end(), '\"') + 1, data.end(), '\"') + 1;
        attributes::SerializedAttribute attribute(attribute_begin, attribute_end);
        std::string attribute_tag =
            attributes::deserializers::Deserializer<attributes::StringAttribute>()(attribute)
                .GetTag();
        element.attributes_.emplace(std::make_pair(attribute_tag, attribute));
        attribute_begin = std::find_if(
            attribute_end, data.end(), [](const char ch) -> bool { return std::isalpha(ch); });
    }
    return element;
}

}  // namespace deserializers

}  // namespace truck::svg_processor::elements