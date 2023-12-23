#include "svg_processor/svg.h"

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace truck::svg_processor::svg {

ClassConstIterator::ClassConstIterator() : it_(ValueType()) {}

ClassConstIterator::ClassConstIterator(const ValueType& it) : it_(it) {}

const elements::Element& ClassConstIterator::operator*() const { return **it_; }

const elements::Element* ClassConstIterator::operator->() const { return &**it_; }

ClassConstIterator& ClassConstIterator::operator++() {
    ++it_;
    return *this;
}

ClassConstIterator ClassConstIterator::operator++(int) {
    ClassConstIterator tmp = *this;
    ++it_;
    return tmp;
}

ClassConstIterator& ClassConstIterator::operator--() {
    --it_;
    return *this;
}

ClassConstIterator ClassConstIterator::operator--(int) {
    ClassConstIterator tmp = *this;
    --it_;
    return tmp;
}

bool operator==(const ClassConstIterator& first, const ClassConstIterator& second) {
    return first.it_ == second.it_;
}

bool operator!=(const ClassConstIterator& first, const ClassConstIterator& second) {
    return !(first == second);
}

SVG::SVG() : svg_("svg") {}

SVG::SVG(std::string_view filename) : svg_("svg") { Load(filename); }

SVG::SVG(const SVG& other) : svg_(other.svg_) {
    for (auto element : other.elements_) {
        AddElement(std::move(element));
    }
}

SVG::SVG(SVG&& other) : SVG() {
    std::swap(svg_, other.svg_);
    std::swap(elements_, other.elements_);
    std::swap(elements_by_id_, other.elements_by_id_);
    std::swap(elements_by_class_, other.elements_by_class_);
}

SVG& SVG::operator=(SVG other) & {
    std::swap(svg_, other.svg_);
    std::swap(elements_, other.elements_);
    std::swap(elements_by_id_, other.elements_by_id_);
    std::swap(elements_by_class_, other.elements_by_class_);
    return *this;
}

SVG& SVG::Load(std::string_view filename) {
    elements_.clear();
    elements_by_id_.clear();
    elements_by_class_.clear();

    std::ifstream file(filename.data(), std::ios::in);
    if (file.is_open()) {
        std::string line;
        getline(file, line);
        svg_ = elements::deserializers::Deserializer()(line);

        while (getline(file, line)) {
            if (line.starts_with("</")) {
                continue;
            }
            AddElement(elements::deserializers::Deserializer()(line));
        }

        file.close();
    }

    return *this;
}

void SVG::Save(std::string_view filename) {
    std::ofstream file(filename.data(), std::ios::trunc);

    if (file.is_open()) {
        std::string header = "<" + svg_.GetName();
        for (const auto& [attribute_tag, serialized_attribute] : svg_.GetAttributesStorage()) {
            header += " " + serialized_attribute;
        }
        header += ">\n";
        file << header;

        for (const auto& element : elements_) {
            file << '\t' << elements::serializers::Serializer()(element) << '\n';
        }

        std::string footer = "</" + svg_.GetName() + ">\n";
        file << footer;

        file << std::flush;

        file.close();
    }
}

SVG& SVG::AddElement(elements::Element&& element) {
    ElementsConstIterator it = elements_.insert(elements_.cend(), std::move(element));
    auto id = it->GetAttribute<attributes::IntAttribute>("id");
    if (id) {
        auto inserted = elements_by_id_.try_emplace(**id, it);
        if (!inserted.second) {
            throw std::logic_error("Got two elements with same id\n");
        }
    }
    auto classes = it->GetAttribute<attributes::MultistringAttribute>("class");
    if (classes) {
        for (const auto& classname : **classes) {
            elements_by_class_[classname].push_back(it);
        }
    }
    return *this;
}

SVG::ElementsConstIterator SVG::GetElement(int element_id) const {
    auto it = elements_by_id_.find(element_id);
    if (it == elements_by_id_.end()) {
        return elements_.end();
    }
    return it->second;
}

ClassConstIterator SVG::ClassCBegin(const std::string& classname) const {
    auto it = elements_by_class_.find(classname);
    if (it == elements_by_class_.end()) {
        return ClassConstIterator();
    }
    return ClassConstIterator(it->second.cbegin());
}

ClassConstIterator SVG::ClassCEnd(const std::string& classname) const {
    auto it = elements_by_class_.find(classname);
    if (it == elements_by_class_.end()) {
        return ClassConstIterator();
    }
    return ClassConstIterator(it->second.cend());
}

SVG::ElementsConstIterator SVG::ElementsCBegin() const { return elements_.cbegin(); }

SVG::ElementsConstIterator SVG::ElementsCEnd() const { return elements_.cend(); }

const elements::Element::NameType& SVG::GetName() const { return svg_.GetName(); }

const elements::Element::AttributesStorageType& SVG::GetAttributesStorage() const {
    return svg_.GetAttributesStorage();
}

}  // namespace truck::svg_processor::svg