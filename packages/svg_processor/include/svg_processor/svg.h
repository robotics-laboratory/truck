#pragma once

#include "svg_processor/attributes.h"
#include "svg_processor/elements.h"

#include <list>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace truck::svg_processor::svg {

class ClassConstIterator {
  public:
    using ValueType = std::vector<std::list<elements::Element>::const_iterator>::const_iterator;

    ClassConstIterator();

    explicit ClassConstIterator(const ValueType& it);

    const elements::Element& operator*() const;

    const elements::Element* operator->() const;

    ClassConstIterator& operator++();

    ClassConstIterator operator++(int);

    ClassConstIterator& operator--();

    ClassConstIterator operator--(int);

    friend bool operator==(const ClassConstIterator& first, const ClassConstIterator& second);

    friend bool operator!=(const ClassConstIterator& first, const ClassConstIterator& second);

  private:
    ValueType it_;
};

bool operator==(const ClassConstIterator& first, const ClassConstIterator& second);

bool operator!=(const ClassConstIterator& first, const ClassConstIterator& second);

class SVG {
  private:
    friend ClassConstIterator;

  public:
    using ElementsConstIterator = std::list<elements::Element>::const_iterator;

    SVG();

    explicit SVG(std::string_view filename);

    SVG(const SVG& other);

    SVG(SVG&& other);

    SVG& operator=(SVG other) &;

    SVG& Load(std::string_view filename);

    void Save(std::string_view filename);

    SVG& AddElement(elements::Element&& element);

    ElementsConstIterator GetElement(int element_id) const;

    ClassConstIterator ClassCBegin(const std::string& classname) const;

    ClassConstIterator ClassCEnd(const std::string& classname) const;

    ElementsConstIterator ElementsCBegin() const;

    ElementsConstIterator ElementsCEnd() const;

    const elements::Element::NameType& GetName() const;

    template<typename T, typename = std::enable_if_t<attributes::IsAttributeType<T>>>
    std::optional<T> GetAttribute(const std::string& key) const {
        return svg_.GetAttribute<T>(key);
    }

    const elements::Element::AttributesStorageType& GetAttributesStorage() const;

    template<typename T, typename = std::enable_if_t<attributes::IsAttributeType<T>>>
    SVG& SetAttribute(const T& attribute) {
        svg_.SetAttribute<T>(attribute);
        return *this;
    }

    ~SVG() = default;

  private:
    elements::Element svg_;
    std::list<elements::Element> elements_;
    std::unordered_map<int, ElementsConstIterator> elements_by_id_;
    std::unordered_map<std::string, std::vector<ElementsConstIterator>> elements_by_class_;
};

}  // namespace truck::svg_processor::svg