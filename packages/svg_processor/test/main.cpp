#include <gtest/gtest.h>

#include "geom/vector.h"

#include "svg_processor/attributes.h"
#include "svg_processor/elements.h"
#include "svg_processor/svg.h"

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <iostream>

using namespace truck::geom;
using namespace truck::svg_processor;

TEST(ObjectSerializers, serialization) {
    EXPECT_EQ(object::serializers::Serializer<int>()(100), "100");
    EXPECT_EQ(object::serializers::Serializer<float>()(42.42), "42.42");
    EXPECT_EQ(object::serializers::Serializer<double>()(179.179), "179.179");
    EXPECT_EQ(object::serializers::Serializer<std::string>()("abc"), "abc");
    EXPECT_EQ(
        object::serializers::Serializer<std::vector<Vec2>>()(
            {Vec2(100.0, 100.0), Vec2(-150, -150), Vec2(178.5, 11.179)}),
        "100,100 -150,-150 178.5,11.179");
    EXPECT_EQ(
        object::serializers::Serializer<std::vector<std::string>>()({"class1", "class2", "class3"}),
        "class1 class2 class3");
}

TEST(ObjectDeserializers, deserialization) {
    EXPECT_EQ(object::deserializers::Deserializer<int>()("  100 "), 100);
    EXPECT_FLOAT_EQ(object::deserializers::Deserializer<float>()("  42.42 "), 42.42);
    EXPECT_DOUBLE_EQ(object::deserializers::Deserializer<double>()("  179.179 "), 179.179);
    EXPECT_EQ(object::deserializers::Deserializer<std::string>()("  abc "), "abc");
    auto points = object::deserializers::Deserializer<std::vector<Vec2>>()(
        "100,100 -150.0,-150.0 178.5,11.179");
    std::vector<Vec2> expected_value = {Vec2(100, 100), Vec2(-150, -150), Vec2(178.5, 11.179)};
    EXPECT_DOUBLE_EQ(points[0].x, expected_value[0].x);
    EXPECT_DOUBLE_EQ(points[0].y, expected_value[0].y);
    EXPECT_DOUBLE_EQ(points[1].x, expected_value[1].x);
    EXPECT_DOUBLE_EQ(points[1].y, expected_value[1].y);
    EXPECT_DOUBLE_EQ(points[2].x, expected_value[2].x);
    EXPECT_DOUBLE_EQ(points[2].y, expected_value[2].y);
    std::vector<std::string> expected_value2{"class1", "class2", "class3"};
    EXPECT_EQ(
        object::deserializers::Deserializer<std::vector<std::string>>()("class1  class2   class3 "),
        expected_value2);
}

TEST(AttributeSerializers, serialization) {
    attributes::IntAttribute attr1("x", 100);
    EXPECT_EQ(attributes::serializers::Serializer<attributes::IntAttribute>()(attr1), "x=\"100\"");
    attributes::FloatAttribute attr2("y", 42.42);
    EXPECT_EQ(
        attributes::serializers::Serializer<attributes::FloatAttribute>()(attr2), "y=\"42.42\"");
    attributes::DoubleAttribute attr3("z", 179.179);
    EXPECT_EQ(
        attributes::serializers::Serializer<attributes::DoubleAttribute>()(attr3), "z=\"179.179\"");
    attributes::StringAttribute attr4("name", "abc");
    EXPECT_EQ(
        attributes::serializers::Serializer<attributes::StringAttribute>()(attr4), "name=\"abc\"");
    attributes::PointsAttriubte attr5(
        "points", {Vec2(100.0, 100.0), Vec2(150, 150), Vec2(178.5, 11.179)});
    EXPECT_EQ(
        attributes::serializers::Serializer<attributes::PointsAttriubte>()(attr5),
        "points=\"100,100 150,150 178.5,11.179\"");
    attributes::MultistringAttribute attr6("class", {"class1", "class2", "class3"});
    EXPECT_EQ(
        attributes::serializers::Serializer<attributes::MultistringAttribute>()(attr6),
        "class=\"class1 class2 class3\"");
}

TEST(AttributeDeserializers, deserialization) {
    auto attr1 =
        attributes::deserializers::Deserializer<attributes::IntAttribute>()("x = \"  100 \"");
    EXPECT_EQ(attr1.GetTag(), "x");
    EXPECT_EQ(attr1.GetObject(), 100);
    auto attr2 =
        attributes::deserializers::Deserializer<attributes::FloatAttribute>()(" y = \"42.42\" ");
    EXPECT_EQ(attr2.GetTag(), "y");
    EXPECT_FLOAT_EQ(attr2.GetObject(), 42.42);
    auto attr3 =
        attributes::deserializers::Deserializer<attributes::DoubleAttribute>()(" z=\" 179.179 \"");
    EXPECT_EQ(attr3.GetTag(), "z");
    EXPECT_DOUBLE_EQ(attr3.GetObject(), 179.179);
    auto attr4 =
        attributes::deserializers::Deserializer<attributes::StringAttribute>()(" name= \" abc\" ");
    EXPECT_EQ(attr4.GetTag(), "name");
    EXPECT_EQ(attr4.GetObject(), "abc");
    auto attr5 = attributes::deserializers::Deserializer<attributes::PointsAttriubte>()(
        "points=\"100,100 150.0,150.0 178.5,11.179\"");
    std::vector<Vec2> expected_value5 = {Vec2(100, 100), Vec2(150, 150), Vec2(178.5, 11.179)};
    EXPECT_EQ(attr5.GetTag(), "points");
    EXPECT_DOUBLE_EQ(attr5.GetObject()[0].x, expected_value5[0].x);
    EXPECT_DOUBLE_EQ(attr5.GetObject()[0].y, expected_value5[0].y);
    EXPECT_DOUBLE_EQ(attr5.GetObject()[1].x, expected_value5[1].x);
    EXPECT_DOUBLE_EQ(attr5.GetObject()[1].y, expected_value5[1].y);
    EXPECT_DOUBLE_EQ(attr5.GetObject()[2].x, expected_value5[2].x);
    EXPECT_DOUBLE_EQ(attr5.GetObject()[2].y, expected_value5[2].y);
    auto attr6 = attributes::deserializers::Deserializer<attributes::MultistringAttribute>()(
        "class = \"class1  class2   class3 \"");
    std::vector<std::string> expected_value6{"class1", "class2", "class3"};
    EXPECT_EQ(attr6.GetTag(), "class");
    EXPECT_EQ(attr6.GetObject(), expected_value6);
}

TEST(ElementsSerializers, serialization) {
    elements::Element circle("circle");
    circle.SetAttribute(attributes::DoubleAttribute("cx", 10.10))
        .SetAttribute(attributes::DoubleAttribute("cy", 20.20));
    EXPECT_EQ(circle.GetName(), "circle");
    auto circle_cx = circle.GetAttribute<attributes::DoubleAttribute>("cx");
    EXPECT_EQ(circle_cx->GetTag(), "cx");
    EXPECT_DOUBLE_EQ(circle_cx->GetObject(), 10.10);
    auto circle_r = circle.GetAttribute<attributes::DoubleAttribute>("r");
    EXPECT_EQ(circle_r, std::nullopt);
    elements::SerializedElement serialized_circle = elements::serializers::Serializer()(circle);
    EXPECT_EQ(
        (serialized_circle == "<circle cx=\"10.1\" cy=\"20.2\"\\>") ||
            (serialized_circle == "<circle cy=\"20.2\" cx=\"10.1\"\\>"),
        true);
    circle.SetAttribute("cx", -1.1);
    serialized_circle = elements::serializers::Serializer()(circle);
    EXPECT_EQ(
        (serialized_circle == "<circle cx=\"-1.1\" cy=\"20.2\"\\>") ||
            (serialized_circle == "<circle cy=\"20.2\" cx=\"-1.1\"\\>"),
        true);
    elements::Element polyline("pl");
    polyline.SetName("polyline")
        .SetAttribute(attributes::PointsAttriubte("points", {Vec2(1, 2), Vec2(3, 4)}));
    EXPECT_EQ(polyline.GetName(), "polyline");
    auto polyline_points = polyline.GetAttribute<attributes::PointsAttriubte>("points");
    std::vector<Vec2> expected_points = {Vec2(1, 2), Vec2(3, 4)};
    EXPECT_DOUBLE_EQ(polyline_points->GetObject()[0].x, expected_points[0].x);
    EXPECT_DOUBLE_EQ(polyline_points->GetObject()[0].y, expected_points[0].y);
    EXPECT_DOUBLE_EQ(polyline_points->GetObject()[1].x, expected_points[1].x);
    EXPECT_DOUBLE_EQ(polyline_points->GetObject()[1].y, expected_points[1].y);
    elements::SerializedElement serialized_polyline = elements::serializers::Serializer()(polyline);
    EXPECT_EQ(serialized_polyline, "<polyline points=\"1,2 3,4\"\\>");
}

TEST(ElementsDeserializers, deserialization) {
    auto circle = elements::deserializers::Deserializer()(
        "<  circle    cx =  \" 10.1 \"   cy  = \"20.2 \"  \\>");
    EXPECT_EQ(circle.GetName(), "circle");
    EXPECT_EQ(circle.GetAttribute<attributes::IntAttribute>("cx")->GetTag(), "cx");
    EXPECT_DOUBLE_EQ(circle.GetAttribute<attributes::DoubleAttribute>("cx")->GetObject(), 10.1);
    EXPECT_EQ(circle.GetAttribute<attributes::IntAttribute>("cy")->GetTag(), "cy");
    EXPECT_EQ(circle.GetAttribute<attributes::IntAttribute>("cy")->GetObject(), 20);
    EXPECT_EQ(circle.GetAttribute<attributes::PointsAttriubte>("r"), std::nullopt);
    auto polyline = elements::deserializers::Deserializer()(
        "<polyline points=\"1,2 3,4\" id=\"0\" class=\"class1 class2\"\\>");
    EXPECT_EQ(polyline.GetName(), "polyline");
    EXPECT_EQ(polyline.GetAttribute<attributes::PointsAttriubte>("points")->GetTag(), "points");
    auto polyline_points = polyline.GetAttribute<attributes::PointsAttriubte>("points");
    std::vector<Vec2> expected_points = {Vec2(1, 2), Vec2(3, 4)};
    EXPECT_DOUBLE_EQ(polyline_points->GetObject()[0].x, expected_points[0].x);
    EXPECT_DOUBLE_EQ(polyline_points->GetObject()[0].y, expected_points[0].y);
    EXPECT_DOUBLE_EQ(polyline_points->GetObject()[1].x, expected_points[1].x);
    EXPECT_DOUBLE_EQ(polyline_points->GetObject()[1].y, expected_points[1].y);
    EXPECT_EQ(polyline.GetAttribute<attributes::MultistringAttribute>("class")->GetTag(), "class");
    auto polyline_class = std::vector<std::string>{"class1", "class2"};
    EXPECT_EQ(
        polyline.GetAttribute<attributes::MultistringAttribute>("class")->GetObject(),
        polyline_class);
}

TEST(SVG, Load) {
    svg::SVG image("/root/truck/packages/svg_processor/data/test_in.svg");

    EXPECT_EQ(image.GetName(), "svg");
    EXPECT_EQ(**image.GetAttribute<attributes::IntAttribute>("width"), 512);
    EXPECT_EQ(
        **image.GetAttribute<attributes::StringAttribute>("xmlns"), "http://www.w3.org/2000/svg");

    EXPECT_EQ(image.GetElement(0), image.ElementsCEnd());

    auto circle_it = image.GetElement(1);
    EXPECT_DOUBLE_EQ(**circle_it->GetAttribute<attributes::DoubleAttribute>("cx"), 11.11);
    EXPECT_DOUBLE_EQ(**circle_it->GetAttribute<attributes::DoubleAttribute>("cy"), 22.22);
    EXPECT_EQ(**circle_it->GetAttribute<attributes::StringAttribute>("color"), "red");

    auto class_it = image.ClassCBegin("round");
    EXPECT_FALSE(class_it == image.ClassCEnd("round"));
    EXPECT_DOUBLE_EQ(**class_it->GetAttribute<attributes::DoubleAttribute>("cx"), 11.11);
    EXPECT_DOUBLE_EQ(**class_it->GetAttribute<attributes::DoubleAttribute>("cy"), 22.22);
    EXPECT_EQ(**class_it->GetAttribute<attributes::StringAttribute>("color"), "red");

    auto rect_it = image.GetElement(2);
    EXPECT_DOUBLE_EQ(**rect_it->GetAttribute<attributes::DoubleAttribute>("x"), 13.13);
    EXPECT_EQ(**rect_it->GetAttribute<attributes::IntAttribute>("y"), 179);

    int i = 0;
    class_it = image.ClassCBegin("figures");
    for (; class_it != image.ClassCEnd("figures"); ++i, ++class_it) {
        EXPECT_EQ(class_it->GetName(), image.GetElement(i + 1)->GetName());
        EXPECT_EQ((*class_it).GetName(), image.GetElement(i + 1)->GetName());
    }
    EXPECT_EQ(i, 2);

    --class_it;
    --i;
    for (; i >= 0; --i, --class_it) {
        EXPECT_EQ(class_it->GetName(), image.GetElement(i + 1)->GetName());
        EXPECT_EQ((*class_it).GetName(), image.GetElement(i + 1)->GetName());
    }
    EXPECT_EQ(i, -1);

    i = 0;
    auto elements_it = image.ElementsCBegin();
    for (; elements_it != image.ElementsCEnd(); ++i, ++elements_it) {
        EXPECT_EQ(elements_it->GetName(), image.GetElement(i + 1)->GetName());
        EXPECT_EQ((*elements_it).GetName(), image.GetElement(i + 1)->GetName());
    }
    EXPECT_EQ(i, 2);
}

TEST(SVG, Save) {
    svg::SVG image("/root/truck/packages/svg_processor/data/test_in.svg");

    elements::Element line("line");
    line.SetAttribute(attributes::IntAttribute("x1", 10))
        .SetAttribute(attributes::IntAttribute("y1", 10))
        .SetAttribute(attributes::IntAttribute("x2", 20))
        .SetAttribute(attributes::IntAttribute("y2", 20))
        .SetAttribute(attributes::IntAttribute("id", 3))
        .SetAttribute(attributes::StringAttribute("class", "figures"));
    image.AddElement(std::move(line));

    auto line_it = image.GetElement(3);
    EXPECT_EQ(line_it->GetName(), "line");

    int i = 0;
    auto class_it = image.ClassCBegin("figures");
    for (; class_it != image.ClassCEnd("figures"); ++i, ++class_it) {
        EXPECT_EQ(class_it->GetName(), image.GetElement(i + 1)->GetName());
        EXPECT_EQ((*class_it).GetName(), image.GetElement(i + 1)->GetName());
    }
    EXPECT_EQ(i, 3);

    auto it = image.GetElement(4);
    EXPECT_EQ(it, image.ElementsCEnd());

    elements::Element polyline("pl");
    polyline.SetName("polyline")
        .SetAttribute(attributes::PointsAttriubte("points", {Vec2(1, 2), Vec2(3, 4)}))
        .SetAttribute(attributes::StringAttribute("class", "figures"));

    image.AddElement(std::move(polyline));

    it = image.GetElement(4);
    EXPECT_EQ(it, image.ElementsCEnd());

    i = 0;
    class_it = image.ClassCBegin("figures");
    for (; class_it != image.ClassCEnd("figures"); ++i, ++class_it) {
    }
    EXPECT_EQ(i, 4);

    image.Save("/root/truck/packages/svg_processor/data/test_out.svg");
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}