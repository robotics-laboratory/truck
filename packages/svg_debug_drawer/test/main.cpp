#include <gtest/gtest.h>

#include <string>
#include <iostream>

#include "geom/test/equal_assert.h"

#include "svg_debug_drawer/sdd.h"

using namespace truck;

TEST(SDD, Constructor) {
    {
        const std::string input_path = "/truck/packages/svg_debug_drawer/data/im1_in.svg";
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im1_out.svg";

        sdd::SDD(input_path, output_path);
    }
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im2_out.svg";

        sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
    }
}

TEST(SDD, DrawMarker) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im3_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        img.DrawMarker(geom::Vec2(128, 128), {{.id = "1", .color = sdd::Color::Aqua}});
        img.DrawMarker(
            geom::Vec2(256, 256),
            {{.id = "2", .color = sdd::Color::Lime}, sdd::MarkerDescription::Type::Square});
        img.DrawMarker(geom::Vec2(384, 384));
    }
}

TEST(SDD, DrawPose) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im4_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        img.DrawPose(
            geom::Pose(geom::Vec2(128, 128), geom::AngleVec2::fromVector(1, 0)),
            {.id = "1", .color = sdd::Color::Blue});
        img.DrawPose(
            geom::Pose(geom::Vec2(256, 256), geom::AngleVec2::fromVector(1, 1)),
            {.id = "2", .color = sdd::Color::Green});
    }
}

TEST(SDD, DrawPolyline) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im5_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        geom::Polyline polyline{geom::Vec2(0, 0), geom::Vec2(100, 100), geom::Vec2(200, 250)};
        img.DrawPolyline(polyline, {.id = "1", .color = sdd::Color::Red});
    }
}

TEST(SDD, DrawPolygon) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im6_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        geom::Polygon polygon{
            geom::Vec2(0, 0), geom::Vec2(0, 100), geom::Vec2(200, 250), geom::Vec2(180, 11.1)};
        img.DrawPolygon(polygon, {.id = "1", .color = sdd::Color::Yellow});
    }
}

TEST(SDD, DrawComplexPolygon) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im7_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        geom::ComplexPolygon polygon(
            geom::Polygon{
                geom::Vec2(10, 10), geom::Vec2(10, 500), geom::Vec2(500, 500), geom::Vec2(500, 10)},
            {geom::Polygon{
                 geom::Vec2(20, 20), geom::Vec2(40, 20), geom::Vec2(40, 40), geom::Vec2(20, 40)},
             geom::Polygon{
                 geom::Vec2(100, 100),
                 geom::Vec2(120, 100),
                 geom::Vec2(120, 120),
                 geom::Vec2(100, 120)}});

        img.DrawComplexPolygon(polygon, {.id = "1", .color = sdd::Color::Fuchsia});
        img.DrawMarker(geom::Vec2(30, 30), {{.id = "2", .color = sdd::Color::Red}});
    }
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im8_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        geom::ComplexPolygon polygon(
            geom::Polygon{
                geom::Vec2(10, 10), geom::Vec2(500, 10), geom::Vec2(500, 500), geom::Vec2(10, 500)},
            {geom::Polygon{
                 geom::Vec2(20, 20), geom::Vec2(20, 40), geom::Vec2(40, 40), geom::Vec2(40, 20)},
             geom::Polygon{
                 geom::Vec2(100, 100),
                 geom::Vec2(120, 100),
                 geom::Vec2(120, 120),
                 geom::Vec2(100, 120)}});

        img.DrawComplexPolygon(polygon, {.id = "1", .color = sdd::Color::Fuchsia});
        img.DrawMarker(geom::Vec2(30, 30), {{.id = "2", .color = sdd::Color::Red}});
    }
}

TEST(SDD, FindMarker) {
    const std::string input_path = "/truck/packages/svg_debug_drawer/data/im3_out.svg";
    auto img = sdd::SDD(input_path);

    {
        auto point = img.FindMarker("1");
        ASSERT_GEOM_EQUAL(point, geom::Vec2(128, 128));
    }
    {
        auto point = img.FindMarker("2");
        ASSERT_GEOM_EQUAL(point, geom::Vec2(256, 256));
    }
}

TEST(SDD, FindPose) {
    const std::string input_path = "/truck/packages/svg_debug_drawer/data/im4_out.svg";
    auto img = sdd::SDD(input_path);

    {
        auto pose = img.FindPose("1");
        ASSERT_GEOM_EQUAL(
            pose, geom::Pose(geom::Vec2(128, 128), geom::AngleVec2::fromVector(1, 0)));
    }
    {
        auto pose = img.FindPose("2");
        ASSERT_GEOM_EQUAL(
            pose, geom::Pose(geom::Vec2(256, 256), geom::AngleVec2::fromVector(1, 1)));
    }
}

TEST(SDD, FindPolyline) {
    const std::string input_path = "/truck/packages/svg_debug_drawer/data/im5_out.svg";
    auto img = sdd::SDD(input_path);

    {
        auto polyline = img.FindPolyline("1");
        ASSERT_GEOM_EQUAL(polyline[0], geom::Vec2(0, 0));
        ASSERT_GEOM_EQUAL(polyline[1], geom::Vec2(100, 100));
        ASSERT_GEOM_EQUAL(polyline[2], geom::Vec2(200, 250));
    }
}

TEST(SDD, FindPolygon) {
    const std::string input_path = "/truck/packages/svg_debug_drawer/data/im6_out.svg";
    auto img = sdd::SDD(input_path);

    {
        auto polygon = img.FindPolygon("1");
        ASSERT_GEOM_EQUAL(polygon[0], geom::Vec2(0, 0));
        ASSERT_GEOM_EQUAL(polygon[1], geom::Vec2(0, 100));
        ASSERT_GEOM_EQUAL(polygon[2], geom::Vec2(200, 250));
        ASSERT_GEOM_EQUAL(polygon[3], geom::Vec2(180, 11.1));
    }
}

TEST(SDD, FindComplexPolygon) {
    {
        const std::string input_path = "/truck/packages/svg_debug_drawer/data/im7_out.svg";

        auto img = sdd::SDD(input_path);
        auto polygon = img.FindComplexPolygon("1");
        ASSERT_GEOM_EQUAL(polygon.outer[0], geom::Vec2(10, 10));
        ASSERT_GEOM_EQUAL(polygon.outer[1], geom::Vec2(10, 500));
        ASSERT_GEOM_EQUAL(polygon.outer[2], geom::Vec2(500, 500));
        ASSERT_GEOM_EQUAL(polygon.outer[3], geom::Vec2(500, 10));
        ASSERT_GEOM_EQUAL(polygon.inners[0][0], geom::Vec2(20, 20));
        ASSERT_GEOM_EQUAL(polygon.inners[0][1], geom::Vec2(40, 20));
        ASSERT_GEOM_EQUAL(polygon.inners[0][2], geom::Vec2(40, 40));
        ASSERT_GEOM_EQUAL(polygon.inners[0][3], geom::Vec2(20, 40));
        ASSERT_GEOM_EQUAL(polygon.inners[1][0], geom::Vec2(100, 100));
        ASSERT_GEOM_EQUAL(polygon.inners[1][1], geom::Vec2(120, 100));
        ASSERT_GEOM_EQUAL(polygon.inners[1][2], geom::Vec2(120, 120));
        ASSERT_GEOM_EQUAL(polygon.inners[1][3], geom::Vec2(100, 120));
    }
    {
        const std::string input_path = "/truck/packages/svg_debug_drawer/data/im8_out.svg";

        auto img = sdd::SDD(input_path);
        auto polygon = img.FindComplexPolygon("1");
        ASSERT_GEOM_EQUAL(polygon.outer[0], geom::Vec2(10, 500));
        ASSERT_GEOM_EQUAL(polygon.outer[1], geom::Vec2(500, 500));
        ASSERT_GEOM_EQUAL(polygon.outer[2], geom::Vec2(500, 10));
        ASSERT_GEOM_EQUAL(polygon.outer[3], geom::Vec2(10, 10));
        ASSERT_GEOM_EQUAL(polygon.inners[0][0], geom::Vec2(40, 20));
        ASSERT_GEOM_EQUAL(polygon.inners[0][1], geom::Vec2(40, 40));
        ASSERT_GEOM_EQUAL(polygon.inners[0][2], geom::Vec2(20, 40));
        ASSERT_GEOM_EQUAL(polygon.inners[0][3], geom::Vec2(20, 20));
        ASSERT_GEOM_EQUAL(polygon.inners[1][0], geom::Vec2(100, 100));
        ASSERT_GEOM_EQUAL(polygon.inners[1][1], geom::Vec2(120, 100));
        ASSERT_GEOM_EQUAL(polygon.inners[1][2], geom::Vec2(120, 120));
        ASSERT_GEOM_EQUAL(polygon.inners[1][3], geom::Vec2(100, 120));
    }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}