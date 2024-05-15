#include <gtest/gtest.h>

#include <string>
#include <iostream>

#include "geom/test/equal_assert.h"

#include "svg_debug_drawer/sdd.h"

using namespace truck;

TEST(SDD, Constructor) {
    {
        const std::string input_path = "/truck/packages/svg_debug_drawer/test/data/im1_in.svg";
        const std::string output_path = "/truck/packages/svg_debug_drawer/test/data/im1_out.svg";

        sdd::SDD(input_path, output_path);
    }
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/test/data/im2_out.svg";

        sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
    }
}

TEST(SDD, DrawMarker) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/test/data/im3_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        img.Add({.point = geom::Vec2(128, 128),
                 .scale = 10.0,
                 .color = sdd::color::aqua,
                 .label = "1"})
            .Add(
                {.point = geom::Vec2(256, 256),
                 .shape = sdd::Marker::Shape::Square,
                 .scale = 10.0,
                 .color = sdd::color::lime,
                 .label = "2"})
            .Add(geom::Vec2(384, 384));
        img << sdd::Marker{
            .point = geom::Vec2(500, 500),
            .shape = sdd::Marker::Shape::Square,
            .scale = 10.0,
            .color = sdd::color::maroon,
            .label = "4"} << geom::Vec2(400, 400);
    }
}

TEST(SDD, DrawPose) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/test/data/im4_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        img.Add({.pose = geom::Pose(geom::Vec2(128, 128), geom::AngleVec2::fromVector(1, 0)),
                 .scale = 10.0,
                 .length = 10.0,
                 .color = sdd::color::blue,
                 .label = "1"})
            .Add(
                {.pose = geom::Pose(geom::Vec2(256, 256), geom::AngleVec2::fromVector(1, 1)),
                 .scale = 10.0,
                 .length = 10.0,
                 .color = sdd::color::green,
                 .label = "2"})
            .Add(geom::Pose(geom::Vec2(384, 384), geom::AngleVec2::fromVector(-1, 1)));
        img << sdd::Pose{
            .pose = geom::Pose(geom::Vec2(500, 500), geom::AngleVec2::fromVector(-1, 0)),
            .scale = 10.0,
            .length = 10.0,
            .color = sdd::color::red,
            .label = "4"} << geom::Pose(geom::Vec2(400, 400), geom::AngleVec2::fromVector(0, -1));
    }
}

TEST(SDD, DrawPolyline) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/test/data/im5_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        img.Add({.polyline = {geom::Vec2(10, 10), geom::Vec2(100, 100), geom::Vec2(200, 250)},
                 .thickness = 10.0,
                 .color = sdd::color::red,
                 .label = "1"})
            .Add(geom::Polyline{geom::Vec2(500, 500), geom::Vec2(412, 412), geom::Vec2(312, 262)});
        img << sdd::Polyline{
            .polyline = {geom::Vec2(10, 500), geom::Vec2(100, 400), geom::Vec2(200, 500)},
            .thickness = 5.0,
            .color = sdd::color::blue,
            .label = "2"} << geom::Polyline{geom::Vec2(500, 10), geom::Vec2(412, 112), geom::Vec2(312, 12)};
    }
}

TEST(SDD, DrawPolygon) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/test/data/im6_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        img.Add({.polygon =
                     {geom::Vec2(0, 0),
                      geom::Vec2(0, 100),
                      geom::Vec2(200, 250),
                      geom::Vec2(180, 11.1)},
                 .color = sdd::color::yellow,
                 .label = "1"})
            .Add(
                {.polygon =
                     {geom::Vec2(10, 500),
                      geom::Vec2(10, 400),
                      geom::Vec2(100, 400),
                      geom::Vec2(100, 500)},
                 .border_thickness = 5.0,
                 .fill = false,
                 .color = sdd::color::teal,
                 .label = "2"})
            .Add(geom::Polygon{
                geom::Vec2(500, 500),
                geom::Vec2(400, 500),
                geom::Vec2(400, 400),
                geom::Vec2(500, 400)});
        img << sdd::Polygon{
            .polygon =
                {geom::Vec2(500, 10),
                 geom::Vec2(400, 10),
                 geom::Vec2(400, 100),
                 geom::Vec2(500, 100)},
            .border_thickness = 5.0,
            .fill = false,
            .color = sdd::color::navy,
            .label = "3"} << geom::Polygon{geom::Vec2(300, 300),
                                            geom::Vec2(300, 350),
                                            geom::Vec2(350, 350),
                                            geom::Vec2(350, 300)};
    }
}

TEST(SDD, DrawComplexPolygon) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/test/data/im7_out.svg";

        auto img = sdd::SDD(sdd::Size{.width = 512, .height = 512}, output_path);
        img.Add({.complex_polygon =
                     {geom::Polygon{
                          geom::Vec2(10, 10),
                          geom::Vec2(10, 200),
                          geom::Vec2(200, 200),
                          geom::Vec2(200, 10)},
                      {geom::Polygon{
                           geom::Vec2(20, 20),
                           geom::Vec2(40, 20),
                           geom::Vec2(40, 40),
                           geom::Vec2(20, 40)},
                       geom::Polygon{
                           geom::Vec2(100, 100),
                           geom::Vec2(120, 100),
                           geom::Vec2(120, 120),
                           geom::Vec2(100, 120)}}},
                 .color = sdd::color::fuchsia,
                 .label = "1"})
            .Add(
                {.point = geom::Vec2(30, 30),
                 .scale = 10.0,
                 .color = sdd::color::red,
                 .label = "2"})
            .Add(
                {.complex_polygon =
                     {geom::Polygon{
                          geom::Vec2(500, 500),
                          geom::Vec2(500, 300),
                          geom::Vec2(300, 300),
                          geom::Vec2(300, 500)},
                      {geom::Polygon{
                           geom::Vec2(480, 480),
                           geom::Vec2(460, 480),
                           geom::Vec2(460, 460),
                           geom::Vec2(480, 460)},
                       geom::Polygon{
                           geom::Vec2(400, 400),
                           geom::Vec2(380, 400),
                           geom::Vec2(380, 380),
                           geom::Vec2(400, 380)}}},
                 .border_thickness = 5.0,
                 .fill = false,
                 .color = sdd::color::lime,
                 .label = "3"})
            .Add(
                {.point = geom::Vec2(470, 470),
                 .scale = 10.0,
                 .color = sdd::color::red,
                 .label = "4"})
            .Add(
                {geom::Polygon{
                     geom::Vec2(10, 500),
                     geom::Vec2(10, 300),
                     geom::Vec2(200, 300),
                     geom::Vec2(200, 500)},
                 {geom::Polygon{
                     geom::Vec2(100, 400),
                     geom::Vec2(150, 400),
                     geom::Vec2(150, 450),
                     geom::Vec2(100, 450)}}});

        img << geom::ComplexPolygon{
            geom::Polygon{
                geom::Vec2(500, 10),
                geom::Vec2(500, 200),
                geom::Vec2(300, 200),
                geom::Vec2(300, 10)},
            {geom::Polygon{
                 geom::Vec2(480, 20),
                 geom::Vec2(460, 20),
                 geom::Vec2(460, 40),
                 geom::Vec2(480, 40)},
             geom::Polygon{
                 geom::Vec2(400, 100),
                 geom::Vec2(380, 100),
                 geom::Vec2(380, 120),
                 geom::Vec2(400, 120)}}};
    }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
