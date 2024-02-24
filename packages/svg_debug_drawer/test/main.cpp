#include <gtest/gtest.h>

#include <string>
#include <iostream>

#include "svg_debug_drawer/sdd.h"

using namespace truck::sdd;
using namespace truck::geom;

TEST(SDD, Constructor) {
    {
        const std::string input_path = "/truck/packages/svg_debug_drawer/data/im1_in.svg";
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im1_out.svg";

        SDD(input_path, output_path);
    }
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im2_out.svg";

        SDD(Size{.width = 512, .height = 512}, output_path);
    }
}

TEST(SDD, DrawMarker) {
    {
        const std::string output_path = "/truck/packages/svg_debug_drawer/data/im3_out.svg";

        auto img = SDD(Size{.width = 512, .height = 512}, output_path);
        img.DrawMarker({{.id = "1", .color = Color::Aqua}, Vec2(128, 128)});
        img.DrawMarker({{.id = "2", .color = Color::Lime}, Vec2(256, 256), Marker::Type::Square});
    }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}