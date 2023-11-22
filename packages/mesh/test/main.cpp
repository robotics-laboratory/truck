#include <gtest/gtest.h>

#include <mesh/mesh_builder.h>

using namespace truck::mesh;

TEST(MeshBuilder, draw_map_1) {
    MeshBuilderParams params {
        .input_file = "map_1.geojson",
        .output_file = "map_1.png",
        .package_data_path = "/truck/packages/mesh/data",
        .img_size_pixel = 1000,
        .meter_to_pixel_scale = 20,
        .thickness = 1,
        .color = MeshBuilderParams::Color {
            .background = { 30, 30, 30 },
            .straight_skeleton = { 255, 0, 0 },
            .poly_outer = { 252, 252, 252 },
            .poly_hole = { 227, 227, 227 },
            .contour = { 150, 150, 150 }
        }
    };

    MeshBuilder mesh = MeshBuilder(params);
    mesh.drawMap();
}

TEST(MeshBuilder, draw_map_2) {
    MeshBuilderParams params {
        .input_file = "map_2.geojson",
        .output_file = "map_2.png",
        .package_data_path = "/truck/packages/mesh/data",
        .img_size_pixel = 1000,
        .meter_to_pixel_scale = 20,
        .thickness = 1,
        .color = MeshBuilderParams::Color {
            .background = { 30, 30, 30 },
            .straight_skeleton = { 255, 0, 0 },
            .poly_outer = { 252, 252, 252 },
            .poly_hole = { 227, 227, 227 },
            .contour = { 150, 150, 150 }
        }
    };

    MeshBuilder mesh = MeshBuilder(params);
    mesh.drawMap();
}

TEST(MeshBuilder, draw_map_3) {
    MeshBuilderParams params {
        .input_file = "map_3.geojson",
        .output_file = "map_3.png",
        .package_data_path = "/truck/packages/mesh/data",
        .img_size_pixel = 1000,
        .meter_to_pixel_scale = 20,
        .thickness = 1,
        .color = MeshBuilderParams::Color {
            .background = { 30, 30, 30 },
            .straight_skeleton = { 255, 0, 0 },
            .poly_outer = { 252, 252, 252 },
            .poly_hole = { 227, 227, 227 },
            .contour = { 150, 150, 150 }
        }
    };

    MeshBuilder mesh = MeshBuilder(params);
    mesh.drawMap();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}