#include <gtest/gtest.h>

#include <mesh/mesh_builder.h>

using namespace truck::mesh;

TEST(MeshBuilder, draw_polygon_with_straight_skeleton) {
    MeshBuilderParams params {
        .input_file = "map.geojson",
        .output_file = "map.png",
        .package_data_path = "/truck/packages/mesh/data",

        .img_size = 1000,
        .scale = 20,

        .color = MeshBuilderParams::Color {
            .background = { 30, 30, 30 },
            .outer_polygon = { 252, 252, 252 },
            .inner_polygon = { 227, 227, 227 },
            .straight_skeleton = { 0, 255, 255 },
        }
    };

    MeshBuilder mesh = MeshBuilder(params);
    mesh.drawMap();
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}