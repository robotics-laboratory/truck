#include <gtest/gtest.h>

#include "map/map_builder.h"
#include "nav_mesh/nav_mesh.h"

using namespace truck::map;
using namespace truck::nav_mesh;

TEST(NavMesh, draw) {
    NavMesh nav_mesh = NavMesh(
        NavMeshParams{
            .dist = 1.4,
            .offset = 3.0,
            .grid_filter = false
        })
        .setPolygons(Map::fromGeoJson("/truck/packages/map/data/map_6.geojson").polygons())
        .build();

    nav_mesh.draw(
        DrawParams {
            .path = "/truck/packages/nav_mesh/data/map_6.png",

            .size = DrawParams::Size {
                .pixels = 2000,
                .scale = 40
            },

            .color = DrawParams::Color {
                .background = { 30, 30, 30 },
                .outer_poly = { 252, 252, 252 },
                .inner_poly = { 227, 227, 227 },
                .offset_poly = { 255, 0, 0 },
                .straight_skeleton = { 0, 200, 0 },
                .mesh = { 0, 0, 255 }
            },

            .show = DrawParams::Show {
                .mesh = true,
                .polys = true,
                .offset_polys = true,
                .straight_skeleton = true,
            }
        }
    );
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}