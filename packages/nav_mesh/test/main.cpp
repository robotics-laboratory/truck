#include <gtest/gtest.h>

#include "map/map.h"
#include "nav_mesh/nav_mesh_builder.h"
#include "nav_mesh/nav_mesh_viewer.h"

using namespace truck;
using namespace truck::nav_mesh;

const std::string ROOT = "/truck/packages";

TEST(NavMesh, draw_map) {
    const std::string MAP = "map_1";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::NavMeshViewerParams viewer_params{
        .path = ROOT + "/nav_mesh/data/" + MAP + ".png",
        .color_rgb = {},
        .thickness = {},
        .enable = {.polygon = true}};

    viewer::NavMeshViewer viewer = viewer::NavMeshViewer();
    viewer.draw(viewer_params, polygons, {});
}

TEST(NavMesh, draw_map_with_mesh) {
    const std::string MAP = "map_2";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::NavMeshViewerParams viewer_params{
        .path = ROOT + "/nav_mesh/data/" + MAP + ".png",
        .color_rgb = {},
        .thickness = {},
        .enable = {
            .polygon = true,
            .mesh = true,
        }};

    builder::NavMeshParams params{.dist = 3.0, .offset = 2.4, .filter = {}};
    builder::NavMeshBuilder builder = builder::NavMeshBuilder(params);
    builder::NavMeshBuild build = builder.build(polygons);

    viewer::NavMeshViewer viewer = viewer::NavMeshViewer();
    viewer.draw(viewer_params, polygons, build);
}

TEST(NavMesh, draw_map_with_mesh_and_debug_info) {
    const std::string MAP = "map_3";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::NavMeshViewerParams viewer_params{
        .path = ROOT + "/nav_mesh/data/" + MAP + ".png",
        .color_rgb = {},
        .thickness = {},
        .enable = {
            .polygon = true,
            .skeleton = true,
            .level_lines = true,
            .mesh = true,
        }};

    builder::NavMeshParams params{.dist = 1.5, .offset = 1.7, .filter = {}};
    builder::NavMeshBuilder builder = builder::NavMeshBuilder(params);
    builder::NavMeshBuild build = builder.build(polygons);

    viewer::NavMeshViewer viewer = viewer::NavMeshViewer();
    viewer.draw(viewer_params, polygons, build);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}