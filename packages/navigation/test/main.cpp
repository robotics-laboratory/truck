#include <gtest/gtest.h>

#include "map/map.h"
#include "navigation/viewer.h"
#include "navigation/mesh_builder.h"

using namespace truck;
using namespace truck::navigation;

const std::string ROOT = "/truck/packages";

TEST(Navigation, map_empty) {
    const std::string MAP = "map_6";
    const std::string FILE_NAME = MAP + "_empty";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png",
        .color_rgb = {},
        .thickness = {},
        .enable = {.polygon = true}};

    viewer::Viewer viewer = viewer::Viewer();
    viewer.draw(viewer_params, polygons, {});
}

TEST(Navigation, map_mesh) {
    const std::string MAP = "map_6";
    const std::string FILE_NAME = MAP + "_mesh";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png",
        .color_rgb = {},
        .thickness = {},
        .enable = {
            .polygon = true,
            .mesh = true,
        }};

    mesh::MeshParams mesh_params{.dist = 1.4, .offset = 1.6, .filter = {}};
    mesh::MeshBuilder mesh_builder = mesh::MeshBuilder(mesh_params);
    mesh::MeshBuild mesh_build = mesh_builder.build(polygons);

    viewer::Viewer viewer = viewer::Viewer();
    viewer.draw(viewer_params, polygons, mesh_build);
}

TEST(Navigation, map_mesh_skeleton_level_lines) {
    const std::string MAP = "map_6";
    const std::string FILE_NAME = MAP + "_mesh_skeleton_level_lines";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png",
        .color_rgb = {},
        .thickness = {},
        .enable = {
            .polygon = true,
            .skeleton = true,
            .level_lines = true,
            .mesh = true,
        }};

    mesh::MeshParams mesh_params{.dist = 1.4, .offset = 1.6, .filter = {}};
    mesh::MeshBuilder mesh_builder = mesh::MeshBuilder(mesh_params);
    mesh::MeshBuild mesh_build = mesh_builder.build(polygons);

    viewer::Viewer viewer = viewer::Viewer();
    viewer.draw(viewer_params, polygons, mesh_build);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}