#include <gtest/gtest.h>

#include "map/map.h"
#include "navigation/viewer.h"
#include "navigation/mesh_builder.h"
#include "navigation/graph_builder.h"
#include "navigation/search.h"

using namespace truck;
using namespace truck::navigation;

const std::string ROOT = "/truck/packages";

TEST(Navigation, map) {
    const std::string MAP = "map_6";
    const std::string FILE_NAME = MAP;

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png",
        .color_rgb = {},
        .thickness = {}};

    viewer::Viewer viewer = viewer::Viewer();
    viewer.draw(viewer_params, polygons);
}

TEST(Navigation, map_mesh) {
    const std::string MAP = "map_6";
    const std::string FILE_NAME = MAP + "_mesh";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png",
        .color_rgb = {},
        .thickness = {}};

    mesh::MeshParams mesh_params{.dist = 1.4, .offset = 1.6, .filter = {}};
    mesh::MeshBuilder mesh_builder = mesh::MeshBuilder(mesh_params);
    mesh::MeshBuild mesh_build = mesh_builder.build(polygons);

    viewer::Viewer viewer = viewer::Viewer();
    viewer.draw(
        viewer_params,
        polygons,
        mesh_build.mesh,
        mesh_build.skeleton,
        mesh_build.level_lines);
}

TEST(Navigation, map_graph) {
    const std::string MAP = "map_6";
    const std::string FILE_NAME = MAP + "_graph";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png",
        .color_rgb = {},
        .thickness = {}};

    mesh::MeshParams mesh_params{.dist = 1.4, .offset = 1.6, .filter = {}};
    mesh::MeshBuilder mesh_builder = mesh::MeshBuilder(mesh_params);
    mesh::MeshBuild mesh_build = mesh_builder.build(polygons);

    graph::GraphParams graph_params{
        .mode = graph::GraphParams::Mode::searchRadius, .search_radius = 3.6};
    graph::GraphBuilder graph_builder =
        graph::GraphBuilder(graph_params)
            .setNodes(mesh_build.mesh)
            .setComplexPolygons(polygons)
            .build();
    
    const auto& edges = graph_builder.getEdges();

    size_t from_node = 108;
    size_t to_node = 354;

    const auto& route = 
        search::toSegments(
            search::Dijkstra(graph_builder.getWeights(), from_node, to_node),
            mesh_build.mesh);

    viewer::Viewer viewer = viewer::Viewer();
    viewer.draw(
        viewer_params,
        polygons,
        mesh_build.mesh,
        std::nullopt,
        std::nullopt,
        edges,
        route);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}