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
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png", .color_rgb = {}, .thickness = {}};

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_params, polygon);
    viewer.addPolygon(polygon);
    viewer.draw();
}

TEST(Navigation, map_mesh) {
    const std::string MAP = "map_6";
    const std::string FILE_NAME = MAP + "_mesh";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png", .color_rgb = {}, .thickness = {}};

    mesh::MeshParams mesh_params{.dist = 1.4, .offset = 1.6, .filter = {}};
    mesh::MeshBuilder mesh_builder = mesh::MeshBuilder(mesh_params);
    mesh::MeshBuild mesh_build = mesh_builder.build(polygons);

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_params, polygon);
    viewer.addPolygon(polygon);
    viewer.addMeshBuild(mesh_build);
    viewer.draw();
}

TEST(Navigation, map_graph) {
    const std::string MAP = "map_6";
    const std::string FILE_NAME = MAP + "_graph";

    geom::ComplexPolygons polygons =
        map::Map::fromGeoJson(ROOT + "/map/data/" + MAP + ".geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = ROOT + "/navigation/data/" + FILE_NAME + ".png", .color_rgb = {}, .thickness = {}};

    mesh::MeshParams mesh_params{.dist = 1.4, .offset = 1.6, .filter = {}};
    mesh::MeshBuilder mesh_builder = mesh::MeshBuilder(mesh_params);
    mesh::MeshBuild mesh_build = mesh_builder.build(polygons);

    graph::GraphParams graph_params{
        .mode = graph::GraphParams::Mode::searchRadius, .search_radius = 3.6};
    graph::GraphBuilder graph_builder = graph::GraphBuilder(graph_params);
    graph::Graph graph = graph_builder.build(mesh_build.mesh, polygons);

    geom::Polyline path = search::toPolyline(graph, search::findShortestPath(graph, 28, 306));

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_params, polygon);
    viewer.addPolygon(polygon);
    viewer.addPath(path);
    viewer.addGraph(graph);
    viewer.draw();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
