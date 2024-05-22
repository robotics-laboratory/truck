#include <gtest/gtest.h>

#include "map/map.h"
#include "navigation/viewer.h"
#include "navigation/mesh_builder.h"
#include "navigation/graph_builder.h"
#include "navigation/search.h"

using namespace truck;
using namespace truck::navigation;

TEST(Navigation, poly) {
    geom::ComplexPolygons polygons = map::Map::fromGeoJson("map/data/map_6.geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = "navigation/test/data/poly.png", .color_rgb = {}, .thickness = {}};

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_params, polygon);
    viewer.addPolygon(polygon);
    viewer.draw();
}

TEST(Navigation, mesh) {
    geom::ComplexPolygons polygons = map::Map::fromGeoJson("map/data/map_6.geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = "navigation/test/data/mesh.png", .color_rgb = {}, .thickness = {}};

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

TEST(Navigation, graph) {
    geom::ComplexPolygons polygons = map::Map::fromGeoJson("map/data/map_6.geojson").polygons();

    viewer::ViewerParams viewer_params{
        .path = "navigation/test/data/graph.png", .color_rgb = {}, .thickness = {}};

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
