#include <gtest/gtest.h>

#include "map/map.h"
#include "navigation/graph_builder.h"
#include "navigation/mesh_builder.h"
#include "navigation/search.h"
#include "navigation/viewer.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace truck;
using namespace truck::navigation;

const std::string kMapPkgPath = ament_index_cpp::get_package_share_directory("map");

TEST(Navigation, poly) {
    const std::string file_path = kMapPkgPath + "/data/map_6.geojson";

    const geom::ComplexPolygons polygons = map::Map::fromGeoJson(file_path).polygons();

    const viewer::ViewerParams viewer_params{
        .path = "test/data/poly.png", .color_rgb = {}, .thickness = {}};

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_params, polygon);
    viewer.addPolygon(polygon);
    viewer.draw();
}

TEST(Navigation, mesh) {
    const std::string file_path = kMapPkgPath + "/data/map_6.geojson";

    const geom::ComplexPolygons polygons = map::Map::fromGeoJson(file_path).polygons();

    const viewer::ViewerParams viewer_params{
        .path = "test/data/mesh.png", .color_rgb = {}, .thickness = {}};

    const mesh::MeshParams mesh_params{.dist = 1.4, .offset = 1.6, .filter = {}};
    const mesh::MeshBuilder mesh_builder = mesh::MeshBuilder(mesh_params);
    const mesh::MeshBuild mesh_build = mesh_builder.build(polygons);

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_params, polygon);
    viewer.addPolygon(polygon);
    viewer.addMeshBuild(mesh_build);
    viewer.draw();
}

TEST(Navigation, graph) {
    const std::string file_path = kMapPkgPath + "/data/map_6.geojson";

    const geom::ComplexPolygons polygons = map::Map::fromGeoJson(file_path).polygons();

    const viewer::ViewerParams viewer_params{
        .path = "test/data/graph.png", .color_rgb = {}, .thickness = {}};

    const mesh::MeshParams mesh_params{.dist = 1.4, .offset = 1.6, .filter = {}};
    const mesh::MeshBuilder mesh_builder = mesh::MeshBuilder(mesh_params);
    const mesh::MeshBuild mesh_build = mesh_builder.build(polygons);

    const graph::GraphParams graph_params{
        .mode = graph::GraphParams::Mode::searchRadius, .search_radius = 3.6};
    const graph::GraphBuilder graph_builder = graph::GraphBuilder(graph_params);
    const graph::Graph graph = graph_builder.build(mesh_build.mesh, polygons);

    const geom::Polyline path = search::toPolyline(graph, search::findShortestPath(graph, 28, 306));

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
