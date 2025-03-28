#include <gtest/gtest.h>

#include "map/map.h"
#include "geom/bezier.h"

#include "motion_planner/graph_builder.h"
#include "motion_planner/search.h"
#include "motion_planner/viewer.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace truck;
using namespace truck::motion_planner;

const std::string kMapPkgPath = ament_index_cpp::get_package_share_directory("map");
const Reference reference = geom::toPoses(geom::compose_bezier3(
    {
        {10, 10},
        {11, 11},
        {12, 11},
        {13, 10},
        {14, 9},
        {15, 9},
        {16, 15},
        {17, 21},
        {16, 30},
        {21, 30},
        {26, 30},
        {30, 30},
        {23, 25},
        {20, 20},
        {22, 21},
        {22, 20},
    },
    size_t(60)));

TEST(MotionPlanner, poly) {
    const std::string file_path = kMapPkgPath + "/data/map_6.geojson";

    const geom::ComplexPolygons polygons = map::Map::fromGeoJson(file_path).polygons();

    const viewer::ViewerConfig viewer_cfg{"test/data/poly.png"};

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_cfg, polygon);
    viewer.addPolygon(polygon);
    viewer.draw();
}

TEST(MotionPlanner, hull) {
    const std::string file_path = kMapPkgPath + "/data/map_6.geojson";

    const geom::ComplexPolygons polygons = map::Map::fromGeoJson(file_path).polygons();

    const viewer::ViewerConfig viewer_cfg{"test/data/hull.png"};

    const hull::GraphParams graph_params{
        .hull_radius = 3.5,
        .milestone_spacing = 2.0,
        .node_spacing = .40,
        .raycast_increment = .1,
        .max_edge_slope = 2.0,
        .safezone_radius = .5,
    };

    const GraphBuilder builder{graph_params};
    const auto [graph, context] = builder.buildGraph(reference, polygons[0]);

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_cfg, polygon);
    viewer.addPolygon(polygon);
    viewer.addHull(reference, context);
    viewer.draw();
}

TEST(MotionPlanner, graph) {
    const std::string file_path = kMapPkgPath + "/data/map_6.geojson";

    const geom::ComplexPolygons polygons = map::Map::fromGeoJson(file_path).polygons();
    const viewer::ViewerConfig viewer_cfg{"test/data/graph.png"};

    const hull::GraphParams graph_params{
        .hull_radius = 3.5,
        .milestone_spacing = 2.0,
        .node_spacing = .40,
        .raycast_increment = .01,
        .max_edge_slope = 2.0,
        .safezone_radius = .5,
    };

    const GraphBuilder builder{graph_params};
    const auto [graph, context] = builder.buildGraph(reference, polygons[0]);

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_cfg, polygon);

    viewer.addPolygon(polygon);
    viewer.addHull(reference, context);
    viewer.addGraph(graph);
    viewer.draw();
}

TEST(MotionPlanner, trajectory) {
    const std::string file_path = kMapPkgPath + "/data/map_6.geojson";

    const geom::ComplexPolygons polygons = map::Map::fromGeoJson(file_path).polygons();
    const viewer::ViewerConfig viewer_cfg{"test/data/trajectory.png"};

    const hull::GraphParams graph_params{
        .hull_radius = 3.5,
        .milestone_spacing = 2.0,
        .node_spacing = .40,
        .raycast_increment = .01,
        .max_edge_slope = 2.0,
        .safezone_radius = .5,
    };

    const GraphBuilder builder{graph_params};
    const auto [graph, context] = builder.buildGraph(reference, polygons[0]);

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_cfg, polygon);
    const auto node_occupancy = std::vector<bool>(graph.nodes.size(), false);

    NodeId from_id = context.milestone_nodes.front().at(3);
    std::set<NodeId> to_ids = {
        context.milestone_nodes.back().cbegin(),
        context.milestone_nodes.back().cend(),
    };

    const search::Path path =
        search::findShortestPath(graph, node_occupancy, from_id, to_ids).value();
    const auto trajectory = search::fitSpline(graph.nodes, path);

    viewer.addPolygon(polygon);
    viewer.addHull(reference, context);
    viewer.addGraph(graph);
    viewer.addMotion(path, trajectory, graph);
    viewer.draw();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
