#include <gtest/gtest.h>

#include "map/map.h"
#include "geom/bezier.h"
#include "model/model.h"

#include "motion_planner/graph_builder.h"
#include "motion_planner/search.h"
#include "motion_planner/viewer.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

using namespace truck;
using namespace truck::motion_planner;

const std::string kMapPkgPath = ament_index_cpp::get_package_share_directory("map");
const std::string kModelPkgPath = ament_index_cpp::get_package_share_directory("model");

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
    std::size_t(60)));

void set_obstacles(
    std::vector<bool>& occupancy_grid, std::initializer_list<NodeId> occupied_nodes) {
    for (const NodeId& node_id : occupied_nodes) {
        occupancy_grid.at(node_id) = true;
    }
}

void dump_milestone_nodes(
    const std::vector<NodeIds>& milestone_nodes, const std::string& file_path) {
    std::ofstream file{file_path};
    for (std::size_t i = 0; i < milestone_nodes.size(); ++i) {
        file << i << ": ";
        for (const NodeId id : milestone_nodes[i]) {
            file << id << " ";
        }
        file << "\n";
    }
}

void dump_path_trace(const search::Path& path, const std::string& file_path) {
    std::ofstream file{file_path};
    for (std::size_t i = 0; i < path.trace.size(); ++i) {
        file << i << ": " << path.trace[i] << "\n";
    }
}

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
    auto occupancy_grid = std::vector<bool>(graph.nodes.size(), false);

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_cfg, polygon);

    viewer.addPolygon(polygon);
    viewer.addHull(reference, context);
    viewer.addGraph(graph, occupancy_grid);
    viewer.draw();
}

TEST(MotionPlanner, trajectory) {
    const std::string map_file_path = kMapPkgPath + "/data/map_6.geojson";
    const std::string model_file_path = kModelPkgPath + "/config/model.yaml";

    const geom::ComplexPolygons polygons = map::Map::fromGeoJson(map_file_path).polygons();
    const model::Model model_ = model::Model(model_file_path);

    const viewer::ViewerConfig viewer_cfg{"test/data/trajectory.png"};

    const auto graph_params = hull::GraphParams{
        .hull_radius = model_.shape().width * 5,
        .milestone_spacing = model_.shape().length * 1.5,
        .node_spacing = model_.shape().radius(),
        .raycast_increment = .1,
        .max_edge_slope = 2.0,
        .safezone_radius = model_.shape().radius(),
    };

    const GraphBuilder builder{graph_params};
    const auto [graph, context] = builder.buildGraph(reference, polygons[0]);

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_cfg, polygon);
    auto occupancy_grid = std::vector<bool>(graph.nodes.size(), false);

    set_obstacles(occupancy_grid, {79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90});
    set_obstacles(
        occupancy_grid, {314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326});

    set_obstacles(
        occupancy_grid, {394, 395, 396, 397, 398, 399, 400, 401, 402, 403, 404, 405, 406});

    set_obstacles(
        occupancy_grid, {445, 446, 447, 448, 449, 450, 451, 452, 453, 454, 455, 456, 457});

    NodeId from_id = context.milestone_nodes.front().at(16);
    std::set<NodeId> to_ids = {
        context.milestone_nodes.back().cbegin(),
        context.milestone_nodes.back().cend(),
    };

    const search::Path path =
        search::findShortestPath(graph, occupancy_grid, from_id, to_ids).value();
    const auto trajectory = search::fitSpline(graph.nodes, path);

    viewer.addPolygon(polygon);
    viewer.addHull(reference, context);
    viewer.addGraph(graph, occupancy_grid);
    viewer.addMotion(path, trajectory, graph);
    viewer.draw();

    dump_milestone_nodes(context.milestone_nodes, "test/data/milestone_nodes.txt");
    dump_path_trace(path, "test/data/path_trace.txt");
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
