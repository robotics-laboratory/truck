#include <gtest/gtest.h>

#include "map/map.h"
#include "geom/bezier.h"

#include "motion_planner/motion_planner.h"
#include "motion_planner/viewer.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace truck;
using namespace truck::motion_planner;

const std::string kMapPkgPath = ament_index_cpp::get_package_share_directory("map");

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

    const Reference reference = geom::bezier3({10, 10}, {11, 11}, {12, 11}, {13, 10}, size_t(60));

    const viewer::ViewerConfig viewer_cfg{"test/data/hull.png"};

    const hull::GraphParams graph_params{
        .hull_radius = 3.5, .milestone_spacing = 2, .node_spacing = .40};

    const GraphBuilder builder{graph_params};
    const hull::GraphBuild graph_build = builder.buildGraph(reference);

    std::cerr << "milestones.size(): " << graph_build.milestones.size() << "\n";

    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    viewer::Viewer viewer = viewer::Viewer(viewer_cfg, polygon);
    viewer.addPolygon(polygon);
    viewer.addHull(graph_build);
    viewer.draw();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
