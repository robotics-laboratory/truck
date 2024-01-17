#include <gtest/gtest.h>

#include "map/map_builder.h"
#include "nav_mesh/nav_mesh_builder.h"
#include "nav_mesh_viewer/nav_mesh_viewer.h"

using namespace truck::geom;
using namespace truck::map;
using namespace truck::nav_mesh;
using namespace truck::nav_mesh_viewer;

TEST(NavMeshViewer, draw_map_2) {
    NavMeshParams nav_mesh_params {
        .dist = 3.0,
        .offset = 3.4,
        .filter = NavMeshParams::Filter {
            .grid = false
        }
    };
    
    NavMeshViewerParams nav_mesh_viewer_params {
        .res = 50,
        .path = "/truck/packages/nav_mesh_viewer/data/map_2.png",

        .color_rgb = NavMeshViewerParams::ColorRGB {
            .background = { 0, 0, 0 },
            .outer_polygon = { 252, 252, 252 },
            .inner_polygon = { 227, 227, 227 },
            .level_lines = { 255, 0, 0 },
            .skeleton = { 0, 200, 0 },
            .mesh = { 0, 0, 255 }
        },

        .thickness = NavMeshViewerParams::Thickness {
            .level_lines = 1,
            .skeleton = 2,
            .mesh = 5
        },

        .enable = NavMeshViewerParams::Enable {
            .polygon = true,
            .skeleton = true,
            .level_lines = true,
            .mesh = true,
        }
    };

    ComplexPolygons polygons = Map::fromGeoJson("/truck/packages/map/data/map_2.geojson").polygons();

    NavMeshBuilder nav_mesh_builder = NavMeshBuilder(nav_mesh_params);
    NavMeshBuild nav_mesh_build = nav_mesh_builder.build(polygons);
    
    NavMeshViewer nav_mesh_viewer = NavMeshViewer();
    nav_mesh_viewer.draw(nav_mesh_viewer_params, polygons, nav_mesh_build);
}

TEST(NavMeshViewer, draw_map_5) {
    NavMeshParams nav_mesh_params {
        .dist = 2.0,
        .offset = 2.8,
        .filter = NavMeshParams::Filter {
            .grid = false
        }
    };
    
    NavMeshViewerParams nav_mesh_viewer_params {
        .res = 50,
        .path = "/truck/packages/nav_mesh_viewer/data/map_5.png",

        .color_rgb = NavMeshViewerParams::ColorRGB {
            .background = { 0, 0, 0 },
            .outer_polygon = { 252, 252, 252 },
            .inner_polygon = { 227, 227, 227 },
            .level_lines = { 255, 0, 0 },
            .skeleton = { 0, 200, 0 },
            .mesh = { 0, 0, 255 }
        },

        .thickness = NavMeshViewerParams::Thickness {
            .level_lines = 1,
            .skeleton = 2,
            .mesh = 5
        },

        .enable = NavMeshViewerParams::Enable {
            .polygon = true,
            .skeleton = true,
            .level_lines = true,
            .mesh = true,
        }
    };

    ComplexPolygons polygons = Map::fromGeoJson("/truck/packages/map/data/map_5.geojson").polygons();

    NavMeshBuilder nav_mesh_builder = NavMeshBuilder(nav_mesh_params);
    NavMeshBuild nav_mesh_build = nav_mesh_builder.build(polygons);
    
    NavMeshViewer nav_mesh_viewer = NavMeshViewer();
    nav_mesh_viewer.draw(nav_mesh_viewer_params, polygons, nav_mesh_build);
}

TEST(NavMeshViewer, draw_map_6) {
    NavMeshParams nav_mesh_params {
        .dist = 1.0,
        .offset = 1.6,
        .filter = NavMeshParams::Filter {
            .grid = false
        }
    };
    
    NavMeshViewerParams nav_mesh_viewer_params {
        .res = 50,
        .path = "/truck/packages/nav_mesh_viewer/data/map_6.png",

        .color_rgb = NavMeshViewerParams::ColorRGB {
            .background = { 0, 0, 0 },
            .outer_polygon = { 252, 252, 252 },
            .inner_polygon = { 227, 227, 227 },
            .level_lines = { 255, 0, 0 },
            .skeleton = { 0, 200, 0 },
            .mesh = { 0, 0, 255 }
        },

        .thickness = NavMeshViewerParams::Thickness {
            .level_lines = 1,
            .skeleton = 2,
            .mesh = 5
        },

        .enable = NavMeshViewerParams::Enable {
            .polygon = true,
            .skeleton = true,
            .level_lines = true,
            .mesh = true,
        }
    };

    ComplexPolygons polygons = Map::fromGeoJson("/truck/packages/map/data/map_6.geojson").polygons();

    NavMeshBuilder nav_mesh_builder = NavMeshBuilder(nav_mesh_params);
    NavMeshBuild nav_mesh_build = nav_mesh_builder.build(polygons);
    
    NavMeshViewer nav_mesh_viewer = NavMeshViewer();
    nav_mesh_viewer.draw(nav_mesh_viewer_params, polygons, nav_mesh_build);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}