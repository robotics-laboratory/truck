#include <gtest/gtest.h>

#include "map/map.h"

#include "motion_planner/motion_planner.h"
#include "motion_planner/viewer.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace truck;
using namespace truck::motion_planner;

const std::string kMapPkgPath = ament_index_cpp::get_package_share_directory("map");

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
