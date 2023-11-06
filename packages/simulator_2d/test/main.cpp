#include <gtest/gtest.h>

#include "simulator_2d/simulator_engine.h"

#include <rclcpp/rclcpp.hpp>

using namespace truck::simulator;

TEST(SimulatorEngine, advance) {
    const auto model = truck::model::Model(
        "$(find-pkg-share model)/config/model.yaml");
    auto engine_ = SimulatorEngine(model);
    
    const int length = 5;
    int script[length][2] 
        = {{10, 100}, {-10, 200}, {10, 300}, {0, 200}};
    const double update_period = 0.01;
    
    for (int i = 0; i < length; ++i) {
        engine_.setBaseControl(script[i][0], 0);
        for (int j = 0; j < script[i][1]; ++j) {
            engine_.advance(update_period);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_node"), 
                std::to_string(engine_.getTime().seconds()) + " "
                + std::to_string(engine_.getBasePose().pos.x) + " "
                + std::to_string(engine_.getBaseTwist().velocity));
            ASSERT_TRUE(false)
                << engine_.getTime().seconds() << ' '
                << engine_.getBasePose().pos.x << ' '
                << engine_.getBaseTwist().velocity << '\n';
        }
    }
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
