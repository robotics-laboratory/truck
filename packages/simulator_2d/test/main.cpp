#include <iomanip>
#include <vector>

#include <gtest/gtest.h>

#include "simulator_2d/simulator_engine.h"

using namespace truck::simulator;

void processTestCase(const std::vector<std::pair<int, int>>& script, double update_period) {
    const auto model = truck::model::Model("test/model.yaml");
    auto engine = SimulatorEngine(model);
    for (const auto step : script) {
        engine.setBaseControl(step.first, 0);
        for (int j = 0; j < step.second; ++j) {
            engine.advance(update_period);
            const auto truck_state = *(engine.getBaseTruckState().get());
            std::cerr << std::fixed << std::setprecision(5)
                << truck_state.getTime().seconds() << ' '
                << truck_state.getPose().pos.x << ' '
                << truck_state.getTwist().velocity << '\n';
        }
    }
}

TEST(SimulatorEngine, straight) {
    std::vector<std::pair<int, int>> script 
        {{10, 500}};
    const double update_period = 0.01;
    
    processTestCase(script, update_period);
}

TEST(SimulatorEngine, straightBackward) {
    std::vector<std::pair<int, int>> script 
        {{10, 100}, {-10, 200}, {10, 300}, {0, 200}};
    const double update_period = 0.01;

    processTestCase(script, update_period);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
