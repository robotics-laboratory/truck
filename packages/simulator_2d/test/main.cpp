#include <iomanip>
#include <vector>
#include <optional>

#include <gtest/gtest.h>

#include "simulator_2d/simulator_engine.h"

using namespace truck::simulator;

struct ScriptStep {
    int iterations;
    double velocity;
    double curvature;
    std::optional<double> acceleration;
};

using Script = std::vector<ScriptStep>;

void printTruckState(const TruckState& truck_state) {
    std::cerr << "{" 
        << std::fixed << std::setprecision(5)
        << "\"time\":" << truck_state.getTime().seconds()
        << ", \"x\":" << truck_state.getBaseOdomPose().pos.x
        << ", \"y\":" << truck_state.getBaseOdomPose().pos.y
        << ", \"velocity\":" << truck_state.getBaseOdomTwist().velocity
        << ", \"steering\":" << truck_state.getCurrentSteering().middle.radians()
        << "}\n";
}

void processTestCase(const Script& script, double update_period) {
    auto model = std::make_unique<truck::model::Model>(
        "/truck/packages/model/config/model.yaml");
    auto engine = SimulatorEngine(std::move(model));
    printTruckState(engine.getTruckState());
    for (const auto step : script) {
        if (step.acceleration) {
            engine.setBaseControl(step.velocity, *step.acceleration, step.curvature);
        } else {
            engine.setBaseControl(step.velocity, step.curvature);
        }
        
        for (int j = 0; j < step.iterations; ++j) {
            engine.advance(update_period);
            printTruckState(engine.getTruckState());
        }
    }
}

TEST(SimulatorEngine, straight) {
    Script script {{500, 10, 0, std::nullopt}};
    const double update_period = 0.01;
    
    processTestCase(script, update_period);
}

TEST(SimulatorEngine, straightBackward) {
    Script script {
        {100, 10, 0, std::nullopt}, 
        {200, -10, 0, std::nullopt}, 
        {300, 10, 0, std::nullopt}, 
        {200, 0, 0, std::nullopt}
    };
    const double update_period = 0.01;

    processTestCase(script, update_period);
}

TEST(SimulatorEngine, circle) {
    Script script {{500, 10, 10, std::nullopt}};
    const double update_period = 0.01;

    processTestCase(script, update_period);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
