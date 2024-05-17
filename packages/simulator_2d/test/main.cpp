#include "simulator_2d/simulator_engine.h"
#include "truck_msgs/msg/simulation_state.hpp"
#include "geom/msg.h"
#include "geom/test/equal_assert.h"

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <iomanip>
#include <vector>
#include <optional>
#include <string>
#include <limits>

using namespace truck::simulator;

const std::string MODEL_CONFIG_PATH = "/truck/packages/model/config/model.yaml";
const std::string TEST_OUTPUT_PATH = "/truck/packages/simulator_2d/test/data/test_output_";
rosbag2_storage::TopicMetadata TEST_OUTPUT_TOPIC{
    "/simulator/state", "truck_msgs/msg/SimulationState", "cdr", ""};
constexpr double EPS = 1e-9;

struct ScriptStep {
    int iterations;
    double velocity;
    double curvature;
    std::optional<double> acceleration;
};

using Script = std::vector<ScriptStep>;

truck_msgs::msg::SimulationState convertStateToMsg(const TruckState& truck_state) {
    truck_msgs::msg::SimulationState state_msg;
    state_msg.header.frame_id = "base";
    state_msg.header.stamp = truck_state.time();

    state_msg.speed = truck_state.baseTwist().velocity;
    state_msg.steering = truck_state.currentSteering().middle.radians();

    const auto pose = truck_state.odomBasePose();
    state_msg.pose = truck::geom::msg::toPose(pose);

    const auto angular_velocity = truck_state.gyroAngularVelocity();
    state_msg.gyro_angular_velocity.x = angular_velocity.x;
    state_msg.gyro_angular_velocity.y = angular_velocity.y;
    state_msg.gyro_angular_velocity.z = angular_velocity.z;

    const auto acceleration = truck_state.accelLinearAcceleration();
    state_msg.accel_linear_acceleration.x = acceleration.x;
    state_msg.accel_linear_acceleration.y = acceleration.y;
    state_msg.accel_linear_acceleration.z = acceleration.z;

    return state_msg;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> serializeMsg(
    const truck_msgs::msg::SimulationState& state_msg) {
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<truck_msgs::msg::SimulationState> serialization;
    serialization.serialize_message(&state_msg, &serialized_msg);

    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->time_stamp = RCUTILS_S_TO_NS(static_cast<int64_t>(state_msg.header.stamp.sec))
                          + state_msg.header.stamp.nanosec;
    bag_msg->topic_name = TEST_OUTPUT_TOPIC.name;
    bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        new rcutils_uint8_array_t, [](rcutils_uint8_array_t* msg) {
            auto fini_return = rcutils_uint8_array_fini(msg);
            delete msg;
            if (fini_return != RCUTILS_RET_OK) {
                std::cerr << "Failed to destroy serialized message "
                          << rcutils_get_error_string().str << '\n';
                FAIL();
            }
        });
    *bag_msg->serialized_data = serialized_msg.release_rcl_serialized_message();

    return bag_msg;
}

void recordTruckState(
    const TruckState& truck_state, rosbag2_cpp::writers::SequentialWriter& writer) {
    const auto state_msg = convertStateToMsg(truck_state);
    auto bag_msg = serializeMsg(state_msg);
    writer.write(bag_msg);
}

TruckState processTestCase(const Script& script, double update_period, const std::string& suffix) {
    auto model = std::make_unique<truck::model::Model>(MODEL_CONFIG_PATH);
    auto engine = SimulatorEngine(std::move(model));

    auto storage_options = rosbag2_storage::StorageOptions();
    storage_options.uri = TEST_OUTPUT_PATH + suffix;
    storage_options.storage_id = "mcap";

    auto converter_options = rosbag2_cpp::ConverterOptions();
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::writers::SequentialWriter writer;
    writer.open(storage_options, converter_options);
    writer.create_topic(TEST_OUTPUT_TOPIC);
    recordTruckState(engine.getTruckState(), writer);

    for (const auto step : script) {
        if (step.acceleration) {
            engine.setBaseControl(step.velocity, *step.acceleration, step.curvature);
        } else {
            engine.setBaseControl(step.velocity, step.curvature);
        }

        for (int j = 0; j < step.iterations; ++j) {
            engine.advance(update_period);
            recordTruckState(engine.getTruckState(), writer);
        }
    }

    writer.close();
    return engine.getTruckState();
}

TEST(SimulatorEngine, straight) {
    // Arrange.
    Script script{{500, 0.8, 0, std::nullopt}};
    const double update_period = 0.01;

    // Act.
    const auto state = processTestCase(script, update_period, "straight");

    // Assert.
    ASSERT_GEOM_EQUAL(state.baseTwist().velocity, 0.8, EPS);
    ASSERT_GEOM_EQUAL(state.currentSteering().middle.radians(), 0., EPS);
    const auto pose = state.odomBasePose();
    ASSERT_GEOM_EQUAL(pose.pos.x, 3.68, EPS);
    ASSERT_GEOM_EQUAL(pose.pos.y, 0., EPS);
    ASSERT_GEOM_EQUAL(pose.dir.angle().degrees(), 0., EPS);
    const auto angular_velocity = state.gyroAngularVelocity();
    ASSERT_GEOM_EQUAL(angular_velocity.x, 0., EPS);
    ASSERT_GEOM_EQUAL(angular_velocity.y, 0., EPS);
    ASSERT_GEOM_EQUAL(angular_velocity.z, 0., EPS);
    const auto acceleration = state.accelLinearAcceleration();
    ASSERT_GEOM_EQUAL(acceleration.x, std::numeric_limits<double>::quiet_NaN(), EPS);
    ASSERT_GEOM_EQUAL(acceleration.y, std::numeric_limits<double>::quiet_NaN(), EPS);
    ASSERT_GEOM_EQUAL(acceleration.z, std::numeric_limits<double>::quiet_NaN(), EPS);
}

TEST(SimulatorEngine, straightBackward) {
    // Arrange.
    Script script{
        {100, 0.8, 0., std::nullopt},
        {200, -0.8, 0., std::nullopt},
        {300, 0.8, 0., std::nullopt},
        {200, 0., 0., std::nullopt}};
    const double update_period = 0.01;

    // Act.
    const auto state = processTestCase(script, update_period, "straightBackward");

    // Assert.
    ASSERT_GEOM_EQUAL(state.baseTwist().velocity, 0., EPS);
    ASSERT_GEOM_EQUAL(state.currentSteering().middle.radians(), 0., EPS);
    const auto pose = state.odomBasePose();
    ASSERT_GEOM_EQUAL(pose.pos.x, 1.9425, EPS);
    ASSERT_GEOM_EQUAL(pose.pos.y, 0., EPS);
    ASSERT_GEOM_EQUAL(pose.dir.angle().degrees(), 0., EPS);
    const auto angular_velocity = state.gyroAngularVelocity();
    ASSERT_GEOM_EQUAL(angular_velocity.x, 0., EPS);
    ASSERT_GEOM_EQUAL(angular_velocity.y, 0., EPS);
    ASSERT_GEOM_EQUAL(angular_velocity.z, 0., EPS);
    const auto acceleration = state.accelLinearAcceleration();
    ASSERT_GEOM_EQUAL(acceleration.x, std::numeric_limits<double>::quiet_NaN(), EPS);
    ASSERT_GEOM_EQUAL(acceleration.y, std::numeric_limits<double>::quiet_NaN(), EPS);
    ASSERT_GEOM_EQUAL(acceleration.z, std::numeric_limits<double>::quiet_NaN(), EPS);
}

TEST(SimulatorEngine, circle) {
    // Arrange.
    Script script{{500, 0.8, 2.0, std::nullopt}};
    const double update_period = 0.01;

    // Act.
    const auto state = processTestCase(script, update_period, "circle");

    // Assert.
    ASSERT_GEOM_EQUAL(state.baseTwist().velocity, 0.8, EPS);
    ASSERT_GEOM_EQUAL(state.currentSteering().middle.radians(), 0.602232435, EPS);
    const auto pose = state.odomBasePose();
    ASSERT_GEOM_EQUAL(pose.pos.x, -0.2314822449, EPS);
    ASSERT_GEOM_EQUAL(pose.pos.y, -0.0320693938, EPS);
    ASSERT_GEOM_EQUAL(pose.dir.angle().degrees(), 336.909262889, EPS);
    const auto angular_velocity = state.gyroAngularVelocity();
    ASSERT_GEOM_EQUAL(angular_velocity.x, 0., EPS);
    ASSERT_GEOM_EQUAL(angular_velocity.y, -0.703276735, EPS);
    ASSERT_GEOM_EQUAL(angular_velocity.z, 0., EPS);
    const auto acceleration = state.accelLinearAcceleration();
    ASSERT_GEOM_EQUAL(acceleration.x, -1.08827613795, EPS);
    ASSERT_GEOM_EQUAL(acceleration.y, -9.81, EPS);
    ASSERT_GEOM_EQUAL(acceleration.z, -0.0346212283, EPS);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
