#include "planner.hpp"

#include "float_comparison.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nlohmann/json.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>


using namespace planning_interfaces;

namespace {

using nlohmann::json;

inline void hash_combine(std::size_t&) {}

template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, Rest... rest) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    hash_combine(seed, rest...);
}

double mod_interval(double x, double modulo) {
    return std::fmod(std::fmod(x, modulo) + modulo, modulo);
}

struct ComparisonTolerances {
    static std::unique_ptr<ComparisonTolerances> instance;

    static double get_x() {
        return instance->x;
    }
    static double get_y() {
        return instance->y;
    }
    static double get_theta() {
        return instance->theta;
    }
    static double get_distance() {
        return instance->distance;
    }

    static void load_from_json(const json& tolerances) {
        instance = std::make_unique<ComparisonTolerances>();
        for (auto it = tolerances.begin(); it != tolerances.end(); ++it) {
            if (it.key() == "x") {
                instance->x = it.value();
            } else if (it.key() == "y") {
                instance->y = it.value();
            } else if (it.key() == "theta") {
                instance->theta = it.value();
            } else if (it.key() == "distance") {
                instance->distance = it.value();
            }
        }
    }

    static void load_default() {
        instance = std::make_unique<ComparisonTolerances>();
        instance->x = 0.00001;
        instance->y = 0.00001;
        instance->theta = 0.01;
        instance->distance = 0.00001;
    }

private:
    double x;
    double y;
    double theta;
    double distance;
};

std::unique_ptr<ComparisonTolerances> ComparisonTolerances::instance = nullptr;

struct State {
    double x;
    double y;
    double theta;

    double distance;

    static State from_json(const json& state) {
        return State{
            state["x"],
            state["y"],
            mod_interval(state["theta"], 2 * M_PI),
            0.0,
        };
    }

    static State from_point(msg::Point::SharedPtr point) {
        return State{
            point->x,
            point->y,
            mod_interval(point->theta, 2 * M_PI),
            0.0,
        };
    }

    json to_json() const {
        json state;
        state["x"] = x;
        state["y"] = y;
        state["theta"] = theta;
        return state;
    }

    geometry_msgs::msg::PoseStamped to_pose_stamped() const {
        geometry_msgs::msg::PoseStamped res;
        res.pose.position.x = x;
        res.pose.position.y = y;

        tf2::Quaternion quart;
        quart.setRPY(0.0, 0.0, theta);
        res.pose.orientation = tf2::toMsg(quart);

        return res;
    }

    bool operator==(State o) const {
        return very_close_equals(x, o.x, ComparisonTolerances::get_x()) &&
            very_close_equals(y, o.y, ComparisonTolerances::get_y()) &&
            very_close_equals(theta, o.theta, ComparisonTolerances::get_theta());
    }
    bool operator!=(State o) const {
        return !(*this == o);
    }
};

}

template <>
struct std::hash<State> {
    std::size_t operator()(const State& s) const {
        size_t res = 0;
        hash_combine(
            res,
            static_cast<int64_t>(s.x * 1000),  // FIXME: use better hash for floating point values
            static_cast<int64_t>(s.y * 1000),
            static_cast<int64_t>(s.theta * 360)
        );
        return res;
    }
};

namespace {

struct StateComparator {
    bool operator()(State a, State b) const {
        // todo: review this (is this a valid comparator?)
        if (a == b) {
            return false;
        }

        // todo: this looks ugly, maybe rewrite it with macros
        if (very_close_less(a.distance, b.distance, ComparisonTolerances::get_distance())) {
            return true;
        } else if (very_close_equals(a.distance, b.distance, ComparisonTolerances::get_distance())) {
            if (very_close_less(a.x, b.x, ComparisonTolerances::get_x())) {
                return true;
            } else if (very_close_equals(a.x, b.x, ComparisonTolerances::get_x())) {
                if (very_close_less(a.y, b.y, ComparisonTolerances::get_y())) {
                    return true;
                } else if (very_close_equals(a.y, b.y, ComparisonTolerances::get_y())) {
                    return very_close_less(a.theta, b.theta, ComparisonTolerances::get_theta());
                }
            }
        }
        return false;
    }
};

struct MotionPrimitive {
    double dx;
    double dy;
    double dtheta;

    double weight;

    static MotionPrimitive from_json(const json& primitive) {
        return MotionPrimitive{
            primitive["dx"],
            primitive["dy"],
            mod_interval(primitive["dtheta"], 2 * M_PI),
            primitive["weight"],
        };
    }

    State apply(State state) const {
        return State{
            state.x + std::cos(state.theta) * dx - std::sin(state.theta) * dy,
            state.y + std::sin(state.theta) * dx + std::cos(state.theta) * dy,
            mod_interval(state.theta + dtheta, 2 * M_PI),
            state.distance + weight,
        };
    }
};

using MotionPrimitives = std::vector<MotionPrimitive>;

struct CollisionTester {
    CollisionTester(msg::Scene::SharedPtr scene) : scene(scene) {
    }

    bool test(State state) {
        if (std::abs(state.x) > 20 || std::abs(state.y) > 20) {
            return true;
        }

        int grid_x = static_cast<int>(state.x - scene->occupancy_grid.info.origin.position.x - 0.5);
        int grid_y = static_cast<int>(state.y - scene->occupancy_grid.info.origin.position.y - 0.5);
        return scene->occupancy_grid.data[grid_y * scene->occupancy_grid.info.width + grid_x] > 0;
    }

private:
    msg::Scene::SharedPtr scene;
};

struct StateSpace {
    StateSpace(CollisionTester tester, MotionPrimitives primitives)
        : tester{tester}
        , primitives{primitives}
        , open_set{StateComparator{}} {
    }

    State get_optimal() {
        return *open_set.begin();
    }

    void expand_optimal() {
        State optimal = get_optimal();
        open_set.erase(open_set.begin());
        open_set_checker.erase(optimal);
        closed_set.insert(optimal);

        for (MotionPrimitive primitive : primitives) {
            State next_state = primitive.apply(optimal);
            if (closed_set.find(next_state) != closed_set.end() ||
                open_set_checker.find(next_state) != closed_set.end() ||
                tester.test(next_state)) {
                continue;
            }
            open_set.insert(next_state);
            open_set_checker.insert(next_state);
            origin[next_state] = optimal;
        }
    }

    bool empty() const {
        return open_set.empty();
    }

    void insert(State state) {
        open_set.insert(state);
        open_set_checker.insert(state);
    }

    CollisionTester tester;
    MotionPrimitives primitives;
    std::set<State, StateComparator> open_set;
    std::unordered_set<State> open_set_checker;
    std::unordered_set<State> closed_set;

    std::unordered_map<State, State> origin;
};

}

namespace planning_node {

struct Planner {
    Planner(
        std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue,
        std::shared_ptr<SingleSlotQueue<msg::Point::SharedPtr>> target_queue,
        rclcpp::Publisher<msg::Path>::SharedPtr path_publisher,
        std::string config_path,
        rclcpp::Logger logger
    ) : scene_queue(scene_queue)
      , target_queue(target_queue)
      , path_publisher(path_publisher)
      , logger(logger) {
        std::ifstream config_stream(config_path);
        json config = json::parse(config_stream);

        ComparisonTolerances::load_from_json(config["tolerances"]);
        for (auto json_primitive : config["primitives"]) {
            primitives.push_back(MotionPrimitive::from_json(json_primitive));
        }

        initial = State::from_json(config["initial"]);
    }

    nav_msgs::msg::Path plan(CollisionTester tester, MotionPrimitives primitives, State initial, State target) {
        StateSpace state_space(tester, primitives);
        state_space.insert(initial);

        bool found = false;
        while (!state_space.empty()) {
            State optimal = state_space.get_optimal();
            if (optimal == target) {
                found = true;
                break;
            }

            state_space.expand_optimal();
        }

        nav_msgs::msg::Path result;
        if (found) {
            State current = target;
            while (current != initial) {
                result.poses.push_back(current.to_pose_stamped());
                current = state_space.origin[current];
            }

            result.poses.push_back(current.to_pose_stamped());
            std::reverse(result.poses.begin(), result.poses.end());
        } else {
            RCLCPP_INFO(logger, "No path found");
        }
        return result;
    }

    void start() {
        std::optional<msg::Scene::SharedPtr> scene;
        while ((scene = scene_queue->take()).has_value()) {
            CollisionTester tester{scene.value()};
            std::optional<msg::Point::SharedPtr> target_point = target_queue->peek();
            if (!target_point.has_value()) {
                RCLCPP_DEBUG(logger, "No target in the topic, skipping planning");
                continue;
            }
            State target = State::from_point(target_point.value());

            msg::Path path;

            path.path = plan(tester, primitives, initial, target);
            path.created_at = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count();

            path_publisher->publish(path);
        }
    }

private:
    std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue;
    std::shared_ptr<SingleSlotQueue<msg::Point::SharedPtr>> target_queue;
    rclcpp::Publisher<msg::Path>::SharedPtr path_publisher;
    rclcpp::Logger logger;
    MotionPrimitives primitives;
    State initial;
};

std::thread start_planner(
    std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue,
    std::shared_ptr<SingleSlotQueue<msg::Point::SharedPtr>> target_queue,
    rclcpp::Publisher<msg::Path>::SharedPtr path_publisher,
    std::string config_path,
    rclcpp::Logger logger
) {
    auto planner_func = [=]() {
        Planner planner{scene_queue, target_queue, path_publisher, config_path, logger};
        planner.start();
    };
    return std::thread{planner_func};

}

}
