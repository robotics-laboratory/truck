#include "planner.hpp"

#include "float_comparison.hpp"

#include <algorithm>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>


using namespace planning_interfaces;

namespace {

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

double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

struct ComparisonTolerances {
    static std::unique_ptr<ComparisonTolerances> instance;

    static double get(std::string key) {
        return instance->values[key];
    }

    // static void load_from_json(json tolerances) {
    //     instance = std::make_unique<ComparisonTolerances>();
    //     for (auto it = tolerances.begin(); it != tolerances.end(); ++it) {
    //         instance->values[it.key()] = it.value();
    //     }
    // }

    static void LoadDefault() {
        instance = std::make_unique<ComparisonTolerances>();
        instance->values["x"] = 0.00001;
        instance->values["y"] = 0.00001;
        instance->values["theta"] = 0.01;
        instance->values["distance"] = 0.00001;
    }

private:
    std::unordered_map<std::string, double> values;
};

std::unique_ptr<ComparisonTolerances> ComparisonTolerances::instance = nullptr;

struct State {
    double x;
    double y;
    double theta;

    double distance;

    // static State from_json(json state) {
    //     return State{
    //         .x = state["x"],
    //         .y = state["y"],
    //         .theta = mod_interval(state["theta"], 360.0),
    //         .distance = 0.0,
    //     };
    // }

    // json to_json() const {
    //     json state;
    //     state["x"] = x;
    //     state["y"] = y;
    //     state["theta"] = theta;
    //     return state;
    // }

    bool operator==(State o) const {
        return o.distance == 0.0;
        // return very_close_equals(x, o.x, ComparisonTolerances::get("x")) && 
        //     very_close_equals(y, o.y, ComparisonTolerances::get("y")) &&
        //     very_close_equals(theta, o.theta, ComparisonTolerances::get("theta"));
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
        hash_combine(res, s.x, s.y, s.theta);
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
        if (very_close_less(a.distance, b.distance, ComparisonTolerances::get("distance"))) {
            return true;
        } else if (very_close_equals(a.distance, b.distance, ComparisonTolerances::get("distance"))) {
            if (very_close_less(a.x, b.x, ComparisonTolerances::get("x"))) {
                return true;
            } else if (very_close_equals(a.x, b.x, ComparisonTolerances::get("x"))) {
                if (very_close_less(a.y, b.y, ComparisonTolerances::get("y"))) {
                    return true;
                } else if (very_close_equals(a.y, b.y, ComparisonTolerances::get("y"))) {
                    return very_close_less(a.theta, b.theta, ComparisonTolerances::get("theta"));
                }
            }
        }
        return false;
    }
};

struct MotionPrimitive {
    double forward;
    double right;
    double angle;
    double distance;

    // static MotionPrimitive FromJSON(json primitive) {
    //     return MotionPrimitive{
    //         .forward = primitive["forward"],
    //         .right = primitive["right"],
    //         .angle = primitive["angle"],
    //         .distance = primitive["distance"],
    //     };
    // }

    State apply(State state) const {
        return State{
            .x = state.x + std::cos(deg_to_rad(state.theta)) * forward + std::sin(deg_to_rad(state.theta)) * right,
            .y = state.y + std::sin(deg_to_rad(state.theta)) * forward - std::cos(deg_to_rad(state.theta)) * right,
            .theta = mod_interval(state.theta + angle, 360.0),
            .distance = state.distance + distance,
        };
    }
};

using MotionPrimitives = std::vector<MotionPrimitive>;

struct CollisionTester {
    CollisionTester(msg::Scene::SharedPtr scene) : scene(scene) {
    }

    bool test(State) {
        return false;
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
            if (closed_set.contains(next_state) || open_set_checker.contains(next_state) || tester.test(next_state)) {
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

msg::Path plan(CollisionTester tester, MotionPrimitives primitives, State initial, State target) {
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

    msg::Path result;

    if (found) {
        result.exists = true;

        State current = target;
        msg::Point point;
        while (current != initial) {
            point.x = current.x;
            point.y = current.y;
            point.theta = current.theta;
            result.trajectory.push_back(point);
            
            current = state_space.origin[current];
        }
        
        point.x = current.x;
        point.y = current.y;
        point.theta = current.theta;
        result.trajectory.push_back(point);
        std::reverse(result.trajectory.begin(), result.trajectory.end());
    } else {
        result.exists = false;
    }
    return result;
}

}

namespace planning_node {

struct Planner {
    Planner(
        std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue,
        rclcpp::Publisher<msg::Path>::SharedPtr path_publisher
    ) : scene_queue(scene_queue)
      , path_publisher(path_publisher) {
    }

    void start() {
        std::optional<msg::Scene::SharedPtr> scene;
        while ((scene = scene_queue->take()).has_value()) {
            CollisionTester tester{scene.value()};
            MotionPrimitives primitives{};  // todo
            State initial{};  // todo
            State target{};  // todo
            msg::Path path = plan(tester, primitives, initial, target);

            path_publisher->publish(path);
        }
    }

private:
    std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue;
    rclcpp::Publisher<msg::Path>::SharedPtr path_publisher;
};

std::thread start_planner(
    std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue,
    rclcpp::Publisher<msg::Path>::SharedPtr path_publisher
) {
    auto planner_func = [=]() {
        Planner planner{scene_queue, path_publisher};
        planner.start();
    };
    return std::thread{planner_func};

}

}
