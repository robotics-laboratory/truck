#include "json/json.hpp"
#include "float_comparison/float_comparison.hpp"

#include <fstream>
#include <iostream>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>


using nlohmann::json;

namespace {
    inline void hashCombine(std::size_t& seed) {}

    template <typename T, typename... Rest>
    inline void hashCombine(std::size_t& seed, const T& v, Rest... rest) {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        hashCombine(seed, rest...);
    }

    double modInterval(double x, double modulo) {
        return std::fmod(std::fmod(x, modulo) + modulo, modulo);
    }

    double degToRad(double deg) {
        return deg * M_PI / 180.0;
    }
}

struct ComparisonTolerances {
    std::unordered_map<std::string, double> values;

    static std::unique_ptr<ComparisonTolerances> instance;

    static double Get(std::string key) {
        return instance->values[key];
    }

    static void LoadFromJSON(json tolerances) {
        instance = std::make_unique<ComparisonTolerances>();
        for (auto it = tolerances.begin(); it != tolerances.end(); ++it) {
            instance->values[it.key()] = it.value();
        }
    }
};

std::unique_ptr<ComparisonTolerances> ComparisonTolerances::instance = nullptr;

struct State {
    double X;
    double Y;
    double Theta;

    double Distance;

    static State FromJSON(json state) {
        return State{
            .X = state["x"],
            .Y = state["y"],
            .Theta = modInterval(state["theta"], 360.0),
            .Distance = 0.0,
        };
    }

    json ToJSON() const {
        json state;
        state["x"] = X;
        state["y"] = Y;
        state["theta"] = Theta;
        return state;
    }

    bool operator==(State o) const {
        return very_close_equals(X, o.X, ComparisonTolerances::Get("x")) && 
            very_close_equals(Y, o.Y, ComparisonTolerances::Get("y")) &&
            very_close_equals(Theta, o.Theta, ComparisonTolerances::Get("theta"));
    }
    bool operator!=(State o) const {
        return !(*this == o);
    }
};

template <>
struct std::hash<State> {
    std::size_t operator()(const State& s) const {
        size_t res = 0;
        hashCombine(res, s.X, s.Y, s.Theta);
        return res;
    }
};

struct StateComparator {
    bool operator()(State a, State b) const {
        // todo: review this (is this a valid comparator?)
        if (a == b) {
            return false;
        }

        // todo: this looks ugly, maybe rewrite it with macros
        if (very_close_less(a.Distance, b.Distance, ComparisonTolerances::Get("distance"))) {
            return true;
        } else if (very_close_equals(a.Distance, b.Distance, ComparisonTolerances::Get("distance"))) {
            if (very_close_less(a.X, b.X, ComparisonTolerances::Get("x"))) {
                return true;
            } else if (very_close_equals(a.X, b.X, ComparisonTolerances::Get("x"))) {
                if (very_close_less(a.Y, b.Y, ComparisonTolerances::Get("y"))) {
                    return true;
                } else if (very_close_equals(a.Y, b.Y, ComparisonTolerances::Get("y"))) {
                    return very_close_less(a.Theta, b.Theta, ComparisonTolerances::Get("theta"));
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

    static MotionPrimitive FromJSON(json primitive) {
        return MotionPrimitive{
            .forward = primitive["forward"],
            .right = primitive["right"],
            .angle = primitive["angle"],
            .distance = primitive["distance"],
        };
    }

    State Apply(State state) const {
        return State{
            .X = state.X + std::cos(degToRad(state.Theta)) * forward + std::sin(degToRad(state.Theta)) * right,
            .Y = state.Y + std::sin(degToRad(state.Theta)) * forward - std::cos(degToRad(state.Theta)) * right,
            .Theta = modInterval(state.Theta + angle, 360.0),
            .Distance = state.Distance + distance,
        };
    }
};

using MotionPrimitives = std::vector<MotionPrimitive>;

struct CollisionTester {
    bool Test(State state) {
        return false;
    }
};

struct StateSpace {
    CollisionTester tester;
    MotionPrimitives primitives;
    std::set<State, StateComparator> openSet;
    std::unordered_set<State> openSetChecker;
    std::unordered_set<State> closedSet;

    std::unordered_map<State, State> Origin;

    StateSpace(CollisionTester tester, MotionPrimitives primitives, State target)
        : tester{tester}
        , primitives{primitives}
        , openSet{StateComparator{}} {
    }

    State GetOptimal() {
        return *openSet.begin();
    }

    void ExpandOptimal() {
        State optimal = GetOptimal();
        openSet.erase(openSet.begin());
        openSetChecker.erase(optimal);
        closedSet.insert(optimal);

        for (MotionPrimitive primitive : primitives) {
            State nextState = primitive.Apply(optimal);
            if (closedSet.contains(nextState) || openSetChecker.contains(nextState) || tester.Test(nextState)) {
                continue;
            }
            openSet.insert(nextState);
            openSetChecker.insert(nextState);
            Origin[nextState] = optimal;
        }
    }

    bool Empty() const {
        return openSet.empty();
    }
    
    void Insert(State state) {
        openSet.insert(state);
        openSetChecker.insert(state);
    }
};

struct Path {
    bool Exists;
    std::vector<State> Sequence;
};

Path Plan(CollisionTester tester, MotionPrimitives primitives, State initial, State target) {
    StateSpace stateSpace(tester, primitives, target);
    stateSpace.Insert(initial);

    bool found = false;
    while (!stateSpace.Empty()) {
        State optimal = stateSpace.GetOptimal();
        if (optimal == target) {
            found = true;
            break;
        }

        stateSpace.ExpandOptimal();
    }

    if (found) {
        std::vector<State> sequence;
        State current = target;
        while (current != initial) {
            sequence.push_back(current);
            current = stateSpace.Origin[current];
        }
        sequence.push_back(current);

        return {.Exists = true, .Sequence = sequence};
    }
    return {.Exists = false};
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config-path>" << std::endl;
        std::exit(1);
    }

    std::ifstream config_stream{argv[1]};
    json config = json::parse(config_stream);

    ComparisonTolerances::LoadFromJSON(config["tolerances"]);

    CollisionTester tester;
    MotionPrimitives primitives;
    for (auto json_primitive : config["primitives"]) {
        primitives.push_back(MotionPrimitive::FromJSON(json_primitive));
    }
    State initial = State::FromJSON(config["initial"]);
    State target = State::FromJSON(config["target"]);

    Path path = Plan(tester, primitives, initial, target);
    
    json json_path;
    json_path["exists"] = path.Exists;

    json sequence;
    for (State state : path.Sequence) {
        sequence.push_back(state.ToJSON());
    }
    json_path["sequence"] = sequence;

    std::ofstream path_stream{"path.json"};
    path_stream << json_path.dump(2);

    return 0;
}