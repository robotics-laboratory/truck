#pragma once
#include "planning_interfaces/msg/scene.hpp"
#include "planning_interfaces/msg/point.hpp"

#include <condition_variable>
#include <mutex>
#include <optional>

namespace planning_node {

using namespace planning_interfaces;

struct SharedState {
    using SharedPtr = std::shared_ptr<SharedState>;
    using Value = std::pair<msg::Scene::SharedPtr, msg::Point::SharedPtr>;

    void set_scene(msg::Scene::SharedPtr item) {
        {
            std::lock_guard<std::mutex> lock{mu};
            scene = item;
            updated = scene && target;
        }
        cv.notify_all();
    }

    void set_target(msg::Point::SharedPtr item) {
        {
            std::lock_guard<std::mutex> lock{mu};
            target = item;
            updated = scene && target;
        }
        cv.notify_all();
    }

    Value take() {
        std::unique_lock<std::mutex> lock{mu};
        cv.wait(lock, [this](){ return updated || stopped; });
        if (stopped) {
            return {};
        }
        updated = false;
        return {scene, target};
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock{mu};
            stopped = true;
        }
        cv.notify_all();
    }

private:
    std::mutex mu;
    std::condition_variable cv;

    msg::Scene::SharedPtr scene;
    msg::Point::SharedPtr target;
    bool updated = false;
    bool stopped = false;
};

}
