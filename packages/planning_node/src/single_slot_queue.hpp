#pragma once
#include <condition_variable>
#include <mutex>
#include <optional>


template <typename T>
struct SingleSlotQueue {
    void put(const T& item) {
        {
            std::lock_guard<std::mutex> lock{mu};
            slot = item;
        }
        cv.notify_one();
    }

    std::optional<T> take() {
        std::unique_lock<std::mutex> lock{mu};
        cv.wait(lock, [this](){ return slot.has_value() || stopped; });
        if (stopped) {
            return {};
        }
        std::optional<T> res = std::move(slot);
        slot.reset();
        return res;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock{mu};
            stopped = true;
        }
        cv.notify_one();
    }

private:
    std::mutex mu;
    std::condition_variable cv;

    std::optional<T> slot;
    bool stopped;
};
