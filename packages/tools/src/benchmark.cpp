#include "tools/benchmark.h"

#include <iostream>
#include <chrono>
#include <stack>
#include <vector>
#include <string>

tools::BenchmarkManager benchmarkManager;

namespace tools {
class BenchmarkTimer {
  public:
    BenchmarkTimer(const std::string& timerName, int timerLevel) :
        name(timerName), level(timerLevel), start_time(std::chrono::steady_clock::now()) {}

    void stop() {
        end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        elapsed_time = elapsed.count();
    }

    void display(bool isLast) const {
        for (int i = 0; i < level - 1; ++i) {
            std::cout << "│   ";
        }
        if (level > 0) {
            std::cout << (isLast ? "└── " : "├── ");
        }
        uint32_t hours = static_cast<uint32_t>(elapsed_time) / 3600;
        uint32_t minutes = static_cast<uint32_t>(elapsed_time) / 60;
        uint32_t seconds = static_cast<uint32_t>(elapsed_time) % 60;

        std::cout << name << ": ";
        if (hours) {
            std::cout << hours << "h " << minutes << "m " << seconds << "s" << std::endl;
        } else if (minutes) {
            std::cout << minutes << "m " << seconds << "s" << std::endl;
        } else {
            std::cout << seconds << "s" << std::endl;
        }
    }

    std::string name;
    int level;
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point end_time;
    float elapsed_time;
};

void BenchmarkManager::startTimer(const std::string& timerName) {
    int level = timer_stack.size();
    timers.push_back(BenchmarkTimer(timerName, level));
    timer_stack.push(&timers.back());
}

void BenchmarkManager::stopTimer() {
    if (!timer_stack.empty()) {
        timer_stack.top()->stop();
        timer_stack.pop();
    } else {
        std::cerr << "No active timers to stop!" << std::endl;
        throw;
    }
}

void BenchmarkManager::displayStatistics() const {
    std::cout << "\nBENCHMARK STATS:\n";
    for (size_t i = 0; i < timers.size(); ++i) {
        bool isLast = (i == timers.size() - 1 || timers[i].level > timers[i + 1].level);
        timers[i].display(isLast);
    }
}
}  // namespace tools
