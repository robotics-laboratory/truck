#include <iostream>
#include <chrono>
#include <stack>
#include <vector>
#include <string>

namespace tools {
class BenchmarkTimer;

class BenchmarkManager {
  public:
    void startTimer(const std::string&);

    void stopTimer();

    void displayStatistics() const;

  private:
    std::stack<BenchmarkTimer*> timer_stack;
    std::deque<BenchmarkTimer> timers;
};
}  // namespace tools
