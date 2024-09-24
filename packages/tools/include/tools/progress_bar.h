#include <string>
#include <iostream>
#include <chrono>

struct Output : std::ostream, std::streambuf {
    Output(bool enabled) : std::ostream(this), m_enabled(enabled) {}

    int overflow(int c) {
        if (m_enabled) std::cout.put(c);
        return 0;
    }

    bool m_enabled;
};

namespace tools {
class ProgressBar {
  public:
    ProgressBar(uint32_t, std::ostream& = std::cout, const std::string& = "");

    // ProgressBar has to be non-copyable
    ProgressBar(const ProgressBar& o) = delete;
    ProgressBar& operator=(const ProgressBar& o) = delete;

    void operator++();

  private:
    uint32_t count_ = 0;
    uint32_t max_count_;
    std::ostream& os_;
    std::string title_;
    std::chrono::time_point<
        std::chrono::steady_clock, std::chrono::duration<long, std::ratio<1, 1000000000>>>
        start_time_point;

    std::string GetTitleString();
    std::string GetBarString();
    std::string GetTimeString();
    void UpdateVisual();

    float GetProgressFraction();
};
}  // namespace tools