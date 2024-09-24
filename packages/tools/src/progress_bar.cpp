#include "tools/progress_bar.h"

#include <stdint.h>
#include <iomanip>
#include <vector>

namespace tools {

void ClearLines(std::ostream& os, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        os << "\033[F\033[2K";
    }
}

const int PROGRESS_BAR_LENGTH = 50;
const std::array<std::string, 9> LOADING_CHARS = {"", "▏", "▎", "▍", "▌", "▋", "▊", "▉", "█"};

std::string Repeat(const std::string& s, int n) {
    std::ostringstream ss;
    for (int i = 0; i < n; i++) {
        ss << s;
    }
    return ss.str();
}

std::string GetTimerString(uint32_t hours, uint32_t minutes, uint32_t seconds) {
    std::stringstream ss;
    if (hours != 0) {
        ss << std::setw(2) << std::setfill('0') << hours << ":";
    }
    ss << std::setw(2) << std::setfill('0') << minutes << ":";
    ss << std::setw(2) << std::setfill('0') << seconds;
    return ss.str();
}

ProgressBar::ProgressBar(uint32_t max_count, std::ostream& os, const std::string& title) :
    max_count_(max_count), os_(os), title_(title) {
    start_time_point = std::chrono::steady_clock::now();
}

std::string ProgressBar::getTitleString() {
    std::ostringstream ss;
    ss << title_ << ": " << count_ << " / " << max_count_;
    return ss.str();
}

std::string ProgressBar::getTimeString() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_point).count();
    uint32_t hours = elapsed / 3600;
    uint32_t minutes = elapsed / 60;
    uint32_t seconds = elapsed % 60;

    uint64_t elapsed_left = count_ != 0 ? elapsed / getProgressFraction() - elapsed : 0;
    uint32_t hours_left = elapsed_left / 3600;
    uint32_t minutes_left = elapsed_left / 60;
    uint32_t seconds_left = elapsed_left % 60;

    std::string elapsed_timer = GetTimerString(hours, minutes, seconds);
    std::string left_timer = GetTimerString(hours_left, minutes_left, seconds_left);

    return "[elapsed: " + elapsed_timer + " left: " + left_timer + "]";
}

std::string ProgressBar::getBarString() {
    float fraction = getProgressFraction();
    std::ostringstream ss;
    ss << ' ' << std::setw(3) << static_cast<int>(fraction * 100) << "."
       << static_cast<int>(fraction * 1000) % 10 << std::right << "% ";
    uint32_t whole_size = static_cast<int>(fraction * PROGRESS_BAR_LENGTH);
    uint32_t remaining = static_cast<int>((fraction * PROGRESS_BAR_LENGTH - whole_size) * 8);
    ss << "[" << Repeat(LOADING_CHARS.back(), whole_size) << LOADING_CHARS[remaining]
       << std::string(PROGRESS_BAR_LENGTH - whole_size - (remaining != 0), ' ') << "]";
    return ss.str();
}

void ProgressBar::operator++() {
    ++count_;
    updateVisual();
}

void ProgressBar::updateVisual() {
    ClearLines(os_, 2);
    os_ << getTitleString() << "\n" << getBarString() << " " << getTimeString() << std::endl;
}

float ProgressBar::getProgressFraction() { return static_cast<float>(count_) / max_count_; }
}  // namespace tools