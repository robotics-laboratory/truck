#pragma once

#include <fstream>
#include <vector>
#include <regex>

namespace servo {

struct __attribute__((packed)) Angles {
    float left_angle;
    float right_angle;
};

struct SteeringControl {
    SteeringControl(const std::string& path) : stream_(path) { steering_ = parse(stream_); }

    static std::unique_ptr<SteeringControl> create(const std::string& path) {
        return std::make_unique<SteeringControl>(path);
    }

    std::vector<std::pair<float, float>> parse(std::istream& stream) {
        std::vector<std::pair<float, float>> result;
        std::string line;
        std::getline(stream, line);
        std::regex fieldsRegx(",");
        while (std::getline(stream, line)) {
            std::sregex_token_iterator iter(line.begin(), line.end(), fieldsRegx, -1);
            float key = std::stof(iter->str());
            ++iter;
            float value = std::stof(iter->str());
            result.emplace_back(key, value);
        }
        return result;
    }

    std::ifstream stream_;
    std::vector<std::pair<float, float>> steering_;
};

}  // namespace servo
