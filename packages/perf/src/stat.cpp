#include "perf/stat.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace truck::perf {

uint64_t CpuStat::used() const {
    return user + nice + system + irq + softirq + steal + guest + guest_nice;
}

uint64_t CpuStat::total() const { return used() + idle + iowait; }

float CpuStat::usageRatio() const { return static_cast<float>(used()) / total(); }

uint16_t CpuStat::usagePercent() const { return static_cast<uint16_t>(used() * 100 / total()); }

CpuStat CpuStat::operator-(const CpuStat& other) const {
    return {
        .user = user - other.user,
        .nice = nice - other.nice,
        .system = system - other.system,
        .idle = idle - other.idle,
        .iowait = iowait - other.iowait,
        .irq = irq - other.irq,
        .softirq = softirq - other.softirq,
        .steal = steal - other.steal,
        .guest = guest - other.guest,
        .guest_nice = guest_nice - other.guest_nice};
}

std::vector<CpuStat> CpuStat::read() {
    std::ifstream stat("/proc/stat");
    std::vector<CpuStat> result;

    for (std::string line; std::getline(stat, line); ) {
        if (not line.starts_with("cpu")) {
            continue;
        }

        CpuStat cpu;
        std::istringstream iss(line);
        std::string cpu_name;
        iss >> cpu_name >> cpu.user >> cpu.nice >> cpu.system >> cpu.idle >> cpu.iowait >>
            cpu.irq >> cpu.softirq >> cpu.steal >> cpu.guest >> cpu.guest_nice;

        // string with cpu id means total cpu stat
        if (cpu_name != "cpu") {
            cpu.id = std::stoi(cpu_name.substr(3));
        }

        result.push_back(cpu);
    }

    return result;
}

uint64_t MemStat::used() const { return total - free; }

float MemStat::usageRatio() const { return static_cast<double>(used()) / total; }

uint16_t MemStat::usagePercent() const { return static_cast<uint16_t>(used() * 100 / total); }

MemStat MemStat::read() {
    std::ifstream meminfo("/proc/meminfo");

    MemStat result;

    for (std::string line; std::getline(meminfo, line);) {
        if (line.starts_with("MemTotal")) {
            result.total = std::stoull(line.substr(9));
        } else if (line.starts_with("MemFree")) {
            result.free = std::stoull(line.substr(8));
        } else if (line.starts_with("MemAvailable")) {
            result.available = std::stoull(line.substr(13));
        }
    }

    return result;
}

Stat Stat::read() {
    return {
        .cpu = CpuStat::read(),
        .mem = MemStat::read(),
    };
}

}  // namespace truck::perf
