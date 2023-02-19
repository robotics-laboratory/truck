#pragma once

#include <cstdint>
#include <vector>

namespace truck::perf {

struct CpuStat {
    int16_t id = -1;  // -1 means total

    uint64_t user = 0;
    uint64_t nice = 0;
    uint64_t system;
    uint64_t idle = 0;
    uint64_t iowait = 0;
    uint64_t irq = 0;
    uint64_t softirq = 0;
    uint64_t steal = 0;
    uint64_t guest = 0;
    uint64_t guest_nice = 0;

    uint64_t total() const;
    uint64_t used() const;

    float usageRatio() const;
    uint16_t usagePercent() const;

    CpuStat operator-(const CpuStat& other) const;

    static std::vector<CpuStat> read();
};

struct MemStat {
    uint64_t total = 0;
    uint64_t free = 0;
    uint64_t available = 0;

    uint64_t used() const;

    float usageRatio() const;
    uint16_t usagePercent() const;

    static MemStat read();
};

struct Stat {
    std::vector<CpuStat> cpu;
    MemStat mem;

    static Stat read();
};

}  // namespace truck::perf