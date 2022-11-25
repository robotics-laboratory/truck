#pragma once

#include <cstdint>

namespace frames {

enum Operation {
    WRITE = 0,
    READ = 1,
};

typedef union {
    uint16_t raw;
    struct __attribute__((packed)) {
        uint16_t addr : 14;
        uint16_t rw : 1;
        uint16_t pard : 1;
    } values;
} CommandFrame;

typedef union {
    uint16_t raw;
    struct __attribute__((packed)) {
        uint16_t data : 14;
        uint16_t ef : 1;
        uint16_t pard : 1;
    } values;
} ReadDataFrame;

typedef union {
    uint16_t raw;
    struct __attribute__((packed)) {
        uint16_t data : 14;
        uint16_t zero : 1;
        uint16_t pard : 1;
    } values;
} WriteDataFrame;

inline bool get_bit(uint16_t word, uint16_t position) { return (word & (1 << position)) != 0; }

inline bool get_parity_bit(uint16_t data) {
    size_t count = 0;
    for (size_t i = 0; i < 15; ++i) {
        if (get_bit(data, i)) {
            ++count;
        }
    }
    return (count % 2) != 0;
}

inline void fill_command_frame(uint16_t address, Operation operation, CommandFrame& command_frame) {
    command_frame.values.addr = address;
    command_frame.values.rw = operation;
    command_frame.values.pard = get_parity_bit(command_frame.raw);
}

}  // namespace frames