#pragma once

#include <spi.h>

#include <cstdio>

extern USART_HandleTypeDef husart1;

namespace as5047p {

namespace {

constexpr float ANGLE_TO_DEGREE = 360.0 / (1 << 14);
constexpr uint32_t SPI_SPEED = 10'000'000;

constexpr uint16_t ANGLEUNC = 0x3ffe;
constexpr uint16_t ANGLECOM = 0x3fff;

constexpr uint16_t DIAAGC = 0x3ffc;

constexpr uint16_t ZPOSM = 0x0016;
constexpr uint16_t ZPOSL = 0x0017;

constexpr uint16_t NOOP = 0x0000;

enum Operation {
  WRITE = 0,
  READ = 1,
};

}  // namespace

class AS5047p {
 public:
  AS5047p(SPI_HandleTypeDef& spi) : _spi(spi) {
    fill_command_frame(ANGLEUNC, Operation::READ, _angle_command_frame);
    fill_command_frame(DIAAGC, Operation::READ, _magnet_command_frame);

    fill_command_frame(ZPOSM, Operation::WRITE, _zposm_command_frame);
    fill_command_frame(ZPOSL, Operation::WRITE, _zposl_command_frame);

    fill_command_frame(NOOP, Operation::READ, _noop_frame);
  }

  uint8_t get_uncompressed_angle() {
    return static_cast<float>(get_14bit_angle()) * ANGLE_TO_DEGREE;
  }

  bool is_magnetic_field_good() {
    ReadDataFrame diagnostic = read_register(_magnet_command_frame.raw);
    return !(get_bit(diagnostic.values.data, 10) || get_bit(diagnostic.values.data, 11));
  }

  void reset_zero_angle() {
    set_new_zero_angle(0);
  }

  void set_zero_angle() {
    reset_zero_angle();
    uint16_t angle = get_14bit_angle();
    set_new_zero_angle(angle);
  }

 private:
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

  void fill_command_frame(uint16_t address, Operation operation,
                          CommandFrame& command_frame) {
    command_frame.values.addr = address;
    command_frame.values.rw = operation;
    command_frame.values.pard = get_parity_bit(command_frame.raw);
  }

  uint16_t get_14bit_angle() {
    return read_register(_angle_command_frame.raw).values.data;
  }

  ReadDataFrame read_register(uint16_t tx_data) {
    ReadDataFrame received_frame;
    _spi.write_read(tx_data, received_frame.raw, 1);
    _spi.write_read(_noop_frame.raw, received_frame.raw, 1);
    return received_frame;
  }

  void write_register(uint16_t tx_data_1, uint16_t tx_data_2) {
    uint16_t rx;
    _spi.write_read(tx_data_1, rx, 1);
    _spi.write_read(tx_data_2, rx, 1);
  }

  void set_new_zero_angle(uint16_t angle) {
    const uint8_t LSB_SIZE = 6;
    const uint8_t MSB_SIZE = 8;
    uint16_t lsb = angle & ((1 << LSB_SIZE) - 1);
    uint16_t msb = (angle >> LSB_SIZE) & ((1 << MSB_SIZE) - 1);

    write_register(_zposl_command_frame.raw, lsb);
    write_register(_zposm_command_frame.raw, msb);
  }

  bool get_bit(uint16_t word, uint16_t position) {
    return (word & (1 << position)) != 0;
  }

  bool get_parity_bit(uint16_t data) {
    size_t count = 0;
    for (size_t i = 0; i < 15; ++i) {
      if (get_bit(data, i)) {
        ++count;
      }
    }
    return (count % 2) != 0;
  }

  spi::SPI _spi;

  CommandFrame _angle_command_frame;
  CommandFrame _magnet_command_frame;
  CommandFrame _zposm_command_frame;
  CommandFrame _zposl_command_frame;
  CommandFrame _noop_frame;
};

}  // namespace as5047p