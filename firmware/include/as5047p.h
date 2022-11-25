#pragma once

#include <magnetic_encoder_spi.h>

namespace as5047p {

namespace {

constexpr uint16_t MAX_ANGLE = 1 << 14;

constexpr uint16_t ANGLEUNC = 0x3ffe;
constexpr uint16_t ANGLECOM = 0x3fff;

constexpr uint16_t DIAAGC = 0x3ffc;

constexpr uint16_t ZPOSM = 0x0016;
constexpr uint16_t ZPOSL = 0x0017;

}  // namespace

using magnetic_encoder_spi::MagneticEncoderSPI;

class AS5047p : public MagneticEncoderSPI {
  public:
    AS5047p(SPI_HandleTypeDef& spi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : MagneticEncoderSPI(spi, cs_port, cs_pin, ANGLEUNC, MAX_ANGLE) {
        frames::fill_command_frame(DIAAGC, frames::Operation::READ, _magnet_command_frame);
        frames::fill_command_frame(ZPOSM, frames::Operation::WRITE, _zposm_command_frame);
        frames::fill_command_frame(ZPOSL, frames::Operation::WRITE, _zposl_command_frame);
    }

    bool is_magnetic_field_good() {
        frames::ReadDataFrame diagnostic = _spi.read_register(_magnet_command_frame.raw);
        return !(
            frames::get_bit(diagnostic.values.data, 10) ||
            frames::get_bit(diagnostic.values.data, 11));
    }

    void reset_zero_angle() {
        set_new_zero_angle(0);
        reset();
    }

    void set_zero_angle() {
        reset_zero_angle();
        uint16_t angle = get_raw_angle();
        set_new_zero_angle(angle);
        reset();
    }

  private:
    void set_new_zero_angle(uint16_t angle) {
        const uint8_t LSB_SIZE = 6;
        const uint8_t MSB_SIZE = 8;
        uint16_t lsb = angle & ((1 << LSB_SIZE) - 1);
        uint16_t msb = (angle >> LSB_SIZE) & ((1 << MSB_SIZE) - 1);

        _spi.write_register(_zposl_command_frame.raw, lsb);
        _spi.write_register(_zposm_command_frame.raw, msb);
    }

    frames::CommandFrame _magnet_command_frame;
    frames::CommandFrame _zposm_command_frame;
    frames::CommandFrame _zposl_command_frame;
};

}  // namespace as5047p