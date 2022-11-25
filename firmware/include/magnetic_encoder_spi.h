#pragma once

#include <spi.h>
#include <stm32f1xx.h>
#include <cmath>
#include <serial.h>

extern USART_HandleTypeDef husart1;

namespace magnetic_encoder_spi {

constexpr uint16_t NOOP = 0x0000;

class MagneticEncoderSPI {
  public:
    MagneticEncoderSPI(
        SPI_HandleTypeDef& spi, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint16_t angle_register,
        float max_angle)
        : _spi(spi, cs_port, cs_pin), _max_angle(max_angle), _angle_to_degree(360.0 / max_angle) {
        frames::fill_command_frame(angle_register, frames::Operation::READ, _angle_command_frame);
        frames::fill_command_frame(NOOP, frames::Operation::READ, _noop_frame);
    }

    uint8_t get_angle() { return static_cast<float>(get_raw_angle()) * _angle_to_degree; }

    int64_t get_number_of_rotations() const { return _number_of_rotations; }

    void update() {
        int32_t new_raw_angle = get_raw_angle();
        int32_t angle_delta = (new_raw_angle + _max_angle - _prev_raw_angle) % _max_angle;
        if (angle_delta > _max_angle / 2) {
            angle_delta -= _max_angle;
        }
        _angle_remains += angle_delta;
        if (std::abs(_angle_remains) > _max_angle) {
            _number_of_rotations += _angle_remains / _max_angle;
            _angle_remains %= _max_angle;
        }

        _prev_raw_angle = new_raw_angle;
    }

    void reset() {
        _number_of_rotations = 0;
        _angle_remains = 0;
        _prev_raw_angle = 0;
    }

  protected:
    uint16_t get_raw_angle() { return _spi.read_register(_angle_command_frame.raw).values.data; }

    spi::SPI _spi;

    frames::CommandFrame _angle_command_frame;
    frames::CommandFrame _noop_frame;

  private:
    const uint16_t _max_angle;
    const float _angle_to_degree;
    int64_t _number_of_rotations = 0;
    int32_t _angle_remains = 0;
    uint32_t _prev_raw_angle = 0;
};

}  // namespace magnetic_encoder_spi