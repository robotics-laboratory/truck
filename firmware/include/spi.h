#pragma once

#include <stm32f1xx.h>
#include <frames.h>
#include <gpio.h>

namespace spi {

namespace {

constexpr uint32_t SPI_TIMEOUT = 1000;

}  // namespace

class SPI {
  public:
    SPI(SPI_HandleTypeDef& spi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : _spi(spi), _cs_port(cs_port), _cs_pin(cs_pin) {
        deselect();
    }

    void select() {
        if (_spi.Init.CLKPolarity == SPI_POLARITY_LOW) {
            gpio::write_pin(_cs_port, _cs_pin, GPIO_PIN_RESET);
        } else {
            gpio::write_pin(_cs_port, _cs_pin, GPIO_PIN_SET);
        }
    }

    void deselect() {
        while (_spi.State == HAL_SPI_STATE_BUSY) {
        }

        if (_spi.Init.CLKPolarity == SPI_POLARITY_LOW) {
            gpio::write_pin(_cs_port, _cs_pin, GPIO_PIN_SET);
        } else {
            gpio::write_pin(_cs_port, _cs_pin, GPIO_PIN_RESET);
        }
    }

    void write_read(uint16_t tx_data, uint16_t& rx_data, uint16_t size) {
        select();
        _spi.Instance->CR1 |= SPI_CR1_SPE;

        while (((_spi.Instance->SR) & (SPI_FLAG_TXE)) == 0) {
        }
        _spi.Instance->DR = tx_data;

        while (((_spi.Instance->SR) & (SPI_FLAG_RXNE)) == 0) {
        }
        rx_data = _spi.Instance->DR;

        deselect();
    }

    frames::ReadDataFrame read_register(uint16_t register_data) {
        frames::ReadDataFrame received_frame;
        write_read(register_data, received_frame.raw, 1);
        write_read(_noop_frame.raw, received_frame.raw, 1);
        return received_frame;
    }

    void write_register(uint16_t register_data, uint16_t tx_data) {
        uint16_t mock_rx;
        write_read(register_data, mock_rx, 1);
        write_read(tx_data, mock_rx, 1);
    }

  private:
    SPI_HandleTypeDef& _spi;
    GPIO_TypeDef* _cs_port;
    uint16_t _cs_pin;
    frames::CommandFrame _noop_frame;
};

}  // namespace spi