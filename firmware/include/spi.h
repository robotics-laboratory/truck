#pragma once

#include <main.h>
#include <stm32f1xx.h>

namespace spi {

namespace {

constexpr uint32_t SPI_TIMEOUT = 1000;

}  // namespace

class SPI {
 public:
  SPI(SPI_HandleTypeDef& spi) : _spi(spi) {
    deselect();
  }

  void delay() const {
    HAL_Delay(1);
  }

  void select() {
    if (_spi.Init.CLKPolarity == SPI_POLARITY_LOW) {
      HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    }
  }

  void deselect() {
    while (_spi.State == HAL_SPI_STATE_BUSY)
      ;
    if (_spi.Init.CLKPolarity == SPI_POLARITY_LOW) {
      HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    }
  }

  void write_read(uint8_t* tx_data, uint8_t* rx_data, uint16_t size) {
    select();
    HAL_SPI_TransmitReceive(&_spi, (uint8_t*)tx_data, (uint8_t*)rx_data, size,
                            SPI_TIMEOUT);
    deselect();
  }

 private:
  SPI_HandleTypeDef& _spi;
};

}  // namespace spi