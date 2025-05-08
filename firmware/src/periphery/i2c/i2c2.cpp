#include "i2c2.h"

#include <cstdio>

#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_i2c.h"
#include "board.h"

I2C& I2C::getInstance() {
    static I2C _instance;
    return _instance;
}

void I2C::restart(void) {
    LL_I2C_Disable(i2c_handle);
    LL_I2C_Enable(i2c_handle);
}

uint32_t I2C::wait_register(
    uint32_t(wait_register)(I2C_TypeDef*), bool wait_state, uint32_t time_out_ts) {
    uint32_t status = 0;

    while (wait_register(i2c_handle) == wait_state) {
        if (time_out_ts < board_get_tick()) {
            restart();
            status = 1;
            break;
        }
    };
    status = 0;
    return status;
}

uint32_t I2C::init(void) {
    uint32_t status = 0;

    if (is_initialized == false) {
        LL_I2C_InitTypeDef I2C_InitStruct = {0};
        LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

        LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_PCLK1);
        LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

        /**I2C2 GPIO Configuration
         PA8   ------> I2C2_SDA
        PA9   ------> I2C2_SCL
        */
        GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
        GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
        GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Peripheral clock enable */
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
        I2C_InitStruct.Timing = 0x00E057FD;
        I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
        I2C_InitStruct.DigitalFilter = 0;
        I2C_InitStruct.OwnAddress1 = 0;
        I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
        I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
        LL_I2C_Init(I2C2, &I2C_InitStruct);
        LL_I2C_EnableAutoEndMode(I2C2);
        LL_I2C_SetOwnAddress2(I2C2, 0, LL_I2C_OWNADDRESS2_NOMASK);
        LL_I2C_DisableOwnAddress2(I2C2);
        LL_I2C_DisableGeneralCall(I2C2);
        LL_I2C_EnableClockStretching(I2C2);

        mutex = xSemaphoreCreateMutex();

        if (mutex != NULL) {
            is_initialized = true;
        } else {
            status = 1;
        }
    }
    return status;
}

uint32_t I2C::read_bytes(
    uint8_t chip_address, uint8_t register_address, uint8_t* ptr_bytes, size_t size,
    uint32_t time_out) {
    uint32_t status = 0;
    do {
        if (is_initialized != true) {
            status = 1;
            break;
        }

        if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
            status = 2;
            break;
        }

        uint32_t time_out_ts = board_get_tick() + time_out;
        if (wait_register(LL_I2C_IsActiveFlag_BUSY, true, time_out_ts) != 0) {
            status = 3;
            break;
        }

        LL_I2C_HandleTransfer(
            i2c_handle,
            (chip_address << 1),
            LL_I2C_ADDRSLAVE_7BIT,
            1,
            LL_I2C_MODE_SOFTEND,
            LL_I2C_GENERATE_START_WRITE);

        if (wait_register(LL_I2C_IsActiveFlag_TXE, false, time_out_ts) != 0) {
            status = 4;
            break;
        }

        if (LL_I2C_IsActiveFlag_NACK(i2c_handle) == SET) {
            status = 4;
            break;
        }

        LL_I2C_TransmitData8(i2c_handle, register_address);

        if (wait_register(LL_I2C_IsActiveFlag_TC, false, time_out_ts) != 0) {
            status = 5;
            break;
        }

        if (LL_I2C_IsActiveFlag_NACK(i2c_handle) == SET) {
            status = 4;
            break;
        }

        LL_I2C_HandleTransfer(
            i2c_handle,
            (chip_address << 1),
            LL_I2C_ADDRSLAVE_7BIT,
            size,
            LL_I2C_MODE_AUTOEND,
            LL_I2C_GENERATE_START_READ);

        for (size_t i = 0; i < size; i++) {
            if (wait_register(LL_I2C_IsActiveFlag_RXNE, false, time_out_ts) != 0) {
                status = 6;
                break;
            }
            ptr_bytes[i] = LL_I2C_ReceiveData8(i2c_handle);
        }
        if (status != 0) {
            break;
        }

        if (wait_register(LL_I2C_IsActiveFlag_STOP, false, time_out_ts) != 0) {
            status = 7;
            break;
        }

        LL_I2C_ClearFlag_STOP(i2c_handle);
    } while (0);

    if ((status == 0) || (status > 1)) {
        xSemaphoreGive(mutex);
    }
    return status;
}

uint32_t I2C::write_bytes(
    uint8_t chip_address, uint8_t register_address, uint8_t* ptr_bytes, size_t size,
    uint32_t time_out) {
    uint32_t status = 0;

    uint32_t i = 0;

    do {
        if (is_initialized != true) {
            status = 1;
            break;
        }

        if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
            status = 2;
            break;
        }

        uint32_t time_out_ts = board_get_tick();

        if (wait_register(LL_I2C_IsActiveFlag_BUSY, true, time_out_ts) != 0) {
            status = 3;
            break;
        }

        LL_I2C_HandleTransfer(
            i2c_handle,
            (chip_address << 1),
            LL_I2C_ADDRSLAVE_7BIT,
            size + 1,
            LL_I2C_MODE_AUTOEND,
            LL_I2C_GENERATE_START_WRITE);
        // TODO check NACK
        if (wait_register(LL_I2C_IsActiveFlag_TXE, false, time_out_ts) != 0) {
            status = 4;
            break;
        }

        LL_I2C_TransmitData8(i2c_handle, register_address);
        while (LL_I2C_IsActiveFlag_STOP(i2c_handle) == false) {
            if (time_out_ts > board_get_tick()) {
                restart();
                status = 3;
                break;
            }
            if (LL_I2C_IsActiveFlag_TXIS(i2c_handle) == true) {
                LL_I2C_TransmitData8(i2c_handle, ptr_bytes[i++]);
            }
        }
        if (status != 0) {
            break;
        }
        LL_I2C_ClearFlag_STOP(i2c_handle);
    } while (0);

    if ((status == 0) || (status > 1)) {
        xSemaphoreGive(mutex);
    }
    return status;
}
