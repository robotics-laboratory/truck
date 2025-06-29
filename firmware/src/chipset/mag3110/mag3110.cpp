#include "mag3110.h"

#include "system_clock.h"

// MAG3110 Register Addresses
#define MAG3110_DR_STATUS (0x00)
#define MAG3110_OUT_X_MSB (0x01)
#define MAG3110_OUT_X_LSB (0x02)
#define MAG3110_OUT_Y_MSB (0x03)
#define MAG3110_OUT_Y_LSB (0x04)
#define MAG3110_OUT_Z_MSB (0x05)
#define MAG3110_OUT_Z_LSB (0x06)
#define MAG3110_WHO_AM_I (0x07)
#define MAG3110_SYSMOD (0x08)
#define MAG3110_OFF_X_MSB (0x09)
#define MAG3110_OFF_X_LSB (0x0A)
#define MAG3110_OFF_Y_MSB (0x0B)
#define MAG3110_OFF_Y_LSB (0x0C)
#define MAG3110_OFF_Z_MSB (0x0D)
#define MAG3110_OFF_Z_LSB (0x0E)
#define MAG3110_CTRL_REG1 (0x10)
#define MAG3110_CTRL_REG2 (0x11)

#define MAG3110_WHO_AM_I_VALUE (0xC4)

int32_t MAG3110::read_data_from_sensor(uint8_t register_address, uint8_t* ptr_data, uint8_t size) {
    return i2c.read_bytes(i2c_address, register_address, ptr_data, size, 15);
}

int32_t MAG3110::write_data_to_sensor(uint8_t register_address, uint8_t* ptr_data, uint8_t size) {
    return i2c.write_bytes(i2c_address, register_address, ptr_data, size, 15);
}

MAG3110& MAG3110::getInstance() {
    static MAG3110 _instance;
    return _instance;
}

int32_t MAG3110::init() {
    int32_t status = 0;
    uint8_t reg_data = 0x00;
    i2c.init();

    do {
        if (i2c.init() != 0) {
            status = 1;
            break;
        }

        reg_data = (1U << 4U);
        if (write_data_to_sensor(MAG3110_CTRL_REG2, &reg_data, 1) != 0) {
            status = 2;
            break;
        }
        system_clock_delay_ticks(10);

        if (read_data_from_sensor(MAG3110_WHO_AM_I, &reg_data, 1) != 0) {
            status = 3;
            break;
        }
        if (reg_data != MAG3110_WHO_AM_I_VALUE) {
            status = 4;
            break;
        }

        reg_data = (1U << 0U);  // Active mode, Trig off, Fast-read disable, 80hz
        if (write_data_to_sensor(MAG3110_CTRL_REG1, &reg_data, 1) != 0) {
            status = 5;
            break;
        }

        reg_data = (1U << 5U);  // RAW mode
        if (write_data_to_sensor(MAG3110_CTRL_REG2, &reg_data, 1) != 0) {
            status = 6;
            break;
        }

        reg_data = (1U << 0U);  // Active mode, RAW data
        if (write_data_to_sensor(MAG3110_SYSMOD, &reg_data, 1) != 0) {
            status = 7;
            break;
        }
    } while (0);

    return status;
}

int32_t MAG3110::get_magn_z(uint16_t& x_axis, uint16_t& y_axis, uint16_t& z_axis) {
    int32_t status = 0;
    uint8_t reg_data[6] = {0x00};

    if (read_data_from_sensor(MAG3110_DR_STATUS, reg_data, 1) != 0) {
        status = 5;
    }

    reg_data[0] = 0;
    if (read_data_from_sensor(MAG3110_OUT_X_MSB, reg_data, 6) == 0) {
        x_axis = reg_data[0] << 8 | reg_data[1];
        y_axis = reg_data[2] << 8 | reg_data[3];
        z_axis = reg_data[4] << 8 | reg_data[5];
    } else {
        x_axis = 0;
        y_axis = 0;
        z_axis = 0;
        status = 1;
    }

    return status;
}
