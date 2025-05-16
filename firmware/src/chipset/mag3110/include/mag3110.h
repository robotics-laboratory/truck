#ifndef TRUCK_HW_CHIPSET_MAG3110_MAG3110_H_
#define TRUCK_HW_CHIPSET_MAG3110_MAG3110_H_

#include "i2c2.h"

class MAG3110 {
  private:
    I2C& i2c;
    const uint8_t i2c_address = 0x0E;

    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;

    MAG3110() : i2c(I2C::getInstance()){};
    ~MAG3110(){};
    MAG3110(const MAG3110& obj) = delete;
    MAG3110& operator=(const MAG3110& obj) = delete;

    int32_t read_data_from_sensor(uint8_t register_address, uint8_t* ptr_data, uint8_t size);
    int32_t write_data_to_sensor(uint8_t register_address, uint8_t* ptr_data, uint8_t size);

  public:
    static MAG3110& getInstance();
    int32_t init();
    int32_t get_magn_z(uint16_t& x_axis, uint16_t& y_axis, uint16_t& z_axis);
};

#endif  // TRUCK_HW_CHIPSET_MAG3110_MAG3110_H_
