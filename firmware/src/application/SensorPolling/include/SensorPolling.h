#ifndef TRUCK_HW_APPLICATION_SENSORPOLLING_INCLUDE_SENSORPOLLING_H_
#define TRUCK_HW_APPLICATION_SENSORPOLLING_INCLUDE_SENSORPOLLING_H_

#include "FreeRTOS.h"
#include "task.h"

class SensorPolling {
  private:
    float magn_x, magn_y, magn_z;
    bool is_initialized = false;
    bool magn_updated = false;
    TaskHandle_t task_handle;
    void task();

    static void startTaskImpl(void* _this) { ((SensorPolling*)_this)->task(); }
    SensorPolling() {}
    ~SensorPolling() {}
    SensorPolling(const SensorPolling& obj) = delete;
    SensorPolling& operator=(const SensorPolling& obj) = delete;

  public:
    static SensorPolling& getInstance();

    int32_t init();
    void start();

    int32_t get_magn_data(float& x_axis, float& y_axis, float& z_axis);
};

#endif  // TRUCK_HW_APPLICATION_SENSORPOLLING_INCLUDE_SENSORPOLLING_H_
