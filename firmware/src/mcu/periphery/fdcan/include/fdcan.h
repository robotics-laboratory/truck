#ifndef TRUCK_HW_PERIPHERY_FDCAN_FDCAN_H_
#define TRUCK_HW_PERIPHERY_FDCAN_FDCAN_H_

#include "FreeRTOS.h"
#include "semphr.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

extern "C" {
void FDCAN1_IT0_IRQHandler(void);
void FDCAN1_IT1_IRQHandler(void);
}

class FDCAN {
 private:
  SemaphoreHandle_t mutex;
  FDCAN_HandleTypeDef fdcan_handle;

  bool is_initialized = false;
  void(*rx_new_data_callback)() = nullptr;

  FDCAN() {};
  ~FDCAN() {};
  FDCAN(const FDCAN &obj) = delete;
  FDCAN &operator=(const FDCAN &obj) = delete;

 public:
  static FDCAN &getInstance();
  FDCAN_HandleTypeDef *get_handle();

  int32_t get_message(uint32_t &message_id, uint8_t *rx_data, uint32_t &rx_data_size);
  void set_rx_complete_callback(void(*callback)());
  void on_new_can_frame();
  uint32_t transmit(uint32_t message_id, uint8_t *p_data, uint32_t size);
  int32_t init();
};
#endif //TRUCK_HW_PERIPHERY_FDCAN_FDCAN_H_
