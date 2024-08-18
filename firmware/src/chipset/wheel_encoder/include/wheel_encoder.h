
#ifndef TRUCK_HW_CHIPSET_WHEEL_ENCODER_INCLUDE_WHEEL_ECNODER_H_
#define TRUCK_HW_CHIPSET_WHEEL_ENCODER_INCLUDE_WHEEL_ECNODER_H_
#include <cstdint>
#include "encoder_timer.h"
#include <unordered_map>

enum class WheelType : uint8_t {
  LEFT_FRONT,
  RIGHT_FRONT,
  LEFT_REAR,
  RIGHT_REAR
};


class WheelEncoder {
 private:
  static constexpr float pi = 3.1415;
  static constexpr float wheel_radius_mm = 30;
  static constexpr uint32_t steps_per_turn = 50;

  bool is_initialized = false;
  enum WheelType type;
  EncoderTimer& encoder_timer_handle;
  static constexpr uint32_t max_data_size = 50;
  std::vector<int> encoder_ticks;

  float last_speed = 0.0f;
//  const std::unordered_map<WheelType, EncoderType> wheel_encoder_map = {{std::pair<WheelType, EncoderType> (WheelType::LEFT_FRONT, EncoderType::ID_0)}};

  static EncoderTimer &get_timer_instance(WheelType wheel_type_) {
      switch (wheel_type_) {
          case WheelType::LEFT_FRONT:
              return EncoderTimer::get_instance(EncoderType::ID_0);
          case WheelType::RIGHT_FRONT:
              return EncoderTimer::get_instance(EncoderType::ID_1);
          case WheelType::LEFT_REAR:
              return EncoderTimer::get_instance(EncoderType::ID_2);
          case WheelType::RIGHT_REAR:
              return EncoderTimer::get_instance(EncoderType::ID_3);
      }
  }

  WheelEncoder(WheelType id) : encoder_timer_handle(get_timer_instance(id)) {
      encoder_ticks.resize(max_data_size);
      if (encoder_timer_handle.init() && (encoder_ticks.capacity() >= max_data_size)) {
          is_initialized = true;
      }
  };

  ~WheelEncoder() {};
  WheelEncoder(const WheelEncoder &obj) = delete;
  WheelEncoder &operator=(const WheelEncoder &obj) = delete;

  int init();

 public:
  static WheelEncoder &get_instance(WheelType id) {
      static std::unordered_map<WheelType, WheelEncoder *> instances;
      auto it = instances.find(id);
      if (it == instances.end()) {
          instances[id] = new WheelEncoder(id);
      }
      return *instances[id];
  }
  int calculate_speed();
};
#endif //TRUCK_HW_CHIPSET_WHEEL_ENCODER_INCLUDE_WHEEL_ECNODER_H_
