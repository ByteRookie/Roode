#include "esphome_i2c_bridge.h"
#include "vl53l1x.h"

namespace esphome::vl53l1x {

int8_t vl53l1x_platform_write_multi(uint8_t device_address, uint16_t register_address, const uint8_t *data,
                                    uint32_t count) {
  auto *sensor = VL53L1X::get_active_sensor();
  if (sensor == nullptr) {
    return 1;
  }
  return sensor->uld_write_register(device_address, register_address, data, count);
}

int8_t vl53l1x_platform_read_multi(uint8_t device_address, uint16_t register_address, uint8_t *data, uint32_t count) {
  auto *sensor = VL53L1X::get_active_sensor();
  if (sensor == nullptr) {
    return 1;
  }
  return sensor->uld_read_register(device_address, register_address, data, count);
}

}  // namespace esphome::vl53l1x
