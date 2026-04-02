#pragma once

#include <cstdint>

namespace esphome::vl53l1x {

int8_t vl53l1x_platform_write_multi(uint8_t device_address, uint16_t register_address, const uint8_t *data,
                                    uint32_t count);
int8_t vl53l1x_platform_read_multi(uint8_t device_address, uint16_t register_address, uint8_t *data, uint32_t count);

}  // namespace esphome::vl53l1x
