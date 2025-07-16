#pragma once
#include <math.h>

#include "VL53L1X_ULD.h"
#include <vector>
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "ranging.h"
#include "roi.h"

namespace esphome {
namespace vl53l1x {
static const char *const TAG = "VL53L1X";

/**
 * A wrapper for the VL53L1X, Time-of-Flight (ToF), laser-ranging sensor.
 * This stores user calibration info.
 */
class VL53L1X : public i2c::I2CDevice, public Component {
 public:
  void setup() override;
  void dump_config() override;
  ~VL53L1X();
  /** This connects directly to a sensor */
  float get_setup_priority() const override { return setup_priority::DATA; };

  optional<uint16_t> read_distance(ROI *roi, VL53L1_Error &error);
  void set_ranging_mode(const RangingMode *mode);

  optional<bool> get_xshut_state() {
    if (this->xshut_pin.has_value())
      return this->xshut_pin.value()->digital_read();
    return {};
  }
  optional<bool> get_interrupt_state() {
    if (this->interrupt_pin.has_value())
      return this->interrupt_pin.value()->digital_read();
    return {};
  }
  int get_recovery_count() const { return recovery_count_; }
  void set_sensor_id(uint8_t id) { sensor_id_ = id; }
  void set_desired_address(uint8_t addr) { desired_address_ = addr; }
  void restart();

  void set_xshut_pin(GPIOPin *pin) { this->xshut_pin = pin; }
  void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin = pin; }
  optional<const RangingMode *> get_ranging_mode_override() { return this->ranging_mode_override; }
  void set_ranging_mode_override(const RangingMode *mode) { this->ranging_mode_override = {mode}; }
  void set_offset(int16_t val) { this->offset = val; }
  void set_xtalk(uint16_t val) { this->xtalk = val; }
  void set_timeout(uint16_t val) { this->timeout = val; }

  bool is_interrupt_enabled() const { return interrupt_active_ && interrupt_pin.has_value(); }

 protected:
  VL53L1X_ULD sensor;
  optional<GPIOPin *> xshut_pin{};
  optional<InternalGPIOPin *> interrupt_pin{};
  const RangingMode * ranging_mode{};
  /** Mode from user config, which can be get/set independently of current mode */
  optional<const RangingMode *> ranging_mode_override{};
  optional<int16_t> offset{};
  optional<uint16_t> xtalk{};
  uint16_t timeout{};
 ROI *last_roi{};
 int recovery_count_{0};
  uint8_t sensor_id_{0};
  uint8_t desired_address_{0x29};
  static std::vector<VL53L1X *> sensors;

  VL53L1_Error init();
  VL53L1_Error wait_for_boot();
  VL53L1_Error get_device_state(uint8_t *device_state);
  /**
   * Validate optional pins and log their status.
   *
   * @return false when the sensor becomes unresponsive while testing pins.
   */
  bool check_features();
  bool validate_interrupt();

  void soft_reset();
  void record_failure();

  bool interrupt_active_{false};
  uint8_t interrupt_miss_count_{0};
  uint32_t last_interrupt_retry_{0};
  uint8_t consecutive_failures_{0};
};

}  // namespace vl53l1x
}  // namespace esphome

