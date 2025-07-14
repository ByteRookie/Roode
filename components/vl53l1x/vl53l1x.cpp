#include "vl53l1x.h"
#include "../roode/roode.h"
#include <cstdio>

namespace esphome {
namespace vl53l1x {

std::vector<VL53L1X *> VL53L1X::sensors{};

VL53L1X::~VL53L1X() {
  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    ESP_LOGD(TAG, "XShut pin set LOW - powering down sensor");
    roode::Roode::log_event("xshut_sensor_" + std::to_string(sensor_id_) + "_off");
    roode::Roode::log_event("xshut_toggled_off");
  }
  this->sensor.StopRanging();
}

void VL53L1X::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L1X:");
  LOG_I2C_DEVICE(this);
  if (this->ranging_mode != nullptr) {
    ESP_LOGCONFIG(TAG, "  Ranging: %s", this->ranging_mode->name);
  }
  if (offset.has_value()) {
    ESP_LOGCONFIG(TAG, "  Offset: %dmm", this->offset.value());
  }
  if (xtalk.has_value()) {
    ESP_LOGCONFIG(TAG, "  XTalk: %dcps", this->xtalk.value());
  }
  LOG_PIN("  Interrupt Pin: ", this->interrupt_pin.value());
  LOG_PIN("  XShut Pin: ", this->xshut_pin.value());
}

void VL53L1X::setup() {
  ESP_LOGD(TAG, "Beginning setup");

  sensors.push_back(this);
  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(false);
      roode::Roode::log_event("xshut_sensor_" + std::to_string(s->sensor_id_) + "_off");
      roode::Roode::log_event("xshut_toggled_off");
    }
  }

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->pin_mode(gpio::FLAG_OUTPUT | gpio::FLAG_PULLUP);
    this->xshut_pin.value()->setup();
    ESP_LOGD(TAG, "XShut pin configured");
    this->xshut_pin.value()->digital_write(true);
    ESP_LOGD(TAG, "XShut pin set HIGH - sensor powered on");
    roode::Roode::log_event("xshut_sensor_" + std::to_string(sensor_id_) + "_on");
    roode::Roode::log_event("xshut_toggled_on");
    delay(2);
  }

  if (this->interrupt_pin.has_value()) {
    this->interrupt_pin.value()->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
    this->interrupt_pin.value()->setup();
    ESP_LOGD(TAG, "Interrupt pin configured");
  }

  auto status = this->init();
  if (status != VL53L1_ERROR_NONE) {
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Device initialized");
  if (desired_address_ != 0x29) {
    status = this->sensor.SetI2CAddress(desired_address_ << 1);
    if (status == VL53L1_ERROR_NONE) {
      char buf[5];
      snprintf(buf, sizeof(buf), "%02X", desired_address_);
      roode::Roode::log_event("sensor_" + std::to_string(sensor_id_) + "_addr = 0x" + std::string(buf));
    } else {
      ESP_LOGE(TAG, "Failed to change address. Error: %d", status);
    }
  }

  if (this->offset.has_value()) {
    ESP_LOGI(TAG, "Setting offset calibration to %d", this->offset.value());
    status = this->sensor.SetOffsetInMm(this->offset.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set offset calibration, error code: %d", status);
      this->mark_failed();
      return;
    }
  }

  if (this->xtalk.has_value()) {
    ESP_LOGI(TAG, "Setting crosstalk calibration to %d", this->xtalk.value());
    status = this->sensor.SetXTalk(this->xtalk.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set crosstalk calibration, error code: %d", status);
      this->mark_failed();
      return;
    }
  }

  if (!this->check_features()) {
    ESP_LOGE(TAG, "Feature check failed. Sensor disabled");
    return;
  }

  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(true);
      roode::Roode::log_event("xshut_sensor_" + std::to_string(s->sensor_id_) + "_on");
      roode::Roode::log_event("xshut_toggled_on");
      delay(2);
    }
  }

  ESP_LOGI(TAG, "Setup complete");
}

VL53L1_Error VL53L1X::init() {
  ESP_LOGD(TAG, "Trying to initialize");

  VL53L1_Error status;

  // If address is non-default, set and try again.
  if (address_ != (sensor.GetI2CAddress() >> 1)) {
    ESP_LOGD(TAG, "Setting different address");
    status = sensor.SetI2CAddress(address_ << 1);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Failed to change address. Error: %d", status);
      return status;
    }
  }

  status = wait_for_boot();
  if (status != VL53L1_ERROR_NONE) {
    return status;
  }

  ESP_LOGD(TAG, "Found device, initializing...");
  status = sensor.Init();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not initialize device, error code: %d", status);
    return status;
  }

  return status;
}

VL53L1_Error VL53L1X::wait_for_boot() {
  // Wait for firmware to copy NVM device_state into registers
  delayMicroseconds(1200);

  uint8_t device_state;
  VL53L1_Error status;
  auto start = millis();
  while ((millis() - start) < this->timeout) {
    status = get_device_state(&device_state);
    if (status != VL53L1_ERROR_NONE) {
      return status;
    }
    if ((device_state & 0x01) == 0x01) {
      ESP_LOGD(TAG, "Finished waiting for boot. Device state: %d", device_state);
      return VL53L1_ERROR_NONE;
    }
    App.feed_wdt();
  }

  ESP_LOGW(TAG, "Timed out waiting for boot. state: %d", device_state);
  return VL53L1_ERROR_TIME_OUT;
}

VL53L1_Error VL53L1X::get_device_state(uint8_t *device_state) {
  VL53L1_Error status = sensor.GetBootState(device_state);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to read device state. error: %d", status);
    return status;
  }

  // Our own logic...device_state is 255 when unable to complete read
  // Not sure why and why other libraries don't account for this.
  // Maybe somehow this is supposed to be 0, and it is getting messed up in I2C layer.
  if (*device_state == 255) {
    *device_state = 98;  // Unknown
  }

  ESP_LOGV(TAG, "Device state: %d", *device_state);

  return VL53L1_ERROR_NONE;
}

void VL53L1X::set_ranging_mode(const RangingMode *mode) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot set ranging mode while component is failed");
    return;
  }

  auto status = this->sensor.SetDistanceMode(mode->mode);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set distance mode: %d, error code: %d", mode->mode, status);
  }

  status = this->sensor.SetTimingBudgetInMs(mode->timing_budget);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set timing budget: %d ms, error code: %d", mode->timing_budget, status);
  }

  status = this->sensor.SetInterMeasurementInMs(mode->delay_between_measurements);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set measurement delay: %d ms, error code: %d", mode->delay_between_measurements, status);
  }

  this->ranging_mode = mode;
  ESP_LOGI(TAG, "Set ranging mode: %s", mode->name);
}

optional<uint16_t> VL53L1X::read_distance(ROI *roi, VL53L1_Error &status) {
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Cannot read distance while component is failed");
    return {};
  }

  ESP_LOGVV(TAG, "Beginning distance read");

  if (last_roi == nullptr || *roi != *last_roi) {
    ESP_LOGVV(TAG, "Setting new ROI: { width: %d, height: %d, center: %d }", roi->width, roi->height, roi->center);

    status = this->sensor.SetROI(roi->width, roi->height);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI width/height, error code: %d", status);
      return {};
    }
    status = this->sensor.SetROICenter(roi->center);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI center, error code: %d", status);
      return {};
    }
    last_roi = roi;
  }

  status = this->sensor.StartRanging();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to start ranging, error code: %d", status);
    return {};
  }

  // Wait for measurement ready using interrupt pin when available
  uint8_t dataReady = false;
  bool use_int = this->interrupt_pin.has_value();
  bool initial_state = false;
  if (use_int) {
    initial_state = this->interrupt_pin.value()->digital_read();
    roode::Roode::log_event("use_interrupt_mode_" + std::string(initial_state ? "high" : "low"));
  }
  auto start_time = millis();
  while (!dataReady && (millis() - start_time) < this->timeout) {
    if (use_int) {
      if (this->interrupt_pin.value()->digital_read() != initial_state) {
        dataReady = true;
      }
    } else {
      status = this->sensor.CheckForDataReady(&dataReady);
      if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to check if data is ready, error code: %d", status);
        return {};
      }
    }
    delay(1);
    App.feed_wdt();
  }
  if (use_int && !dataReady) {
    roode::Roode::log_event("int_pin_missed_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("interrupt_fallback_polling");
    // Fallback to polling for this measurement
    start_time = millis();
    while (!dataReady && (millis() - start_time) < this->timeout) {
      status = this->sensor.CheckForDataReady(&dataReady);
      if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to check if data is ready, error code: %d", status);
        return {};
      }
      delay(1);
      App.feed_wdt();
    }
  }
  if (!dataReady) {
    ESP_LOGW(TAG, "Timed out waiting for measurement ready");
    status = VL53L1_ERROR_TIME_OUT;
    this->sensor.StopRanging();
    if (this->xshut_pin.has_value()) {
      this->xshut_pin.value()->digital_write(false);
      roode::Roode::log_event("xshut_pulse_off_sensor_" + std::to_string(sensor_id_));
      ESP_LOGW(TAG, "XShut pin set LOW - resetting sensor");
      delay(100);
      this->xshut_pin.value()->digital_write(true);
      roode::Roode::log_event("xshut_reinitialize_sensor_" + std::to_string(sensor_id_));
      ESP_LOGD(TAG, "XShut pin set HIGH - reset complete");
      this->wait_for_boot();
      roode::Roode::log_event("sensor_" + std::to_string(sensor_id_) + ".recovered_via_xshut");
      recovery_count_++;
    }
    return {};
  }

  // Get the results
  uint16_t distance;
  status = this->sensor.GetDistanceInMm(&distance);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not get distance, error code: %d", status);
    return {};
  }

  // After reading the results reset the interrupt to be able to take another measurement
  status = this->sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not clear interrupt, error code: %d", status);
    return {};
  }
  status = this->sensor.StopRanging();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not stop ranging, error code: %d", status);
    return {};
  }

  ESP_LOGV(TAG, "Finished distance read: %d", distance);
  return {distance};
}

bool VL53L1X::check_features() {
  ESP_LOGI(TAG, "Validating optional pins");
  bool xshut_ok = false;
  bool int_ok = false;

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    ESP_LOGD(TAG, "XShut pin set LOW - validating pin");
    delay(10);
    this->xshut_pin.value()->digital_write(true);
    ESP_LOGD(TAG, "XShut pin set HIGH - validation reset complete");
    xshut_ok = (this->wait_for_boot() == VL53L1_ERROR_NONE);
    if (!xshut_ok) {
      ESP_LOGE(TAG, "XShut pin validation failed, disabling power cycle support");
      this->xshut_pin.reset();
      ESP_LOGW(TAG, "XShut pin disabled due to validation failure");
      if (this->wait_for_boot() != VL53L1_ERROR_NONE) {
        this->mark_failed();
        return false;
      }
    } else {
      ESP_LOGI(TAG, "XShut pin working");
    }
  }

  if (!this->xshut_pin.has_value()) {
    ESP_LOGI(TAG, "XShut disabled");
  }

  if (this->interrupt_pin.has_value()) {
    bool initial = this->interrupt_pin.value()->digital_read();
    ESP_LOGD(TAG, "Interrupt pin initial state: %d", initial);
    auto status = this->sensor.StartRanging();
    if (status == VL53L1_ERROR_NONE) {
      auto start = millis();
      while ((millis() - start) < this->timeout) {
        if (this->interrupt_pin.value()->digital_read() != initial) {
          ESP_LOGD(TAG, "Interrupt pin state changed - measurement ready");
          int_ok = true;
          break;
        }
        App.feed_wdt();
      }
      if (!int_ok)
        ESP_LOGD(TAG, "Interrupt pin did not change state during validation");
      this->sensor.ClearInterrupt();
      this->sensor.StopRanging();
    }
    if (!int_ok) {
      ESP_LOGE(TAG, "Interrupt pin validation failed, falling back to polling");
      this->interrupt_pin.reset();
      ESP_LOGW(TAG, "Interrupt pin disabled due to validation failure");
    } else {
      ESP_LOGI(TAG, "Interrupt pin working");
    }
  }

  if (!this->interrupt_pin.has_value()) {
    ESP_LOGI(TAG, "Interrupt disabled");
  }

  if (this->xshut_pin.has_value()) {
    ESP_LOGI(TAG, "XShut %s", xshut_ok ? "working" : "disabled");
  }
  if (this->interrupt_pin.has_value()) {
    ESP_LOGI(TAG, "Interrupt %s", int_ok ? "working" : "disabled");
  }

  return !this->is_failed();
}

}  // namespace vl53l1x
}  // namespace esphome
