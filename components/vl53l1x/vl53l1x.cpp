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
    roode::Roode::log_event("xshut_toggled");
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

#ifdef CONFIG_IDF_TARGET_ESP32
  // Force Arduino Wire to initialize on the configured pins to resolve IDF conflict
  ESP_LOGD(TAG, "Forcing Arduino Wire initialization on SDA=21, SCL=22");
  bool wire_ok = Wire.begin(21, 22);
  ESP_LOGD(TAG, "Wire.begin result: %s", wire_ok ? "success" : "failed");
#endif

  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(false);
      roode::Roode::log_event("xshut_sensor_" + std::to_string(s->sensor_id_) + "_off");
      roode::Roode::log_event("xshut_toggled_off");
      roode::Roode::log_event("xshut_toggled");
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
    roode::Roode::log_event("xshut_toggled");
    delay(200); // Increased for stability
  }

  if (this->interrupt_pin.has_value()) {
    this->interrupt_pin.value()->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
    this->interrupt_pin.value()->setup();
    ESP_LOGD(TAG, "Interrupt pin configured");
  }

  // Set a longer timeout for initial boot
  uint16_t original_timeout = this->timeout;
  this->timeout = 5000;

  auto status = this->init();

  // Restore original timeout
  this->timeout = original_timeout;

  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Sensor initialization failed with error code: %d", status);
    this->mark_failed();
  } else {
    ESP_LOGD(TAG, "Device initialized successfully");
  }

  // IMPORTANT: Power back up all other sensors regardless of success/failure
  for (auto *s : sensors) {
    if (s != this && s->xshut_pin.has_value()) {
      s->xshut_pin.value()->digital_write(true);
      roode::Roode::log_event("xshut_sensor_" + std::to_string(s->sensor_id_) + "_on");
      roode::Roode::log_event("xshut_toggled_on");
      roode::Roode::log_event("xshut_toggled");
      delay(50);
    }
  }

  if (this->is_failed()) return;

  if (desired_address_ != 0x29 && desired_address_ != address_) {
#ifdef CONFIG_IDF_TARGET_ESP32
    if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreTake(roode::Roode::i2c_mutex_, portMAX_DELAY);
#endif
    status = this->sensor.SetI2CAddress(desired_address_ << 1);
#ifdef CONFIG_IDF_TARGET_ESP32
    if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreGive(roode::Roode::i2c_mutex_);
#endif
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
#ifdef CONFIG_IDF_TARGET_ESP32
    if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreTake(roode::Roode::i2c_mutex_, portMAX_DELAY);
#endif
    status = this->sensor.SetOffsetInMm(this->offset.value());
#ifdef CONFIG_IDF_TARGET_ESP32
    if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreGive(roode::Roode::i2c_mutex_);
#endif
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set offset calibration, error code: %d", status);
      this->mark_failed();
      return;
    }
  }

  if (this->xtalk.has_value()) {
    ESP_LOGI(TAG, "Setting crosstalk calibration to %d", this->xtalk.value());
#ifdef CONFIG_IDF_TARGET_ESP32
    if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreTake(roode::Roode::i2c_mutex_, portMAX_DELAY);
#endif
    status = this->sensor.SetXTalk(this->xtalk.value());
#ifdef CONFIG_IDF_TARGET_ESP32
    if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreGive(roode::Roode::i2c_mutex_);
#endif
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

  ESP_LOGI(TAG, "Setup complete");
}

VL53L1_Error VL53L1X::init() {
  ESP_LOGD(TAG, "Trying to initialize at address 0x%02X", address_);

  VL53L1_Error status;

#ifdef CONFIG_IDF_TARGET_ESP32
  if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreTake(roode::Roode::i2c_mutex_, portMAX_DELAY);
#endif
  uint16_t current_addr = sensor.GetI2CAddress() >> 1;
#ifdef CONFIG_IDF_TARGET_ESP32
  if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreGive(roode::Roode::i2c_mutex_);
#endif

  ESP_LOGD(TAG, "Current ULD driver address: 0x%02X", current_addr);

  // Emergency re-addressing: if we expect 0x66 but sensor is at 0x29
  if (address_ != current_addr) {
    ESP_LOGD(TAG, "Address mismatch. Checking if sensor is at 0x29...");
#ifdef CONFIG_IDF_TARGET_ESP32
    if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreTake(roode::Roode::i2c_mutex_, portMAX_DELAY);
#endif
    status = sensor.SetI2CAddress(0x29 << 1); // Try to talk to 0x29
    if (status == VL53L1_ERROR_NONE) {
       ESP_LOGI(TAG, "Sensor found at 0x29. Moving to 0x%02X...", address_);
       status = sensor.SetI2CAddress(address_ << 1);
    }
#ifdef CONFIG_IDF_TARGET_ESP32
    if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreGive(roode::Roode::i2c_mutex_);
#endif
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGW(TAG, "Could not find sensor at 0x%02X or 0x29", address_);
    }
  }

  status = wait_for_boot();
  if (status != VL53L1_ERROR_NONE) {
    return status;
  }

  ESP_LOGD(TAG, "Found device, initializing...");
#ifdef CONFIG_IDF_TARGET_ESP32
  if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreTake(roode::Roode::i2c_mutex_, portMAX_DELAY);
#endif
  status = sensor.Init();
#ifdef CONFIG_IDF_TARGET_ESP32
  if (roode::Roode::i2c_mutex_ != nullptr) xSemaphoreGive(roode::Roode::i2c_mutex_);
#endif
  return status;
}

VL53L1_Error VL53L1X::wait_for_boot() {
  // Wait for firmware to copy NVM device_state into registers
  delayMicroseconds(1200);

  uint8_t device_state = 0;
  VL53L1_Error status;
  auto start = millis();
  ESP_LOGD(TAG, "Waiting for boot, timeout: %dms", timeout);
  while ((millis() - start) < this->timeout) {
    status = get_device_state(&device_state);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "get_device_state failed: %d", status);
      return status;
    }
    if ((device_state & 0x01) == 0x01) {
      ESP_LOGD(TAG, "Finished waiting for boot. Device state: %d", device_state);
      return VL53L1_ERROR_NONE;
    }
    App.feed_wdt();
    delay(1);
  }

  ESP_LOGW(TAG, "Timed out waiting for boot. Current state: %d", device_state);
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
    record_failure();
    return {};
  }

  ESP_LOGVV(TAG, "Beginning distance read");

  if (last_roi == nullptr || *roi != *last_roi) {
    ESP_LOGVV(TAG, "Setting new ROI: { width: %d, height: %d, center: %d }", roi->width, roi->height, roi->center);

    status = this->sensor.SetROI(roi->width, roi->height);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI width/height, error code: %d", status);
      record_failure();
      return {};
    }
    status = this->sensor.SetROICenter(roi->center);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI center, error code: %d", status);
      record_failure();
      return {};
    }
    last_roi = roi;
  }

  // Decide whether we can use the interrupt pin for this reading
  uint8_t dataReady = false;
  bool use_int = is_interrupt_enabled();
  if (!use_int && this->interrupt_pin.has_value() &&
      (millis() - last_interrupt_retry_ >= 1800000UL)) {
    if (validate_interrupt()) {
      interrupt_active_ = true;
      interrupt_miss_count_ = 0;
      roode::Roode::log_event("interrupt_recovered");
      use_int = true;
    } else {
      last_interrupt_retry_ = millis();
    }
  }

  status = this->sensor.StartRanging();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to start ranging, error code: %d", status);
    record_failure();
    return {};
  }

  // Wait for measurement ready using interrupt pin when available
  bool initial_state = false;
  if (use_int) {
    initial_state = this->interrupt_pin.value()->digital_read();
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
        record_failure();
        return {};
      }
    }
    delay(1);
    App.feed_wdt();
  }
  if (use_int && !dataReady) {
    roode::Roode::log_event("int_pin_missed_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("int_pin_missed");
    interrupt_miss_count_++;
    if (interrupt_miss_count_ >= 5) {
      roode::Roode::log_event("interrupt_fallback_polling");
      interrupt_active_ = false;
      last_interrupt_retry_ = millis();
    } else {
      roode::Roode::log_event("interrupt_fallback");
    }
    // Fallback to polling for this measurement
    start_time = millis();
    while (!dataReady && (millis() - start_time) < this->timeout) {
      status = this->sensor.CheckForDataReady(&dataReady);
      if (status != VL53L1_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to check if data is ready, error code: %d", status);
        record_failure();
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
      roode::Roode::log_event("xshut_pulse_off");
      ESP_LOGW(TAG, "XShut pin set LOW - resetting sensor");
      delay(100);
      this->xshut_pin.value()->digital_write(true);
      roode::Roode::log_event("xshut_reinitialize_sensor_" + std::to_string(sensor_id_));
      roode::Roode::log_event("xshut_reinitialize");
      ESP_LOGD(TAG, "XShut pin set HIGH - reset complete");
      this->wait_for_boot();
      roode::Roode::log_event("sensor_" + std::to_string(sensor_id_) + ".recovered_via_xshut");
      roode::Roode::log_event("sensor.recovered_via_xshut");
      recovery_count_++;
    }
    record_failure();
    return {};
  }

  // Get the results
  uint16_t distance;
  status = this->sensor.GetDistanceInMm(&distance);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not get distance, error code: %d", status);
    record_failure();
    return {};
  }

  // After reading the results reset the interrupt to be able to take another measurement
  status = this->sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not clear interrupt, error code: %d", status);
    record_failure();
    return {};
  }
  status = this->sensor.StopRanging();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not stop ranging, error code: %d", status);
    record_failure();
    return {};
  }

  if (use_int)
    interrupt_miss_count_ = 0;

  ESP_LOGV(TAG, "Finished distance read: %d", distance);
  consecutive_failures_ = 0;
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
    int_ok = validate_interrupt();
    if (!int_ok) {
      ESP_LOGE(TAG, "Interrupt pin validation failed, falling back to polling");
      interrupt_active_ = false;
      interrupt_miss_count_ = 0;
      last_interrupt_retry_ = millis();
    } else {
      ESP_LOGI(TAG, "Interrupt pin working");
      interrupt_active_ = true;
      roode::Roode::log_event("interrupt_initialized");
      interrupt_miss_count_ = 0;
    }
  } else {
    interrupt_active_ = false;
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

bool VL53L1X::validate_interrupt() {
  bool ok = false;
  if (!this->interrupt_pin.has_value())
    return false;
  bool initial = this->interrupt_pin.value()->digital_read();
  ESP_LOGD(TAG, "Interrupt pin initial state: %d", initial);
  auto status = this->sensor.StartRanging();
  if (status == VL53L1_ERROR_NONE) {
    auto start = millis();
    while ((millis() - start) < this->timeout) {
      if (this->interrupt_pin.value()->digital_read() != initial) {
        ESP_LOGD(TAG, "Interrupt pin state changed - measurement ready");
        ok = true;
        break;
      }
      delay(1);
      App.feed_wdt();
    }
    if (!ok)
      ESP_LOGD(TAG, "Interrupt pin did not change state during validation");
    this->sensor.ClearInterrupt();
    this->sensor.StopRanging();
  }
  return ok;
}

void VL53L1X::restart() {
  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    roode::Roode::log_event("xshut_pulse_off_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("xshut_pulse_off");
    ESP_LOGW(TAG, "XShut pin set LOW - restarting sensor");
    delay(100);
    this->xshut_pin.value()->digital_write(true);
    roode::Roode::log_event("xshut_reinitialize_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("xshut_reinitialize");
    ESP_LOGD(TAG, "XShut pin set HIGH - restart complete");
    this->wait_for_boot();
    roode::Roode::log_event("sensor_" + std::to_string(sensor_id_) + ".recovered_via_xshut");
    roode::Roode::log_event("sensor.recovered_via_xshut");
    recovery_count_++;
  } else {
    ESP_LOGW(TAG, "Restarting sensor without XSHUT pin");
    this->init();
  }
}

void VL53L1X::soft_reset() {

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    roode::Roode::log_event("xshut_pulse_off_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("xshut_pulse_off");

    ESP_LOGW(TAG, "XShut pin set LOW - resetting sensor");

    delay(100);
    this->xshut_pin.value()->digital_write(true);
    roode::Roode::log_event("xshut_reinitialize_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("xshut_reinitialize");

    ESP_LOGD(TAG, "XShut pin set HIGH - reset complete");

    this->wait_for_boot();
    roode::Roode::log_event("sensor_" + std::to_string(sensor_id_) + ".recovered_via_xshut");
    roode::Roode::log_event("sensor.recovered_via_xshut");
    recovery_count_++;
  } else {
    ESP_LOGW(TAG, "Restarting sensor without XSHUT pin");
    this->init();
  }
}


void VL53L1X::record_failure() {
  if (++consecutive_failures_ >= 10) {
    roode::Roode::log_event("10 read errors — triggering recovery");
    ESP_LOGW(TAG, "10 read errors — triggering recovery");
    soft_reset();
    consecutive_failures_ = 0;
  }
}


optional<uint16_t> VL53L1X::get_signal_rate() {
  uint16_t signal_rate;
  auto status = this->sensor.GetSignalRate(&signal_rate);
  if (status != VL53L1_ERROR_NONE) {
    return {};
  }
  return {signal_rate};
}

}  // namespace vl53l1x
}  // namespace esphome
