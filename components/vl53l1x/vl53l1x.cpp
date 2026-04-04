#include "vl53l1x.h"
#include "../roode/roode.h"
#include <array>
#include <cstring>
#include <cstdio>

namespace esphome {
namespace vl53l1x {

std::vector<VL53L1X *> VL53L1X::sensors{};
thread_local VL53L1X *VL53L1X::active_sensor_ = nullptr;

namespace {

class ScopedActiveSensor {
 public:
  explicit ScopedActiveSensor(VL53L1X *sensor) : previous_(VL53L1X::get_active_sensor()) { VL53L1X::set_active_sensor(sensor); }
  ~ScopedActiveSensor() { VL53L1X::set_active_sensor(previous_); }

 protected:
  VL53L1X *previous_;
};

}  // namespace

VL53L1X::~VL53L1X() {
  ScopedActiveSensor scoped(this);
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
  ScopedActiveSensor scoped(this);

  sensors.push_back(this);

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

  status = this->apply_runtime_configuration_();
  if (status != VL53L1_ERROR_NONE) {
    this->mark_failed();
    return;
  }

  if (!this->check_features()) {
    ESP_LOGE(TAG, "Feature check failed. Sensor disabled");
    return;
  }

  ESP_LOGI(TAG, "Setup complete");
}

int8_t VL53L1X::uld_write_register(uint8_t address, uint16_t register_address, const uint8_t *data, size_t len) {
  if (this->bus_ == nullptr || len > 64) {
    return 1;
  }
  uint8_t effective_addr = this->i2c_address_override_ ? this->i2c_address_override_ : address;
  std::array<uint8_t, 66> buffer{};
  buffer[0] = register_address >> 8;
  buffer[1] = register_address & 0xFF;
  if (len > 0) {
    memcpy(buffer.data() + 2, data, len);
  }
  return this->bus_->write_readv(effective_addr, buffer.data(), len + 2, nullptr, 0) == i2c::ERROR_OK ? 0 : 1;
}

int8_t VL53L1X::uld_read_register(uint8_t address, uint16_t register_address, uint8_t *data, size_t len) {
  if (this->bus_ == nullptr || len > 64) {
    return 1;
  }
  uint8_t effective_addr = this->i2c_address_override_ ? this->i2c_address_override_ : address;
  std::array<uint8_t, 2> buffer{{static_cast<uint8_t>(register_address >> 8), static_cast<uint8_t>(register_address & 0xFF)}};
  return this->bus_->write_readv(effective_addr, buffer.data(), buffer.size(), data, len) == i2c::ERROR_OK ? 0 : 1;
}

VL53L1_Error VL53L1X::init() {
  static constexpr uint8_t FACTORY_DEFAULT = 0x29;
  // Common relocated address used by older firmware versions. If the sensor was previously
  // moved here and the ESP restarted (without power-cycling the sensor), we need to find it.
  static constexpr uint8_t ALT_ADDR = 0x66;

  ESP_LOGD(TAG, "Initializing sensor, configured address: 0x%02X", address_);

  // Use the address override mechanism so we control exactly which I2C address the ULD
  // driver talks to, regardless of its internal cached state.
  VL53L1_Error status;
  uint8_t found_at = 0;

  // 1. Try the configured address first (covers: ESP restart with sensor keeping its address,
  //    or first boot when address_ == 0x29 and sensor is at factory default).
  i2c_address_override_ = address_;
  status = wait_for_boot();
  if (status == VL53L1_ERROR_NONE) {
    found_at = address_;
    ESP_LOGD(TAG, "Sensor found at configured address 0x%02X", address_);
  }

  // 2. Fall back to factory default 0x29 (covers: XShut reset resets sensor to 0x29,
  //    and address_ != 0x29 — the most common recovery scenario).
  if (found_at == 0 && address_ != FACTORY_DEFAULT) {
    ESP_LOGD(TAG, "Not found at 0x%02X, trying factory default 0x29", address_);
    i2c_address_override_ = FACTORY_DEFAULT;
    status = wait_for_boot();
    if (status == VL53L1_ERROR_NONE) {
      found_at = FACTORY_DEFAULT;
      ESP_LOGD(TAG, "Sensor found at factory default 0x29");
    }
  }

  // 3. If configured for 0x29 but not found, try 0x66 (covers: old firmware relocated
  //    the sensor to 0x66 and the ESP restarted without power-cycling the sensor).
  if (found_at == 0 && address_ == FACTORY_DEFAULT && ALT_ADDR != FACTORY_DEFAULT) {
    ESP_LOGD(TAG, "Not found at 0x29, trying alternate address 0x%02X", ALT_ADDR);
    i2c_address_override_ = ALT_ADDR;
    status = wait_for_boot();
    if (status == VL53L1_ERROR_NONE) {
      found_at = ALT_ADDR;
      ESP_LOGD(TAG, "Sensor found at alternate address 0x%02X", ALT_ADDR);
    }
  }

  if (found_at == 0) {
    i2c_address_override_ = 0;
    ESP_LOGE(TAG, "VL53L1X not found at 0x%02X, 0x29, or 0x%02X", address_, ALT_ADDR);
    return status;  // last non-VL53L1_ERROR_NONE status
  }

  // Initialize the sensor at the address where it was found.
  ESP_LOGD(TAG, "Initializing device at 0x%02X", found_at);
#ifdef CONFIG_IDF_TARGET_ESP32
  roode::Roode::i2c_lock();
#endif
  status = sensor.Init();
#ifdef CONFIG_IDF_TARGET_ESP32
  roode::Roode::i2c_unlock();
#endif
  if (status != VL53L1_ERROR_NONE) {
    i2c_address_override_ = 0;
    ESP_LOGE(TAG, "sensor.Init() failed at 0x%02X, error: %d", found_at, status);
    return status;
  }

  // If found at a different address than configured, relocate the sensor.
  // The override is still active (pointing at found_at).
  //
  // IMPORTANT: VL53L1X_ULD._i2c_address defaults to 0x52 (= 0x29 << 1).
  // SetI2CAddress() has an early-exit: "if (new_address == _i2c_address) return OK".
  // Calling SetI2CAddress(address_ << 1 = 0x52) would hit this guard and do nothing —
  // the sensor stays at found_at and the override gets cleared → all I/O fails with -13.
  //
  // Fix: first sync the ULD's cache to found_at by calling SetI2CAddress(found_at << 1).
  // That call won't early-exit (found_at<<1 != 0x52 when found_at=0x66).
  // It writes "stay at found_at" to the hardware (no-op), but advances _i2c_address to found_at<<1.
  // Now SetI2CAddress(address_ << 1) won't early-exit either, and actually relocates the sensor.
  if (found_at != address_) {
    ESP_LOGI(TAG, "Relocating sensor from 0x%02X to configured address 0x%02X", found_at, address_);
#ifdef CONFIG_IDF_TARGET_ESP32
    roode::Roode::i2c_lock();
#endif
    // Step 1: sync ULD cache → found_at (override still = found_at, so writes go to found_at)
    sensor.SetI2CAddress(found_at << 1);
    // Step 2: now relocate the sensor from found_at to address_
    status = sensor.SetI2CAddress(address_ << 1);
#ifdef CONFIG_IDF_TARGET_ESP32
    roode::Roode::i2c_unlock();
#endif
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Failed to relocate sensor address, error: %d", status);
    } else {
      delay(2);  // Allow sensor time to switch addresses
      ESP_LOGD(TAG, "Sensor relocated to 0x%02X", address_);
    }
  }

  i2c_address_override_ = 0;
  return status;
}

VL53L1_Error VL53L1X::apply_runtime_configuration_() {
  VL53L1_Error status = VL53L1_ERROR_NONE;

#ifdef CONFIG_IDF_TARGET_ESP32
  roode::Roode::i2c_lock();
#endif
  if (this->offset.has_value()) {
    ESP_LOGI(TAG, "Setting offset calibration to %d", this->offset.value());
    status = this->sensor.SetOffsetInMm(this->offset.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set offset calibration, error code: %d", status);
      goto done;
    }
  }

  if (this->xtalk.has_value()) {
    ESP_LOGI(TAG, "Setting crosstalk calibration to %d", this->xtalk.value());
    status = this->sensor.SetXTalk(this->xtalk.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set crosstalk calibration, error code: %d", status);
      goto done;
    }
  }

  // Address relocation is handled in init() — skip here to avoid confusion.
  // Clear cached ROI so it is re-applied after re-initialization.
  has_last_roi_ = false;
  {
    // Use user-configured ranging mode, or fall back to Medium (33ms) for reasonable
    // out-of-the-box performance. Without this, the sensor defaults to ~100ms/measurement.
    const RangingMode *effective_mode = this->ranging_mode ? this->ranging_mode : Ranging::Medium;
    status = this->sensor.SetDistanceMode(effective_mode->mode);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set distance mode: %d, error code: %d", effective_mode->mode, status);
      goto done;
    }
    status = this->sensor.SetTimingBudgetInMs(effective_mode->timing_budget);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set timing budget: %d ms, error code: %d", effective_mode->timing_budget, status);
      goto done;
    }
    status = this->sensor.SetInterMeasurementInMs(effective_mode->delay_between_measurements);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set measurement delay: %d ms, error code: %d",
               effective_mode->delay_between_measurements, status);
      goto done;
    }
    if (!this->ranging_mode) {
      ESP_LOGI(TAG, "No ranging mode configured — using default Medium (33ms)");
    }
  }

done:
#ifdef CONFIG_IDF_TARGET_ESP32
  roode::Roode::i2c_unlock();
#endif
  return status;
}

VL53L1_Error VL53L1X::reinitialize_after_hard_reset_() {
  auto status = this->init();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Sensor reinitialization failed with error code: %d", status);
    return status;
  }

  status = this->apply_runtime_configuration_();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Restoring runtime configuration failed with error code: %d", status);
    return status;
  }

  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1X::wait_for_boot() {
  // Wait for firmware to copy NVM device_state into registers
  delayMicroseconds(1200);

  uint8_t device_state = 0;
  VL53L1_Error status = VL53L1_ERROR_CONTROL_INTERFACE;
  auto start = millis();
  ESP_LOGD(TAG, "Waiting for boot, timeout: %dms", timeout);
  while ((millis() - start) < this->timeout) {
    status = get_device_state(&device_state);
    if (status == VL53L1_ERROR_NONE && (device_state & 0x01) == 0x01) {
      ESP_LOGD(TAG, "Finished waiting for boot. Device state: %d", device_state);
      return VL53L1_ERROR_NONE;
    }
    App.feed_wdt();
    delay(1);
  }

  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGD(TAG, "Timed out waiting for boot (I2C error: %d)", status);
  } else {
    ESP_LOGW(TAG, "Timed out waiting for boot. Current state: %d", device_state);
    status = VL53L1_ERROR_TIME_OUT;
  }
  return status;
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
  ScopedActiveSensor scoped(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot set ranging mode while component is failed");
    return;
  }

#ifdef CONFIG_IDF_TARGET_ESP32
  roode::Roode::i2c_lock();
#endif
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
#ifdef CONFIG_IDF_TARGET_ESP32
  roode::Roode::i2c_unlock();
#endif
  ESP_LOGI(TAG, "Set ranging mode: %s", mode->name);
}

void VL53L1X::start_continuous_ranging() {
  continuous_mode_ = true;
}

void VL53L1X::stop_continuous_ranging() {
  continuous_mode_ = false;
}

optional<uint16_t> VL53L1X::read_distance(ROI *roi, VL53L1_Error &status) {
  ScopedActiveSensor scoped(this);
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!roode::Roode::i2c_lock()) return {};
#endif
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Cannot read distance while component is failed");
    record_failure();
#ifdef CONFIG_IDF_TARGET_ESP32
    roode::Roode::i2c_unlock();
#endif
    return {};
  }

  ESP_LOGVV(TAG, "Beginning distance read");

  if (!has_last_roi_ || *roi != last_roi_val_) {
    ESP_LOGV(TAG, "Applying ROI to sensor: { width: %d, height: %d, center: %d }", roi->width, roi->height, roi->center);

    status = this->sensor.SetROI(roi->width, roi->height);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI width/height, error code: %d", status);
      record_failure();
#ifdef CONFIG_IDF_TARGET_ESP32
      roode::Roode::i2c_unlock();
#endif
      return {};
    }
    status = this->sensor.SetROICenter(roi->center);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI center, error code: %d", status);
      record_failure();
#ifdef CONFIG_IDF_TARGET_ESP32
      roode::Roode::i2c_unlock();
#endif
      return {};
    }
    last_roi_val_ = *roi;
    has_last_roi_ = true;
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
#ifdef CONFIG_IDF_TARGET_ESP32
    roode::Roode::i2c_unlock();
#endif
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
#ifdef CONFIG_IDF_TARGET_ESP32
        roode::Roode::i2c_unlock();
#endif
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
#ifdef CONFIG_IDF_TARGET_ESP32
        roode::Roode::i2c_unlock();
#endif
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
      delay(100);
      this->xshut_pin.value()->digital_write(true);
      ESP_LOGW(TAG, "Sensor timeout — XShut reset triggered");
      auto recovery_status = this->reinitialize_after_hard_reset_();
      if (recovery_status == VL53L1_ERROR_NONE) {
        recovery_count_++;
        ESP_LOGD(TAG, "Sensor recovered via XShut (total recoveries: %d)", recovery_count_);
      } else {
        status = recovery_status;
      }
    }
    record_failure();
#ifdef CONFIG_IDF_TARGET_ESP32
    roode::Roode::i2c_unlock();
#endif
    return {};
  }

  // Get the results
  uint16_t distance;
  status = this->sensor.GetDistanceInMm(&distance);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not get distance, error code: %d", status);
    record_failure();
#ifdef CONFIG_IDF_TARGET_ESP32
    roode::Roode::i2c_unlock();
#endif
    return {};
  }

  ERangeStatus rs; this->sensor.GetRangeStatus(&rs);
  if (rs != RangeValid) {
    ESP_LOGV(TAG, "Range status: %d (distance: %dmm)", (int)rs, distance);
  }

  // After reading the results reset the interrupt to be able to take another measurement
  status = this->sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not clear interrupt, error code: %d", status);
    record_failure();
#ifdef CONFIG_IDF_TARGET_ESP32
    roode::Roode::i2c_unlock();
#endif
    return {};
  }
  status = this->sensor.StopRanging();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not stop ranging, error code: %d", status);
    record_failure();
#ifdef CONFIG_IDF_TARGET_ESP32
    roode::Roode::i2c_unlock();
#endif
    return {};
  }

  if (use_int)
    interrupt_miss_count_ = 0;

  ESP_LOGV(TAG, "Finished distance read: %dmm", distance);
  consecutive_failures_ = 0;
#ifdef CONFIG_IDF_TARGET_ESP32
  roode::Roode::i2c_unlock();
#endif
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
    xshut_ok = (this->reinitialize_after_hard_reset_() == VL53L1_ERROR_NONE);
    if (!xshut_ok) {
      ESP_LOGE(TAG, "XShut pin validation failed, disabling power cycle support");
      this->xshut_pin.reset();
      ESP_LOGW(TAG, "XShut pin disabled due to validation failure");
      if (this->reinitialize_after_hard_reset_() != VL53L1_ERROR_NONE) {
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
  ScopedActiveSensor scoped(this);
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
  ScopedActiveSensor scoped(this);
  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    roode::Roode::log_event("xshut_pulse_off_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("xshut_pulse_off");
    ESP_LOGW(TAG, "XShut pin set LOW - restarting sensor");
    delay(100);
    this->xshut_pin.value()->digital_write(true);
    delay(10); // Allow sensor time to stabilize
    roode::Roode::log_event("xshut_reinitialize_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("xshut_reinitialize");
    ESP_LOGD(TAG, "XShut pin set HIGH - restart complete");
    if (this->reinitialize_after_hard_reset_() == VL53L1_ERROR_NONE) {
      roode::Roode::log_event("sensor_" + std::to_string(sensor_id_) + ".recovered_via_xshut");
      roode::Roode::log_event("sensor.recovered_via_xshut");
      recovery_count_++;
    } else {
      this->mark_failed();
    }
  } else {
    ESP_LOGW(TAG, "Restarting sensor without XSHUT pin");
    if (this->reinitialize_after_hard_reset_() != VL53L1_ERROR_NONE) {
      this->mark_failed();
    }
  }
}

void VL53L1X::soft_reset() {
  ScopedActiveSensor scoped(this);

  if (this->xshut_pin.has_value()) {
    this->xshut_pin.value()->digital_write(false);
    roode::Roode::log_event("xshut_pulse_off_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("xshut_pulse_off");

    ESP_LOGW(TAG, "XShut pin set LOW - resetting sensor");

    delay(100);
    this->xshut_pin.value()->digital_write(true);
    delay(10); // Allow sensor time to stabilize
    roode::Roode::log_event("xshut_reinitialize_sensor_" + std::to_string(sensor_id_));
    roode::Roode::log_event("xshut_reinitialize");

    ESP_LOGD(TAG, "XShut pin set HIGH - reset complete");

    if (this->reinitialize_after_hard_reset_() == VL53L1_ERROR_NONE) {
      roode::Roode::log_event("sensor_" + std::to_string(sensor_id_) + ".recovered_via_xshut");
      roode::Roode::log_event("sensor.recovered_via_xshut");
      recovery_count_++;
    } else {
      this->mark_failed();
    }
  } else {
    ESP_LOGW(TAG, "Restarting sensor without XSHUT pin");
    if (this->reinitialize_after_hard_reset_() != VL53L1_ERROR_NONE) {
      this->mark_failed();
    }
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
  ScopedActiveSensor scoped(this);
  uint16_t signal_rate;
  auto status = this->sensor.GetSignalRate(&signal_rate);
  if (status != VL53L1_ERROR_NONE) {
    return {};
  }
  return {signal_rate};
}

}  // namespace vl53l1x
}  // namespace esphome
