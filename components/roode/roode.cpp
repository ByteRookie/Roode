#include "roode.h"
#include "select.h"
#include "setting_number.h"
#include "switch.h"
#include "esphome/core/hal.h"     // delay(), millis(), delayMicroseconds()
#include "esp_system.h"           // esp_restart()
#include "esp_heap_caps.h"        // heap_caps_get_*
#include "esp_flash.h"            // esp_flash_get_size()
#include "esp_ota_ops.h"          // esp_ota_get_running_partition()
#include "esp_chip_info.h"        // esp_chip_info()
#ifdef USE_ESP32
#include "esp_task_wdt.h"         // ESP32 task watchdog
#endif
#include <string>
#include <optional>
#include <vector>
#include <algorithm>
#include <ctime>

namespace esphome {
namespace roode {

static const char *filter_mode_to_str(FilterMode mode) {
  if (mode == FILTER_MEDIAN) return "median";
  if (mode == FILTER_PERCENTILE10) return "percentile10";
  return "min";
}

// When disabled, fallback diagnostics are omitted from the log to reduce noise.
bool Roode::log_fallback_events_ = false;
Roode *Roode::instance_ = nullptr;
void Roode::log_event(const std::string &msg) {
  if (!log_fallback_events_) {
    if (msg == "interrupt_fallback" || msg == "interrupt_fallback_polling")
      return;
    if (msg == "int_pin_missed" || msg.rfind("int_pin_missed_sensor_", 0) == 0)
      return;
    if (msg == "xshut_toggled" || msg == "xshut_toggled_on" || msg == "xshut_toggled_off" || msg == "xshut_pulse_off" ||
        msg == "xshut_reinitialize" || msg == "sensor.recovered_via_xshut" || msg.rfind("xshut_sensor_", 0) == 0 ||
        msg.rfind("xshut_pulse_off_sensor_", 0) == 0 || msg.rfind("xshut_reinitialize_sensor_", 0) == 0 ||
        (msg.rfind("sensor_", 0) == 0 && msg.find(".recovered_via_xshut") != std::string::npos))
      return;
  }

  static uint32_t last_int_log = 0;
  if (msg == "interrupt_fallback" || msg == "interrupt_fallback_polling" || msg == "int_pin_missed" ||
      msg.rfind("int_pin_missed_sensor_", 0) == 0) {
    uint32_t now = millis();
    if (last_int_log != 0 && (now - last_int_log) < 5000)
      return;
    last_int_log = now;
  }

  std::string out = msg;
  if (msg == "use_dual_core")
    out += " - launching task on core 1";
  else if (msg.rfind("retry_multicore_", 0) == 0)
    out += " - retry creating task";
  else if (msg == "dual_core_success")
    out += " - task running on core 1";
  else if (msg == "dual_core_failed")
    out += " - task creation failed";
  else if (msg == "fallback_single_core")
    out += " - switching to single core";
  else if (msg == "force_single_core")
    out += " - single core forced by config";
  else if (msg.rfind("xshut_sensor_", 0) == 0) {
    bool on = msg.find("_on") != std::string::npos;
    size_t start = sizeof("xshut_sensor_") - 1;
    size_t end = msg.find('_', start);
    std::string id = msg.substr(start, end - start);
    out += on ? " - sensor " + id + " ON" : " - sensor " + id + " OFF";
  } else if (msg == "xshut_toggled_on")
    out += " - XSHUT pin HIGH";
  else if (msg == "xshut_toggled_off")
    out += " - XSHUT pin LOW";
  else if (msg == "xshut_toggled")
    out += " - XSHUT pin toggled";
  else if (msg.rfind("xshut_pulse_off_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("xshut_pulse_off_sensor_") - 1);
    out += " - pulsing LOW for sensor " + id;
  } else if (msg == "xshut_pulse_off") {
    out += " - pulsing LOW";
  } else if (msg.rfind("xshut_reinitialize_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("xshut_reinitialize_sensor_") - 1);
    out += " - reinitializing sensor " + id;
  } else if (msg == "xshut_reinitialize") {
    out += " - reinitializing";
  } else if (msg.rfind("sensor_", 0) == 0 && msg.find("_addr") != std::string::npos) {
    size_t start = sizeof("sensor_") - 1;
    size_t end = msg.find('_', start);
    std::string id = msg.substr(start, end - start);
    std::string addr = msg.substr(msg.find("0x") + 2);
    out += " - address 0x" + addr + " for sensor " + id;
  } else if (msg.rfind("sensor_", 0) == 0 && msg.find(".recovered_via_xshut") != std::string::npos) {
    std::string id = msg.substr(sizeof("sensor_") - 1, msg.find('.') - (sizeof("sensor_") - 1));
    out += " - sensor " + id + " recovered via XSHUT";
  } else if (msg == "sensor.recovered_via_xshut") {
    out += " - sensor recovered via XSHUT";
  } else if (msg == "interrupt_fallback_polling" || msg == "interrupt_fallback")
    out += " - INT pin timeout, polling";
  else if (msg == "int_pin_missed")
    out += " - INT miss";
  else if (msg.rfind("int_pin_missed_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("int_pin_missed_sensor_") - 1);
    out += " - INT miss sensor " + id;
  } else if (msg.rfind("manual_adjust", 0) == 0)
    out += " - user corrected";
  const char *color = "\033[32m";  // green by default
  if (msg.find("fail") != std::string::npos || msg.find("fallback") != std::string::npos ||
      msg.find("missed") != std::string::npos)
    color = "\033[31m";  // red for errors
  else if (msg.find("retry") != std::string::npos || msg.find("manual_adjust") != std::string::npos ||
           msg.find("reinitialize") != std::string::npos || msg.find("pulse_off") != std::string::npos)
    color = "\033[33m";  // yellow for informational

  std::string colored = std::string(color) + out + "\033[0m";
  ESP_LOGI(TAG, "%s", colored.c_str());
  if (instance_ != nullptr) {
    if (msg.find("reinitialize") != std::string::npos) {
      instance_->update_status_text("reinitializing");
    } else if (msg.find("recovered_via_xshut") != std::string::npos) {
      instance_->update_status_text("ok");
    }
    if (msg == "dual_core_success" || msg == "fallback_single_core" || msg == "force_single_core" ||
        msg == "interrupt_fallback_polling" || msg == "interrupt_recovered") {
      instance_->publish_feature_list();
    }
  }
}

Roode::~Roode() {
  delete entry;
  delete exit;
}
void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Sample size: %d", samples);
  LOG_UPDATE_INTERVAL(this);
  entry->dump_config();
  exit->dump_config();
}

void Roode::setup() {
  ESP_LOGI(SETUP, "Booting Roode %s", VERSION);
  if (version_sensor != nullptr) {
    version_sensor->publish_state(VERSION);
  }
  ESP_LOGI(SETUP, "Using sampling with sampling size: %d", samples);

  if (this->distanceSensor->is_failed()) {
    this->mark_failed();
    update_status_text("offline");
    ESP_LOGE(TAG, "Roode cannot be setup without a valid VL53L1X sensor");
    return;
  }

  // Initialize filtering options before calibrating so threshold sampling uses
  // the configured window and mode
  entry->set_filter_window(filter_window_);
  entry->set_filter_mode(filter_mode_);
  exit->set_filter_window(filter_window_);
  exit->set_filter_mode(filter_mode_);

  // Restore all runtime settings (filter mode, sampling, thresholds, ROI, ranging mode, direction)
  // from flash before calibration so they are applied before the first measurement.
  restore_settings_from_flash();

  if (calibration_persistence_) {
    calibration_prefs_[0] = global_preferences->make_preference<CalibrationPrefs>(0xA0);
    calibration_prefs_[1] = global_preferences->make_preference<CalibrationPrefs>(0xA1);
    bool loaded = true;
    for (int i = 0; i < 2; i++) {
      if (calibration_prefs_[i].load(&calibration_data_[i])) {
        Zone *z = i == 0 ? entry : exit;
        z->threshold->idle = calibration_data_[i].baseline_mm;
        z->threshold->min = calibration_data_[i].threshold_min_mm;
        z->threshold->max = calibration_data_[i].threshold_max_mm;
        // Sanity-check the loaded calibration before trusting it.
        // If max >= idle the zone triggers on any sub-idle noise reading (the
        // specific failure seen when min_pct=1% / max_pct=100% was persisted),
        // causing hundreds of false detections per minute.
        if (z->threshold->idle < 200 || z->threshold->idle > 4000 ||
            z->threshold->max >= z->threshold->idle ||
            z->threshold->min < (z->threshold->idle * 2) / 100 ||
            z->threshold->max < z->threshold->min + 100) {
          ESP_LOGW(SETUP,
                   "Rejecting invalid calibration for zone %d: idle=%d min=%d max=%d — forcing recalibration",
                   i, z->threshold->idle, z->threshold->min, z->threshold->max);
          loaded = false;
          break;
        }
        int valid_count = 0;
        for (int s = 0; s < 5; s++) {
          z->readDistance(distanceSensor);
          if (abs((int) z->getDistance() - (int) z->threshold->idle) < (z->threshold->idle * 0.1))
            valid_count++;
        }
        if (valid_count < 5) {
          loaded = false;
          break;
        }
      } else {
        loaded = false;
        break;
      }
    }
    if (loaded) {
      entry->reset_roi(orientation_ == Parallel ? 167 : 195);
      exit->reset_roi(orientation_ == Parallel ? 231 : 60);
      entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
      exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
      auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
      distanceSensor->set_ranging_mode(mode);
      publish_sensor_configuration(entry, exit, true);
      publish_sensor_configuration(entry, exit, false);
      // Validation reads above used the pre-ROI wide aperture — clear those samples
      // so the first detections use fresh readings with the calibrated ROI.
      entry->reset_samples();
      exit->reset_samples();
    } else {
      calibrate_zones();
    }
  } else {
    calibrate_zones();
  }
  last_calibration_ts_ =
      std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
#ifdef USE_ESP32
  if (!force_single_core_) {
    log_event("use_dual_core");
    vTaskDelay(pdMS_TO_TICKS(200));
    BaseType_t res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 8192, this, 1, &sensor_task_handle_, 1);
    multicore_retry_count_ = 0;
    while (res != pdPASS && multicore_retry_count_ < 2) {
      multicore_retry_count_++;
      log_event(std::string("retry_multicore_") + std::to_string(multicore_retry_count_));
      vTaskDelay(pdMS_TO_TICKS(200));
      res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 8192, this, 1, &sensor_task_handle_, 1);
    }
    if (res == pdPASS) {
      use_sensor_task_ = true;
      log_event("dual_core_success");
    } else {
      log_event("dual_core_failed");
      log_event("fallback_single_core");
      use_sensor_task_ = false;
    }
  } else {
    use_sensor_task_ = false;
    log_event("force_single_core");
  }
#else
  use_sensor_task_ = false;
#endif
  loop_window_start_ = millis();
  loop_time_sum_ = 0;
  loop_count_ = 0;
  if (status_sensor != nullptr)
    status_sensor->publish_state(sensor_status);

  if (loop_time_sensor != nullptr)
    loop_time_sensor->publish_state(0);
  if (cpu_usage_sensor != nullptr)
    cpu_usage_sensor->publish_state(0);
  if (ram_free_sensor != nullptr)
    ram_free_sensor->publish_state(0);
  if (flash_free_sensor != nullptr)
    flash_free_sensor->publish_state(0);
  manual_adjustment_count_ = 0;
  if (manual_adjustment_sensor != nullptr)
    manual_adjustment_sensor->publish_state(0);
  if (xshut_state_binary_sensor != nullptr) {
    auto val = distanceSensor->get_xshut_state();
    if (val.has_value())
      xshut_state_binary_sensor->publish_state(*val);
  }
  if (interrupt_status_sensor != nullptr) {
    auto val = distanceSensor->get_interrupt_state();
    if (val.has_value())
      interrupt_status_sensor->publish_state(*val ? 1 : 0);
  }
  if (people_counter != nullptr)
    expected_counter_ = people_counter->state;

  // Seed crossing timestamp so fail-safe calibration doesn't fire in the first 2 minutes of operation.
  last_valid_crossing_ts_ = millis();

  publish_feature_list();
  publish_setting_entities();
  // Only set "ok" if calibration didn't just run (calibrate_zones sets its own status + reset timer)
  if (calibration_status_reset_ts_ == 0)
    update_status_text("ok");
}

void Roode::update() {
  if (distance_entry != nullptr) {
    distance_entry->publish_state(entry->getDistance());
  }
  if (distance_exit != nullptr) {
    distance_exit->publish_state(exit->getDistance());
  }
  if (xshut_state_binary_sensor != nullptr) {
    auto val = distanceSensor->get_xshut_state();
    if (val.has_value())
      xshut_state_binary_sensor->publish_state(*val);
  }
  if (interrupt_status_sensor != nullptr) {
    auto val = distanceSensor->get_interrupt_state();
    if (val.has_value())
      interrupt_status_sensor->publish_state(*val ? 1 : 0);
  }
  if (people_counter != nullptr && fabs(people_counter->state - expected_counter_) > 0.001f) {
    // Suppress detection for 300 ms after an automatic counter update.
    // On dual-core ESP32 the sensor task calls updateCounter() on core 1 while
    // update() runs on core 0; the brief window between call.perform() and
    // expected_counter_ being written caused legitimate automatic count changes
    // to be logged as phantom "manual_adjust" events.
    if (millis() - last_auto_update_ts_ < 300) {
      expected_counter_ = people_counter->state;
    } else {
      int diff = (int) roundf(people_counter->state - expected_counter_);
      manual_adjustment_count_ += abs(diff);
      expected_counter_ = people_counter->state;
      if (manual_adjustment_sensor != nullptr)
        manual_adjustment_sensor->publish_state(manual_adjustment_count_);
      if (diff != 0) {
        std::string sign = diff > 0 ? "+" : "";
        log_event("manual_adjust " + sign + std::to_string(diff) + " total=" + std::to_string(manual_adjustment_count_));
      }
    }
  }
}

void Roode::loop() {
  if (use_sensor_task_) {
    // When running on dual core the sensor loop runs in a separate task
    // Skip execution from main loop
    return;
  }
  uint32_t now = millis();
  if (last_loop_update_ts_ != 0 && (now - last_loop_update_ts_ > restart_timeout_ms_) &&
      (now - last_sensor_restart_ts_ > restart_backoff_ms_)) {
    ESP_LOGW(TAG, "Sensor unresponsive >%ds, restarting...", restart_timeout_ms_ / 1000);
    restart_sensor();
  }
  unsigned long start = micros();
  VL53L1_Error status = this->current_zone->readDistance(distanceSensor);
  if (status == VL53L1_ERROR_NONE) {
    last_loop_update_ts_ = millis();
    restart_backoff_ms_ = restart_timeout_ms_;  // reset backoff on successful read
  }
  uint16_t dist = this->current_zone->getDistance();
  if (status == VL53L1_ERROR_NONE && (dist == 0 || dist > 4000)) {
    invalid_read_count_++;
  } else {
    invalid_read_count_ = 0;
  }
  // Attempt to recover the sensor when repeated invalid distance values are observed
  if (invalid_read_count_ > invalid_distance_limit_ && (now - last_sensor_restart_ts_ > restart_backoff_ms_)) {
    ESP_LOGW(TAG, "Consecutive invalid distances, restarting...");
    restart_sensor();
  }
  bool zone_trig = current_zone->getMinDistance() < current_zone->threshold->max &&
                   current_zone->getMinDistance() > current_zone->threshold->min;
  // Also call path_tracking when zone was previously occupied but just cleared,
  // so the algorithm can register the NOBODY transition and fire count events.
  bool zone_was_active = zone_active_prev_[current_zone->id];
  zone_active_prev_[current_zone->id] = zone_trig;
  if (!cpu_optimizations_active_ || zone_trig || zone_was_active)
    path_tracking(this->current_zone);
  handle_sensor_status();
  this->current_zone = this->current_zone == this->entry ? this->exit : this->entry;
  // ESP_LOGI("Experimental", "Entry zone: %d, exit zone: %d",
  // entry->getDistance(Roode::distanceSensor, Roode::sensor_status),
  // exit->getDistance(Roode::distanceSensor, Roode::sensor_status)); unsigned
  unsigned long end = micros();
  unsigned long delta = end - start;
  loop_time_sum_ += delta;
  loop_count_++;
  update_metrics();
  // Periodic auto-calibration (single-core path).
  // Guard: epoch must be valid (NTP synced), interval must be non-zero,
  // and both zones must be clear so we never calibrate over a live reading.
  uint32_t now_epoch = static_cast<uint32_t>(time(nullptr));
  if (auto_calibration_interval_sec_ > 0 && now_epoch > 100000 &&
      now_epoch - last_calibration_ts_ >= auto_calibration_interval_sec_) {
    // Use debounced state — a momentary raw reading above threshold->max does not
    // mean the zone is truly empty (noise can produce false clearances).
    bool zones_clear = !zone_debounced_active_[0] && !zone_debounced_active_[1];
    if (zones_clear) {
      ESP_LOGI(TAG, "auto_calibration_running");
      calibrate_zones();
    } else {
      ESP_LOGD(TAG, "auto_calibration_deferred: zones occupied");
    }
  }
  delay(polling_interval_ms_);
}

bool Roode::handle_sensor_status() {
  bool check_status = false;
  std::string text_state;
  if (distanceSensor->is_failed()) {
    text_state = "offline";
  } else if (sensor_status == VL53L1_ERROR_NONE) {
    text_state = "ok";
    if (last_sensor_status != sensor_status) {
      if (status_sensor != nullptr)
        status_sensor->publish_state(sensor_status);
      check_status = true;
    }
  } else if (sensor_status == VL53L1_ERROR_TIME_OUT) {
    text_state = "timeout";
    if (status_sensor != nullptr)
      status_sensor->publish_state(sensor_status);
  } else {
    text_state = "error";
    if (status_sensor != nullptr)
      status_sensor->publish_state(sensor_status);
  }

  // Don't overwrite a pending calibration status message with "ok" — let the
  // 5-second reset timer in update_metrics() handle the transition back to "ok".
  if (text_state == "ok" && calibration_status_reset_ts_ != 0) {
    last_sensor_status = sensor_status;
    sensor_status = VL53L1_ERROR_NONE;
    return check_status;
  }
  update_status_text(text_state);
  last_sensor_status = sensor_status;
  sensor_status = VL53L1_ERROR_NONE;
  return check_status;
}

void Roode::path_tracking(Zone *zone) {
  static int PathTrack[] = {0, 0, 0, 0};
  static int PathTrackFillingSize = 1;  // init this to 1 as we start from state
                                        // where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  uint32_t timeout = state_ == STATE_ENTRY_ACTIVE ? 2500 : 3500;
  if (state_ != STATE_IDLE && millis() - state_started_ts > timeout) {
    state_ = STATE_IDLE;
    // Clear the PathTrack array and filling size so stale partial sequence data
    // cannot combine with the NEXT crossing to fire a spurious count.
    // Do NOT reset LeftPreviousStatus / RightPreviousStatus here — those reflect
    // real zone occupancy. Resetting them while a zone is still occupied would
    // immediately re-fire events on the next call, creating rapid state cycling
    // that produces endless counts.
    PathTrackFillingSize = 1;
    PathTrack[0] = PathTrack[1] = PathTrack[2] = PathTrack[3] = 0;
    path_track_first_event_ts_ = 0;
    // If both zones are genuinely idle (debounced), also clear the static
    // previous-status variables.  Without this, a stale SOMEONE value can
    // survive the timeout and combine with the very next zone event to fire
    // a phantom count — the primary cause of counts drifting up over time.
    if (!zone_debounced_active_[0] && !zone_debounced_active_[1]) {
      LeftPreviousStatus = NOBODY;
      RightPreviousStatus = NOBODY;
    }
    ESP_LOGW(TAG, "fsm_timeout_reset");
  }

  ESP_LOGV(TAG, "Zone %d distance %u (min=%u max=%u)", zone->id, zone->getMinDistance(), zone->threshold->min,
           zone->threshold->max);

  // PathTrack algorithm — debounce zone status to reject brief noise spikes.
  // A zone must be continuously within threshold for kZoneDwellMs before it
  // registers as SOMEONE, and continuously outside for kZoneClearMs before it
  // registers as NOBODY.  Single sensor frames of noise (< 150 ms) cannot
  // advance the FSM, eliminating the primary source of phantom crossings.
  {
    static constexpr uint32_t kZoneDwellMs = 150;
    static constexpr uint32_t kZoneClearMs = 80;
    uint32_t now_deb = millis();
    uint8_t zid = zone->id;
    bool raw_active = zone->getMinDistance() < zone->threshold->max &&
                      zone->getMinDistance() > zone->threshold->min;
    if (raw_active) {
      zone_dwell_first_clear_[zid] = 0;
      if (zone_dwell_first_active_[zid] == 0)
        zone_dwell_first_active_[zid] = now_deb;
      if (!zone_debounced_active_[zid] &&
          (now_deb - zone_dwell_first_active_[zid]) >= kZoneDwellMs)
        zone_debounced_active_[zid] = true;
    } else {
      zone_dwell_first_active_[zid] = 0;
      if (zone_debounced_active_[zid]) {
        if (zone_dwell_first_clear_[zid] == 0)
          zone_dwell_first_clear_[zid] = now_deb;
        if ((now_deb - zone_dwell_first_clear_[zid]) >= kZoneClearMs)
          zone_debounced_active_[zid] = false;
      } else {
        zone_dwell_first_clear_[zid] = 0;
      }
    }
  }

  if (zone_debounced_active_[zone->id]) {
    // Someone is in the sensing area
    CurrentZoneStatus = SOMEONE;
    if (presence_sensor != nullptr) {
      presence_sensor->publish_state(true);
    }
    // Expose occupancy for the specific zone when configured
    if (zone->id == 0 && entry_presence_sensor != nullptr) {
      entry_presence_sensor->publish_state(true);
    }
    if (zone->id == 1 && exit_presence_sensor != nullptr) {
      exit_presence_sensor->publish_state(true);
    }
    if (zone_triggered_start_[zone->id] == 0) {
      zone_triggered_start_[zone->id] = millis();
    }
  }
  if (CurrentZoneStatus == NOBODY) {
    // Clear zone-specific occupancy sensors once motion has left the area
    if (zone->id == 0 && entry_presence_sensor != nullptr) {
      entry_presence_sensor->publish_state(false);
    }
    if (zone->id == 1 && exit_presence_sensor != nullptr) {
      exit_presence_sensor->publish_state(false);
    }
    zone_triggered_start_[zone->id] = 0;
  } else if (zone_triggered_start_[zone->id] != 0 && millis() - zone_triggered_start_[zone->id] >= 10000 &&
             millis() - last_valid_crossing_ts_ >= 120000) {
    // Only fire fail-safe calibration when BOTH zones are debounced-clear.
    // Using raw getMinDistance() is insufficient: noise can temporarily push a
    // reading above threshold->max even while the zone is genuinely oscillating,
    // causing calibration to fire with a person present and corrupt the baseline.
    bool zones_clear = !zone_debounced_active_[0] && !zone_debounced_active_[1];
    if (zones_clear) {
      ESP_LOGI(CALIBRATION, "Fail safe calibration triggered for zone %d", zone->id);
      run_zone_calibration(zone->id);
      fail_safe_triggered_ = true;
      zone_triggered_start_[zone->id] = 0;
    } else {
      // Zone still occupied — defer and restart the 10s window so we keep checking
      ESP_LOGD(CALIBRATION, "Fail safe deferred: zone %d still occupied", zone->id);
      zone_triggered_start_[zone->id] = millis();
    }
  }

  // left zone
  if (zone == (this->invert_direction_ ? this->exit : this->entry)) {
    if (CurrentZoneStatus != LeftPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE) {
        state_ = STATE_ENTRY_ACTIVE;
        state_started_ts = millis();
      }

      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE) {
        // event in right zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else {
    if (CurrentZoneStatus != RightPreviousStatus) {
      // event in right zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
        if (state_ == STATE_ENTRY_ACTIVE) {
          state_ = STATE_BOTH_ACTIVE;
          state_started_ts = millis();
        }
      }
      // need to check left zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }

  // if an event has occured
  if (AnEventHasOccured) {
    ESP_LOGD(TAG, "Event has occured, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);
    if (PathTrackFillingSize < 4) {
      PathTrackFillingSize++;
    }
    // Record when the first event in a crossing sequence occurs.
    // We use this below to enforce a minimum crossing duration and reject
    // sequences that complete too quickly to be a real person traversal.
    // (The 150 ms zone dwell debounce already enforces ~460 ms naturally;
    // this 300 ms gate is a second independent layer of defence.)
    if (PathTrackFillingSize == 2 && path_track_first_event_ts_ == 0) {
      path_track_first_event_ts_ = millis();
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {
      ESP_LOGD(TAG, "Nobody anywhere, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1
      // 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is
        // always the case

        // Reject sequences that completed faster than a real person could walk
        // through two zones.  300 ms is conservative — actual people take 700 ms+.
        bool timing_ok = (path_track_first_event_ts_ == 0) ||
                         ((millis() - path_track_first_event_ts_) >= 300);

        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // This an exit
          if (timing_ok) {
            ESP_LOGI("Roode pathTracking", "Exit detected.");
            this->updateCounter(-1);
            last_valid_crossing_ts_ = millis();
            if (entry_exit_event_sensor != nullptr) {
              entry_exit_event_sensor->publish_state("Exit");
            }
          } else {
            ESP_LOGD(TAG, "crossing_rejected: exit sequence too fast (%ums)",
                     (unsigned) (millis() - path_track_first_event_ts_));
          }
        } else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an entry
          if (timing_ok) {
            ESP_LOGI("Roode pathTracking", "Entry detected.");
            this->updateCounter(1);
            last_valid_crossing_ts_ = millis();
            if (entry_exit_event_sensor != nullptr) {
              entry_exit_event_sensor->publish_state("Entry");
            }
          } else {
            ESP_LOGD(TAG, "crossing_rejected: entry sequence too fast (%ums)",
                     (unsigned) (millis() - path_track_first_event_ts_));
          }
        }
      }

      PathTrackFillingSize = 1;
      PathTrack[0] = PathTrack[1] = PathTrack[2] = PathTrack[3] = 0;
      path_track_first_event_ts_ = 0;
      state_ = STATE_IDLE;
    } else {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize - 1] = AllZonesCurrentStatus;
    }
  }
  if (presence_sensor != nullptr) {
    // Use debounced zone state rather than the FSM static variables
    // (LeftPreviousStatus / RightPreviousStatus).  The statics can carry a
    // stale SOMEONE value across a timeout or recalibration event, leaving
    // presence permanently stuck true even when the room is empty.
    if (!zone_debounced_active_[0] && !zone_debounced_active_[1]) {
      presence_sensor->publish_state(false);
    }
  }
}
void Roode::updateCounter(int delta) {
  if (this->people_counter == nullptr) {
    return;
  }
  auto next = this->people_counter->state + (float) delta;
  ESP_LOGI(TAG, "Updating people count: %d", (int) next);
  // Record the time before performing the update so update() can suppress the
  // narrow race window where people_counter->state has not yet reflected the
  // new value, which previously caused the automatic count change to be logged
  // as a phantom "manual_adjust".
  last_auto_update_ts_ = millis();
  auto call = this->people_counter->make_call();
  call.set_value(next);
  call.perform();
  // Set expected AFTER perform() so that if update() runs on the main core
  // between these two lines, it sees state==next and expected==old (diff=+delta)
  // rather than state==old and expected==next (diff=-delta).  The suppression
  // window in update() handles any remaining transient in either direction.
  expected_counter_ = next;
}
void Roode::suspend_sensor_task_for_calibration(uint32_t timeout_ms) {
  // use_sensor_task_ is false on single-core / ESP8266 — no sync needed there.
  if (!use_sensor_task_) return;
  calibration_in_progress_ = true;
  // Spin-wait until sensor_task has finished its current iteration and gone to
  // sleep (vTaskDelay).  sensor_task_reading_ covers the entire loop body so
  // this guarantees no concurrent sensor access — readDistance(), path_tracking,
  // run_zone_calibration(), and auto-calibrate_zones() are all included.
  // Timeout is 3 s to accommodate auto-cal aborting mid-calibrateThreshold
  // (worst case ~1.5 s per 50-sample pass) without racing.
  uint32_t deadline = millis() + timeout_ms;
  while (sensor_task_reading_ && millis() < deadline) {
    delay(1);
    App.feed_wdt();  // keep Core 0 WDT alive while we spin-wait
  }
  if (sensor_task_reading_) {
    ESP_LOGW(CALIBRATION, "sensor_task_reading timed out after %ums — proceeding anyway", timeout_ms);
  }
}

void Roode::resume_sensor_task_after_calibration() {
  if (!use_sensor_task_) return;
  calibration_in_progress_ = false;
}

void Roode::recalibration() {
  suspend_sensor_task_for_calibration();
  calibrate_zones();
  // Reset PathTrack FSM state so any in-flight partial crossing sequence from
  // before calibration cannot combine with post-calibration readings to fire a
  // phantom count.
  state_ = STATE_IDLE;
  zone_debounced_active_[0] = false;
  zone_debounced_active_[1] = false;
  zone_dwell_first_active_[0] = zone_dwell_first_active_[1] = 0;
  zone_dwell_first_clear_[0] = zone_dwell_first_clear_[1] = 0;
  path_track_first_event_ts_ = 0;
  resume_sensor_task_after_calibration();
}

void Roode::run_zone_calibration(uint8_t zone_id) {
  ESP_LOGI(CALIBRATION, "Calibration triggered for zone %d", zone_id);
  Zone *z = zone_id == 0 ? entry : exit;
  z->reset_roi(zone_id == 0 ? (orientation_ == Parallel ? 167 : 195) : (orientation_ == Parallel ? 231 : 60));

  // First pass: measure with reset ROI to get idle distance for ROI sizing.
  // 25 samples — enough for a reliable average while keeping WDT budget safe.
  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif
  z->calibrateThreshold(distanceSensor, 25);

  // Recalculate ROI for both zones based on the measured idle distance.
  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);

  // Second pass: re-measure with the calibrated ROI so thresholds match the ROI
  // that will actually be used during detection.
  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif
  z->calibrateThreshold(distanceSensor, 15);

  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif

  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  distanceSensor->set_ranging_mode(mode);

  calibration_data_[zone_id].baseline_mm = z->threshold->idle;
  calibration_data_[zone_id].threshold_min_mm = z->threshold->min;
  calibration_data_[zone_id].threshold_max_mm = z->threshold->max;
  calibration_data_[zone_id].last_calibrated_ts = static_cast<uint32_t>(time(nullptr));
  if (calibration_persistence_) {
    calibration_prefs_[zone_id].save(&calibration_data_[zone_id]);
  }

  // Clear sample buffers so stale distances don't bleed into post-calibration detection.
  z->reset_samples();

  // Publish updated calibration so HA sees new thresholds and ROI immediately.
  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);
  publish_setting_entities();
  last_calibration_ts_ =
      std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
  publish_feature_list();
  char cal_msg[64];
  snprintf(cal_msg, sizeof(cal_msg), "zone %d cal: idle=%dmm thresh=%d-%d%%",
           zone_id, z->threshold->idle,
           z->threshold->min_percentage.value_or(15),
           z->threshold->max_percentage.value_or(80));
  update_status_text(std::string(cal_msg));
  calibration_status_reset_ts_ = millis();
}

void Roode::apply_cpu_optimizations(float cpu) {
  if (cpu_optimizations_active_ || cpu <= cpu_opt_activate_threshold_)
    return;
  ESP_LOGW(TAG, "CPU usage %.1f%% exceeded threshold, applying optimizations", cpu);
  polling_interval_ms_ = 30;

  // Avoid extremely small windows and accuracy-reducing filters.
  if (filter_window_ < 5) {
    filter_window_ = 5;
    entry->set_filter_window(5);
    exit->set_filter_window(5);
  }
  if (filter_mode_ != FILTER_MEDIAN) {
    filter_mode_ = FILTER_MEDIAN;
    entry->set_filter_mode(FILTER_MEDIAN);
    exit->set_filter_mode(FILTER_MEDIAN);
  }
  cpu_optimizations_active_ = true;
}

void Roode::reset_cpu_optimizations(float cpu) {
  if (!cpu_optimizations_active_ || cpu > cpu_opt_deactivate_threshold_)
    return;
  ESP_LOGI(TAG, "CPU usage %.1f%% stable, reverting optimizations", cpu);
  polling_interval_ms_ = 10;
  filter_window_ = default_filter_window_;
  entry->set_filter_window(default_filter_window_);
  exit->set_filter_window(default_filter_window_);
  filter_mode_ = default_filter_mode_;
  entry->set_filter_mode(default_filter_mode_);
  exit->set_filter_mode(default_filter_mode_);
  cpu_optimizations_active_ = false;
}

void Roode::update_metrics() {
  uint32_t now = millis();
  if (now - loop_window_start_ < 10000)
    return;
  float cpu = 0.0f;
  if (loop_count_ > 0) {
    float avg_ms = (float) loop_time_sum_ / loop_count_ / 1000.0f;
    if (loop_time_sensor != nullptr)
      loop_time_sensor->publish_state(avg_ms);
    cpu = ((float) loop_time_sum_ / ((now - loop_window_start_) * 1000.0f)) * 100.0f;
    if (cpu_usage_sensor != nullptr)
      cpu_usage_sensor->publish_state(cpu);
  }
  if (ram_free_sensor != nullptr) {
    uint32_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    float used_percent = 0;
    if (total_heap > 0) {
      uint32_t used = total_heap - heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
      used_percent = ((float) used / (float) total_heap) * 100.0f;
    }
    ram_free_sensor->publish_state(used_percent);
  }
  if (flash_free_sensor != nullptr) {
    uint32_t total_flash = 0;
    esp_flash_get_size(nullptr, &total_flash);
    float used_percent = 0;
    if (total_flash > 0) {
      const esp_partition_t *running = esp_ota_get_running_partition();
      uint32_t used = running ? running->size : 0;
      used_percent = ((float) used / (float) total_flash) * 100.0f;
    }
    flash_free_sensor->publish_state(used_percent);
  }
  apply_cpu_optimizations(cpu);
  reset_cpu_optimizations(cpu);
  // Reset calibration status message back to "ok" after 5 seconds
  if (calibration_status_reset_ts_ != 0 && (now - calibration_status_reset_ts_) >= 5000) {
    calibration_status_reset_ts_ = 0;
    last_status_text_ = "";  // Force re-publish even if previous text was also "ok"
    update_status_text("ok");
  }
  loop_time_sum_ = 0;
  loop_count_ = 0;
  loop_window_start_ = now;
}

const RangingMode *Roode::determine_ranging_mode(uint16_t average_entry_zone_distance,
                                                 uint16_t average_exit_zone_distance) {
  uint16_t min = average_entry_zone_distance < average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  uint16_t max = average_entry_zone_distance > average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  if (min <= short_distance_threshold) {
    return Ranging::Short;
  }
  if (max > short_distance_threshold && min <= medium_distance_threshold) {
    return Ranging::Medium;
  }
  if (max > medium_distance_threshold && min <= medium_long_distance_threshold) {
    return Ranging::Long;
  }
  if (max > medium_long_distance_threshold && min <= long_distance_threshold) {
    return Ranging::Longer;
  }
  return Ranging::Longest;
}

void Roode::calibrate_zones() {
  ESP_LOGI(SETUP, "Calibrating sensor zones");
  update_status_text("cal: keep room empty...");

  entry->reset_roi(orientation_ == Parallel ? 167 : 195);
  exit->reset_roi(orientation_ == Parallel ? 231 : 60);

  // Phase 1: baseline measurement with wide default ROI.
  // Uses 30 samples per zone — statistically solid, stays within the 5 s task WDT.
  update_status_text("cal: measuring baseline...");
  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif
  calibrateDistance();

  // If Core 0 requested manual calibration while we were measuring baseline,
  // abort so Core 0 can take over without a concurrent sensor access race.
  if (calibration_in_progress_) {
    ESP_LOGW(CALIBRATION, "auto-cal aborted after baseline (manual cal requested)");
    return;
  }

  // Phase 2: entry zone — compute optimal ROI then re-measure with it.
  // 20 samples is enough for a reliable threshold on the narrower ROI.
  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif
  update_status_text("cal: calibrating entry zone...");
  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  entry->calibrateThreshold(distanceSensor, 20);

  if (calibration_in_progress_) {
    ESP_LOGW(CALIBRATION, "auto-cal aborted after entry zone (manual cal requested)");
    return;
  }

  // Phase 3: exit zone.
  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif
  update_status_text("cal: calibrating exit zone...");
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->calibrateThreshold(distanceSensor, 20);

  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif

  // Recalculate ranging mode after the final calibrated measurements — idle distances
  // can shift slightly once the optimised ROI is applied.
  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  distanceSensor->set_ranging_mode(mode);

  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);
  // Sync HA number entities (ROI h/w, threshold %, etc.) with the new values
  // so the dashboard reflects what was actually calibrated without a reboot.
  publish_setting_entities();

  uint32_t now_epoch = static_cast<uint32_t>(time(nullptr));
  calibration_data_[0] = {entry->threshold->idle, entry->threshold->min, entry->threshold->max, now_epoch};
  calibration_data_[1] = {exit->threshold->idle, exit->threshold->min, exit->threshold->max, now_epoch};

  if (calibration_persistence_) {
    calibration_prefs_[0].save(&calibration_data_[0]);
    calibration_prefs_[1].save(&calibration_data_[1]);
  }
  // Clear sample buffers so stale pre-calibration distances don't pollute the
  // first readings after the new thresholds are applied.
  entry->reset_samples();
  exit->reset_samples();

  ESP_LOGI(SETUP, "Finished calibrating: entry idle=%dmm %d-%d%% | exit idle=%dmm %d-%d%%",
           entry->threshold->idle,
           entry->threshold->min_percentage.value_or(15),
           entry->threshold->max_percentage.value_or(80),
           exit->threshold->idle,
           exit->threshold->min_percentage.value_or(15),
           exit->threshold->max_percentage.value_or(80));
  last_calibration_ts_ =
      std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
  publish_feature_list();
  char cal_msg[80];
  snprintf(cal_msg, sizeof(cal_msg), "cal done: e=%dmm x=%dmm thresh=%d-%d%%",
           entry->threshold->idle, exit->threshold->idle,
           entry->threshold->min_percentage.value_or(15),
           entry->threshold->max_percentage.value_or(80));
  update_status_text(std::string(cal_msg));
  calibration_status_reset_ts_ = millis();
}

void Roode::calibrateDistance() {
  auto *const initial = distanceSensor->get_ranging_mode_override().value_or(Ranging::Longest);
  distanceSensor->set_ranging_mode(initial);

  // 30 samples each: statistically sound, fast enough to stay within WDT budget.
  entry->calibrateThreshold(distanceSensor, 30);
  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif
  exit->calibrateThreshold(distanceSensor, 30);
  App.feed_wdt();
#ifdef USE_ESP32
  esp_task_wdt_reset();
#endif

  if (distanceSensor->get_ranging_mode_override().has_value()) {
    return;
  }
  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  if (mode != initial) {
    distanceSensor->set_ranging_mode(mode);
  }
}

void Roode::publish_sensor_configuration(Zone *entry, Zone *exit, bool isMax) {
  if (isMax) {
    if (max_threshold_entry_sensor != nullptr) {
      max_threshold_entry_sensor->publish_state(entry->threshold->max);
    }

    if (max_threshold_exit_sensor != nullptr) {
      max_threshold_exit_sensor->publish_state(exit->threshold->max);
    }
  } else {
    if (min_threshold_entry_sensor != nullptr) {
      min_threshold_entry_sensor->publish_state(entry->threshold->min);
    }
    if (min_threshold_exit_sensor != nullptr) {
      min_threshold_exit_sensor->publish_state(exit->threshold->min);
    }
  }

  if (entry_roi_height_sensor != nullptr) {
    entry_roi_height_sensor->publish_state(entry->roi->height);
  }
  if (entry_roi_width_sensor != nullptr) {
    entry_roi_width_sensor->publish_state(entry->roi->width);
  }

  if (exit_roi_height_sensor != nullptr) {
    exit_roi_height_sensor->publish_state(exit->roi->height);
  }
  if (exit_roi_width_sensor != nullptr) {
    exit_roi_width_sensor->publish_state(exit->roi->width);
  }
}

void Roode::publish_feature_list() {
  auto fmt_bytes = [](uint32_t bytes) {
    char buf[16];
    if (bytes >= 1024UL * 1024UL * 1024UL)
      snprintf(buf, sizeof(buf), "%uGB", bytes / 1024 / 1024 / 1024);
    else if (bytes >= 1024 * 1024)
      snprintf(buf, sizeof(buf), "%uMB", bytes / 1024 / 1024);
    else
      snprintf(buf, sizeof(buf), "%uKB", bytes / 1024);
    return std::string(buf);
  };

  auto fmt_time = [](uint32_t epoch) {
    if (epoch == 0)
      return std::string("unknown");
    time_t t = epoch;
    struct tm tm_time;
    if (!localtime_r(&t, &tm_time))
      return std::string("unknown");
    char buf[8];
    int hour = tm_time.tm_hour % 12;
    if (hour == 0)
      hour = 12;
    snprintf(buf, sizeof(buf), "%d:%02d%cM", hour, tm_time.tm_min, tm_time.tm_hour >= 12 ? 'P' : 'A');
    return std::string(buf);
  };

  std::vector<std::pair<std::string, std::string>> features;
#ifdef USE_ESP32
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  features.push_back({"cpu_mode", use_sensor_task_ ? "dual" : "single"});
  features.push_back({"cpu", CONFIG_IDF_TARGET});
  features.push_back({"cpu_cores", std::to_string(chip_info.cores)});
#else
  features.push_back({"cpu_mode", "single"});
  features.push_back({"cpu", "ESP8266"});
  features.push_back({"cpu_cores", "1"});
#endif
  features.push_back({"xshut", distanceSensor->get_xshut_state().has_value() ? "enabled" : "disabled"});
  features.push_back({"refresh", distanceSensor->is_interrupt_enabled() ? "interrupt" : "polling"});
  features.push_back({"ram", fmt_bytes(heap_caps_get_total_size(MALLOC_CAP_DEFAULT))});
  uint32_t flash_size = 0; esp_flash_get_size(nullptr, &flash_size);
  features.push_back({"flash", fmt_bytes(flash_size)});
  features.push_back({"calibration_value", std::to_string(entry->threshold->idle)});
  uint32_t last_cal_epoch = std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
  features.push_back({"calibration", fmt_time(last_cal_epoch)});

  std::string feature_list;
  for (size_t i = 0; i < features.size(); ++i) {
    feature_list += features[i].first + ":" + features[i].second;
    if (i + 1 < features.size())
      feature_list += "\n";
  }
  if (enabled_features_sensor != nullptr)
    enabled_features_sensor->publish_state(feature_list);
  log_event(std::string("features_enabled: ") + feature_list);
}

void Roode::update_status_text(const std::string &status) {
  if (status_text_sensor != nullptr && status != last_status_text_) {
    status_text_sensor->publish_state(status);
    last_status_text_ = status;
  }
}

void Roode::apply_filter_mode(FilterMode mode) {
  filter_mode_ = mode;
  default_filter_mode_ = mode;
  entry->set_filter_mode(mode);
  exit->set_filter_mode(mode);
  if (calibration_persistence_) {
    uint8_t mode_byte = static_cast<uint8_t>(mode);
    filter_mode_pref_.save(&mode_byte);
  }
  if (filter_mode_select_ != nullptr) {
    filter_mode_select_->publish_state(filter_mode_to_str(mode));
  }
  ESP_LOGI(TAG, "filter_mode_changed: %s", filter_mode_to_str(mode));
}

void Roode::restore_settings_from_flash() {
  if (!calibration_persistence_)
    return;

  filter_mode_pref_       = global_preferences->make_preference<uint8_t>(0xB0);
  sampling_pref_          = global_preferences->make_preference<uint8_t>(0xB1);
  filter_window_pref_     = global_preferences->make_preference<uint8_t>(0xB2);
  entry_max_pct_pref_     = global_preferences->make_preference<uint8_t>(0xB3);
  entry_min_pct_pref_     = global_preferences->make_preference<uint8_t>(0xB4);
  exit_max_pct_pref_      = global_preferences->make_preference<uint8_t>(0xB5);
  exit_min_pct_pref_      = global_preferences->make_preference<uint8_t>(0xB6);
  auto_cal_interval_pref_ = global_preferences->make_preference<uint8_t>(0xB7);
  entry_roi_height_pref_  = global_preferences->make_preference<uint8_t>(0xB8);
  entry_roi_width_pref_   = global_preferences->make_preference<uint8_t>(0xB9);
  exit_roi_height_pref_   = global_preferences->make_preference<uint8_t>(0xBA);
  exit_roi_width_pref_    = global_preferences->make_preference<uint8_t>(0xBB);
  ranging_mode_pref_      = global_preferences->make_preference<uint8_t>(0xBC);
  invert_direction_pref_  = global_preferences->make_preference<uint8_t>(0xBD);

  uint8_t val;

  if (filter_mode_pref_.load(&val) && val < 3) {
    auto mode = static_cast<FilterMode>(val);
    filter_mode_ = mode; default_filter_mode_ = mode;
    entry->set_filter_mode(mode); exit->set_filter_mode(mode);
  }
  if (sampling_pref_.load(&val) && val >= 1) {
    samples = val;
    entry->set_max_samples(val); exit->set_max_samples(val);
  }
  if (filter_window_pref_.load(&val) && val >= 1) {
    filter_window_ = val; default_filter_window_ = val;
    entry->set_filter_window(val); exit->set_filter_window(val);
  }
  // Reject extreme percentages that produce degenerate thresholds.
  // max_pct=100% makes threshold->max == idle, so any sub-idle noise triggers
  // zone active (768 false detections/21 min observed in the field).
  // min_pct=1% makes threshold->min ≈ 22 mm, matching virtually every reading.
  // Out-of-range values are silently discarded; the defaults (15%/80%) apply.
  if (entry_max_pct_pref_.load(&val) && val >= 51 && val <= 97)
    entry->set_threshold_percentages(entry->threshold->min_percentage.value_or(15), val);
  if (entry_min_pct_pref_.load(&val) && val >= 2 && val <= 49)
    entry->set_threshold_percentages(val, entry->threshold->max_percentage.value_or(80));
  if (exit_max_pct_pref_.load(&val) && val >= 51 && val <= 97)
    exit->set_threshold_percentages(exit->threshold->min_percentage.value_or(15), val);
  if (exit_min_pct_pref_.load(&val) && val >= 2 && val <= 49)
    exit->set_threshold_percentages(val, exit->threshold->max_percentage.value_or(80));
  if (auto_cal_interval_pref_.load(&val))
    auto_calibration_interval_sec_ = static_cast<uint32_t>(val) * 1800;  // stored in 30-min units
  if (entry_roi_height_pref_.load(&val) && val >= 4 && val <= 16)
    entry->roi_override->set_height(val);
  if (entry_roi_width_pref_.load(&val) && val >= 4 && val <= 16)
    entry->roi_override->set_width(val);
  if (exit_roi_height_pref_.load(&val) && val >= 4 && val <= 16)
    exit->roi_override->set_height(val);
  if (exit_roi_width_pref_.load(&val) && val >= 4 && val <= 16)
    exit->roi_override->set_width(val);
  if (ranging_mode_pref_.load(&val) && val > 0 && val < 6) {
    const RangingMode *modes[] = {nullptr, Ranging::Short, Ranging::Medium, Ranging::Long, Ranging::Longer, Ranging::Longest};
    distanceSensor->set_ranging_mode_override(modes[val]);
  }
  if (invert_direction_pref_.load(&val))
    invert_direction_ = (val != 0);
}

void Roode::publish_setting_entities() {
  if (filter_mode_select_ != nullptr)
    filter_mode_select_->publish_state(filter_mode_to_str(filter_mode_));
  if (ranging_mode_select_ != nullptr) {
    auto ov = distanceSensor->get_ranging_mode_override();
    std::string rm = "auto";
    if (ov.has_value()) {
      auto *m = *ov;
      if (m == Ranging::Short) rm = "short";
      else if (m == Ranging::Medium) rm = "medium";
      else if (m == Ranging::Long) rm = "long";
      else if (m == Ranging::Longer) rm = "longer";
      else if (m == Ranging::Longest) rm = "longest";
    }
    ranging_mode_select_->publish_state(rm);
  }
  if (invert_direction_switch_ != nullptr)
    invert_direction_switch_->publish_state(invert_direction_);
  if (cal_persistence_switch_ != nullptr)
    cal_persistence_switch_->publish_state(calibration_persistence_);
  if (filter_window_number_ != nullptr)
    filter_window_number_->publish_state(filter_window_);
  if (sampling_number_ != nullptr)
    sampling_number_->publish_state(samples);
  if (entry_max_threshold_number_ != nullptr)
    entry_max_threshold_number_->publish_state(entry->threshold->max_percentage.value_or(80));
  if (entry_min_threshold_number_ != nullptr)
    entry_min_threshold_number_->publish_state(entry->threshold->min_percentage.value_or(15));
  if (exit_max_threshold_number_ != nullptr)
    exit_max_threshold_number_->publish_state(exit->threshold->max_percentage.value_or(80));
  if (exit_min_threshold_number_ != nullptr)
    exit_min_threshold_number_->publish_state(exit->threshold->min_percentage.value_or(15));
  if (auto_cal_interval_number_ != nullptr)
    auto_cal_interval_number_->publish_state(auto_calibration_interval_sec_ / 3600.0f);
  if (entry_roi_height_number_ != nullptr)
    entry_roi_height_number_->publish_state(entry->roi->height);
  if (entry_roi_width_number_ != nullptr)
    entry_roi_width_number_->publish_state(entry->roi->width);
  if (exit_roi_height_number_ != nullptr)
    exit_roi_height_number_->publish_state(exit->roi->height);
  if (exit_roi_width_number_ != nullptr)
    exit_roi_width_number_->publish_state(exit->roi->width);
}

void Roode::apply_sampling(uint8_t val) {
  if (val < 1) val = 1;
  samples = val;
  entry->set_max_samples(val);
  exit->set_max_samples(val);
  if (calibration_persistence_) {
    sampling_pref_.save(&val);
  }
  if (sampling_number_ != nullptr)
    sampling_number_->publish_state(val);
  ESP_LOGI(TAG, "sampling changed: %u", val);
}

void Roode::apply_filter_window(uint8_t val) {
  if (val < 1) val = 1;
  filter_window_ = val;
  default_filter_window_ = val;
  entry->set_filter_window(val);
  exit->set_filter_window(val);
  if (calibration_persistence_) {
    filter_window_pref_.save(&val);
  }
  if (filter_window_number_ != nullptr)
    filter_window_number_->publish_state(val);
  ESP_LOGI(TAG, "filter_window changed: %u", val);
}

void Roode::apply_entry_max_threshold_pct(uint8_t pct) {
  if (pct < 51) pct = 51;
  if (pct > 97) pct = 97;  // never let max reach idle (100% = instant false detection)
  entry->set_threshold_percentages(entry->threshold->min_percentage.value_or(15), pct);
  if (calibration_persistence_)
    entry_max_pct_pref_.save(&pct);
  publish_sensor_configuration(entry, exit, true);
  if (entry_max_threshold_number_ != nullptr)
    entry_max_threshold_number_->publish_state(pct);
}

void Roode::apply_entry_min_threshold_pct(uint8_t pct) {
  if (pct < 2) pct = 2;
  if (pct > 49) pct = 49;
  entry->set_threshold_percentages(pct, entry->threshold->max_percentage.value_or(80));
  if (calibration_persistence_)
    entry_min_pct_pref_.save(&pct);
  publish_sensor_configuration(entry, exit, false);
  if (entry_min_threshold_number_ != nullptr)
    entry_min_threshold_number_->publish_state(pct);
}

void Roode::apply_exit_max_threshold_pct(uint8_t pct) {
  if (pct < 51) pct = 51;
  if (pct > 97) pct = 97;  // never let max reach idle (100% = instant false detection)
  exit->set_threshold_percentages(exit->threshold->min_percentage.value_or(15), pct);
  if (calibration_persistence_)
    exit_max_pct_pref_.save(&pct);
  publish_sensor_configuration(entry, exit, true);
  if (exit_max_threshold_number_ != nullptr)
    exit_max_threshold_number_->publish_state(pct);
}

void Roode::apply_exit_min_threshold_pct(uint8_t pct) {
  if (pct < 2) pct = 2;
  if (pct > 49) pct = 49;
  exit->set_threshold_percentages(pct, exit->threshold->max_percentage.value_or(80));
  if (calibration_persistence_)
    exit_min_pct_pref_.save(&pct);
  publish_sensor_configuration(entry, exit, false);
  if (exit_min_threshold_number_ != nullptr)
    exit_min_threshold_number_->publish_state(pct);
}

void Roode::apply_auto_calibration_interval(float hours) {
  if (hours < 0) hours = 0;
  auto_calibration_interval_sec_ = static_cast<uint32_t>(hours * 3600.0f);
  if (calibration_persistence_) {
    // Store in 30-min units (0–48 fits in uint8_t for 0–24h range)
    uint8_t units = static_cast<uint8_t>(hours * 2.0f);
    auto_cal_interval_pref_.save(&units);
  }
  if (auto_cal_interval_number_ != nullptr)
    auto_cal_interval_number_->publish_state(hours);
  ESP_LOGI(TAG, "auto_cal_interval changed: %.1fh", hours);
}

void Roode::apply_entry_roi(uint8_t h, uint8_t w) {
  if (h < 4) h = 4; if (h > 16) h = 16;
  if (w < 4) w = 4; if (w > 16) w = 16;
  entry->roi_override->set_height(h);
  entry->roi_override->set_width(w);
  if (calibration_persistence_) {
    entry_roi_height_pref_.save(&h);
    entry_roi_width_pref_.save(&w);
  }
  update_status_text("calibrating entry ROI...");
  // Suspend the sensor task before touching the hardware — run_zone_calibration
  // drives the sensor bus from Core 0 and must not race with Core 1's sensor_task.
  suspend_sensor_task_for_calibration();
  run_zone_calibration(0);
  resume_sensor_task_after_calibration();
  // run_zone_calibration already calls publish_sensor_configuration + publish_setting_entities.
  // Re-publish the actual calibrated ROI so the number entities reflect the true values
  // (the override may differ from what roi_calibration() chose if it was partially constrained).
  if (entry_roi_height_number_ != nullptr)
    entry_roi_height_number_->publish_state(entry->roi->height);
  if (entry_roi_width_number_ != nullptr)
    entry_roi_width_number_->publish_state(entry->roi->width);
}

void Roode::apply_exit_roi(uint8_t h, uint8_t w) {
  if (h < 4) h = 4; if (h > 16) h = 16;
  if (w < 4) w = 4; if (w > 16) w = 16;
  exit->roi_override->set_height(h);
  exit->roi_override->set_width(w);
  if (calibration_persistence_) {
    exit_roi_height_pref_.save(&h);
    exit_roi_width_pref_.save(&w);
  }
  update_status_text("calibrating exit ROI...");
  suspend_sensor_task_for_calibration();
  run_zone_calibration(1);
  resume_sensor_task_after_calibration();
  if (exit_roi_height_number_ != nullptr)
    exit_roi_height_number_->publish_state(exit->roi->height);
  if (exit_roi_width_number_ != nullptr)
    exit_roi_width_number_->publish_state(exit->roi->width);
}

void Roode::apply_invert_direction(bool inv) {
  invert_direction_ = inv;
  if (calibration_persistence_) {
    uint8_t val = inv ? 1 : 0;
    invert_direction_pref_.save(&val);
  }
  if (invert_direction_switch_ != nullptr)
    invert_direction_switch_->publish_state(inv);
  ESP_LOGI(TAG, "invert_direction changed: %s", inv ? "true" : "false");
}

void Roode::apply_calibration_persistence(bool val) {
  calibration_persistence_ = val;
  if (cal_persistence_switch_ != nullptr)
    cal_persistence_switch_->publish_state(val);
  ESP_LOGI(TAG, "calibration_persistence changed: %s", val ? "true" : "false");
}

void Roode::apply_ranging_mode(const std::string &mode) {
  uint8_t mode_idx = 0;  // 0 = auto
  if (mode == "short") {
    distanceSensor->set_ranging_mode_override(Ranging::Short); mode_idx = 1;
    distanceSensor->set_ranging_mode(Ranging::Short);
  } else if (mode == "medium") {
    distanceSensor->set_ranging_mode_override(Ranging::Medium); mode_idx = 2;
    distanceSensor->set_ranging_mode(Ranging::Medium);
  } else if (mode == "long") {
    distanceSensor->set_ranging_mode_override(Ranging::Long); mode_idx = 3;
    distanceSensor->set_ranging_mode(Ranging::Long);
  } else if (mode == "longer") {
    distanceSensor->set_ranging_mode_override(Ranging::Longer); mode_idx = 4;
    distanceSensor->set_ranging_mode(Ranging::Longer);
  } else if (mode == "longest") {
    distanceSensor->set_ranging_mode_override(Ranging::Longest); mode_idx = 5;
    distanceSensor->set_ranging_mode(Ranging::Longest);
  } else {
    // "auto" — no override; ranging mode will be auto-selected on next full recalibration
    mode_idx = 0;
  }
  if (calibration_persistence_) {
    ranging_mode_pref_.save(&mode_idx);
  }
  if (ranging_mode_select_ != nullptr)
    ranging_mode_select_->publish_state(mode);
  ESP_LOGI(TAG, "ranging_mode changed: %s", mode.c_str());
}

void Roode::person_calibration() {
  suspend_sensor_task_for_calibration();
  ESP_LOGI(CALIBRATION, "person_calibration: starting");
  update_status_text("person cal: stand in doorway...");
  calibration_status_reset_ts_ = 0;

  const int num_samples = 30;
  bool any_adjusted = false;

  for (int z = 0; z < 2; z++) {
    Zone *zone = z == 0 ? entry : exit;
    char zone_msg[48];
    snprintf(zone_msg, sizeof(zone_msg), "person cal: measuring zone %d...", z);
    update_status_text(std::string(zone_msg));

    uint32_t sum = 0;
    int valid = 0;
    for (int i = 0; i < num_samples; i++) {
      zone->readDistance(distanceSensor);
      uint16_t d = zone->getDistance();
      if (d > 0 && d < 4000) {
        sum += d;
        valid++;
      }
      delay(20);
    }
    if (valid == 0) {
      update_status_text("person cal: failed - no readings");
      calibration_status_reset_ts_ = millis();
      resume_sensor_task_after_calibration();
      return;
    }
    uint16_t avg_mm = static_cast<uint16_t>(sum / valid);
    uint16_t idle = zone->threshold->idle;
    if (idle == 0) continue;

    // Person distance as percentage of idle distance
    uint8_t person_pct = static_cast<uint8_t>((avg_mm * 100) / idle);
    uint8_t cur_max = zone->threshold->max_percentage.value_or(80);
    uint8_t cur_min = zone->threshold->min_percentage.value_or(15);

    if (person_pct > cur_max || person_pct < cur_min) {
      // Person is outside the detection window — adjust max upward with 5% margin
      uint8_t new_max = static_cast<uint8_t>(std::min(static_cast<int>(person_pct) + 5, 100));
      zone->set_threshold_percentages(cur_min, new_max);
      // Save to flash
      if (calibration_persistence_) {
        if (z == 0) {
          entry_max_pct_pref_.save(&new_max);
        } else {
          exit_max_pct_pref_.save(&new_max);
        }
      }
      char msg[56];
      snprintf(msg, sizeof(msg), "person cal: zone %d max -> %u%%", z, new_max);
      update_status_text(std::string(msg));
      ESP_LOGI(CALIBRATION, "zone %d: person_pct=%u adjusted max to %u%%", z, person_pct, new_max);
      any_adjusted = true;
    }
  }

  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);
  // Publish updated threshold number entities
  if (entry_max_threshold_number_ != nullptr)
    entry_max_threshold_number_->publish_state(entry->threshold->max_percentage.value_or(80));
  if (exit_max_threshold_number_ != nullptr)
    exit_max_threshold_number_->publish_state(exit->threshold->max_percentage.value_or(80));

  if (!any_adjusted) {
    update_status_text("person cal: thresholds already ok");
  } else {
    update_status_text("person cal: done");
  }
  calibration_status_reset_ts_ = millis();
  resume_sensor_task_after_calibration();
}

void Roode::calibrate_low_obstacle() {
  // Low obstacle (e.g. pet, plant) — reads at a FAR distance, ~70-90% of idle.
  // Lower the max threshold so the obstacle distance falls outside the detection window.
  suspend_sensor_task_for_calibration();
  ESP_LOGI(CALIBRATION, "calibrate_low_obstacle: starting");
  update_status_text("low obs cal: keep obstacle in place...");
  calibration_status_reset_ts_ = 0;

  const int num_samples = 30;

  for (int z = 0; z < 2; z++) {
    Zone *zone = z == 0 ? entry : exit;
    char zone_msg[56];
    snprintf(zone_msg, sizeof(zone_msg), "low obs cal: measuring zone %d...", z);
    update_status_text(std::string(zone_msg));

    uint32_t sum = 0;
    int valid = 0;
    for (int i = 0; i < num_samples; i++) {
      zone->readDistance(distanceSensor);
      uint16_t d = zone->getDistance();
      if (d > 0 && d < 4000) {
        sum += d;
        valid++;
      }
      delay(20);
    }
    if (valid == 0) {
      update_status_text("low obs cal: failed - no readings");
      calibration_status_reset_ts_ = millis();
      resume_sensor_task_after_calibration();
      return;
    }

    uint16_t obs_avg = static_cast<uint16_t>(sum / valid);
    uint16_t idle = zone->threshold->idle;
    if (idle == 0) continue;

    // Set max 40mm below the obstacle so it falls outside the detection window
    int new_max_mm = static_cast<int>(obs_avg) - 40;
    if (new_max_mm < 100) {
      update_status_text("low obs cal: obstacle too close to sensor");
      calibration_status_reset_ts_ = millis();
      resume_sensor_task_after_calibration();
      return;
    }
    uint8_t new_max_pct = static_cast<uint8_t>((static_cast<uint32_t>(new_max_mm) * 100) / idle);
    new_max_pct = std::max<uint8_t>(51, std::min<uint8_t>(95, new_max_pct));
    uint8_t cur_min = zone->threshold->min_percentage.value_or(15);
    if (new_max_pct <= cur_min + 2) {
      update_status_text("low obs cal: no detection window - check placement");
      calibration_status_reset_ts_ = millis();
      resume_sensor_task_after_calibration();
      return;
    }

    zone->set_threshold_percentages(cur_min, new_max_pct);
    if (calibration_persistence_) {
      if (z == 0) entry_max_pct_pref_.save(&new_max_pct);
      else         exit_max_pct_pref_.save(&new_max_pct);
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "low obs cal: zone %d max -> %u%% (%dmm)", z, new_max_pct, new_max_mm);
    update_status_text(std::string(msg));
    ESP_LOGI(CALIBRATION, "zone %d low obs: avg=%dmm new_max=%dmm (%u%%)", z, obs_avg, new_max_mm, new_max_pct);
  }

  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);
  if (entry_max_threshold_number_ != nullptr)
    entry_max_threshold_number_->publish_state(entry->threshold->max_percentage.value_or(80));
  if (exit_max_threshold_number_ != nullptr)
    exit_max_threshold_number_->publish_state(exit->threshold->max_percentage.value_or(80));

  update_status_text("low obs cal: done");
  calibration_status_reset_ts_ = millis();
  resume_sensor_task_after_calibration();
}

void Roode::calibrate_high_obstacle() {
  // High obstacle (e.g. open door, cabinet) — reads at a CLOSE distance, ~20-50% of idle.
  // Raise the min threshold so the obstacle distance falls outside the detection window.
  suspend_sensor_task_for_calibration();
  ESP_LOGI(CALIBRATION, "calibrate_high_obstacle: starting");
  update_status_text("high obs cal: keep obstacle in place...");
  calibration_status_reset_ts_ = 0;

  const int num_samples = 30;

  for (int z = 0; z < 2; z++) {
    Zone *zone = z == 0 ? entry : exit;
    char zone_msg[56];
    snprintf(zone_msg, sizeof(zone_msg), "high obs cal: measuring zone %d...", z);
    update_status_text(std::string(zone_msg));

    uint32_t sum = 0;
    int valid = 0;
    for (int i = 0; i < num_samples; i++) {
      zone->readDistance(distanceSensor);
      uint16_t d = zone->getDistance();
      if (d > 0 && d < 4000) {
        sum += d;
        valid++;
      }
      delay(20);
    }
    if (valid == 0) {
      update_status_text("high obs cal: failed - no readings");
      calibration_status_reset_ts_ = millis();
      resume_sensor_task_after_calibration();
      return;
    }

    uint16_t obs_avg = static_cast<uint16_t>(sum / valid);
    uint16_t idle = zone->threshold->idle;
    if (idle == 0) continue;

    // Set min 40mm above the obstacle so it falls outside the detection window
    int new_min_mm = static_cast<int>(obs_avg) + 40;
    uint8_t new_min_pct = static_cast<uint8_t>((static_cast<uint32_t>(new_min_mm) * 100) / idle);
    new_min_pct = std::max<uint8_t>(2, std::min<uint8_t>(49, new_min_pct));
    // Enforce 100mm absolute floor
    if ((idle * new_min_pct) / 100 < 100) {
      new_min_pct = static_cast<uint8_t>(std::min<int>((100 * 100) / idle + 1, 49));
    }
    uint8_t cur_max = zone->threshold->max_percentage.value_or(80);
    if (new_min_pct + 2 >= cur_max) {
      update_status_text("high obs cal: no detection window - check placement");
      calibration_status_reset_ts_ = millis();
      resume_sensor_task_after_calibration();
      return;
    }

    zone->set_threshold_percentages(new_min_pct, cur_max);
    if (calibration_persistence_) {
      if (z == 0) entry_min_pct_pref_.save(&new_min_pct);
      else         exit_min_pct_pref_.save(&new_min_pct);
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "high obs cal: zone %d min -> %u%% (%dmm)", z, new_min_pct, new_min_mm);
    update_status_text(std::string(msg));
    ESP_LOGI(CALIBRATION, "zone %d high obs: avg=%dmm new_min=%dmm (%u%%)", z, obs_avg, new_min_mm, new_min_pct);
  }

  publish_sensor_configuration(entry, exit, false);
  if (entry_min_threshold_number_ != nullptr)
    entry_min_threshold_number_->publish_state(entry->threshold->min_percentage.value_or(15));
  if (exit_min_threshold_number_ != nullptr)
    exit_min_threshold_number_->publish_state(exit->threshold->min_percentage.value_or(15));

  update_status_text("high obs cal: done");
  calibration_status_reset_ts_ = millis();
  resume_sensor_task_after_calibration();
}

// ---- Entity control callbacks ----

void FilterModeSelect::control(const std::string &value) {
  FilterMode mode = FILTER_MIN;
  if (value == "median") mode = FILTER_MEDIAN;
  else if (value == "percentile10") mode = FILTER_PERCENTILE10;
  hub_->apply_filter_mode(mode);
}

void RangingModeSelect::control(const std::string &value) {
  hub_->apply_ranging_mode(value);
}

void RoodeSettingNumber::control(float value) {
  switch (setting_) {
    case FILTER_WINDOW:         hub_->apply_filter_window(static_cast<uint8_t>(value)); break;
    case SAMPLING:              hub_->apply_sampling(static_cast<uint8_t>(value)); break;
    case ENTRY_MAX_PCT:         hub_->apply_entry_max_threshold_pct(static_cast<uint8_t>(value)); break;
    case ENTRY_MIN_PCT:         hub_->apply_entry_min_threshold_pct(static_cast<uint8_t>(value)); break;
    case EXIT_MAX_PCT:          hub_->apply_exit_max_threshold_pct(static_cast<uint8_t>(value)); break;
    case EXIT_MIN_PCT:          hub_->apply_exit_min_threshold_pct(static_cast<uint8_t>(value)); break;
    case AUTO_CAL_INTERVAL_HOURS: hub_->apply_auto_calibration_interval(value); break;
    case ENTRY_ROI_HEIGHT:      hub_->apply_entry_roi(static_cast<uint8_t>(value), hub_->entry->roi->width); break;
    case ENTRY_ROI_WIDTH:       hub_->apply_entry_roi(hub_->entry->roi->height, static_cast<uint8_t>(value)); break;
    case EXIT_ROI_HEIGHT:       hub_->apply_exit_roi(static_cast<uint8_t>(value), hub_->exit->roi->width); break;
    case EXIT_ROI_WIDTH:        hub_->apply_exit_roi(hub_->exit->roi->height, static_cast<uint8_t>(value)); break;
  }
  publish_state(value);
}

void InvertDirectionSwitch::write_state(bool state) {
  hub_->apply_invert_direction(state);
  publish_state(state);
}

void CalibrationPersistenceSwitch::write_state(bool state) {
  hub_->apply_calibration_persistence(state);
  publish_state(state);
}

void Roode::restart_sensor() {
  uint32_t now = millis();
  if (now - last_sensor_restart_ts_ > restart_backoff_ms_) {
    // No restart was needed for the full backoff window — reset the counter
    restart_attempt_count_ = 0;
    restart_backoff_ms_ = restart_timeout_ms_;
  }
  restart_attempt_count_++;
  ESP_LOGW(TAG, "sensor_restart_attempt_%u (backoff=%us)", restart_attempt_count_, restart_backoff_ms_ / 1000);
  log_event(std::string("sensor_restart_attempt_") + std::to_string(restart_attempt_count_));
  distanceSensor->restart();
  last_sensor_restart_ts_ = now;
  invalid_read_count_ = 0;
  // Double the backoff for the next attempt, capped at 120s
  restart_backoff_ms_ = std::min(restart_backoff_ms_ * 2, static_cast<uint32_t>(120000));
  if (restart_attempt_count_ >= max_restart_attempts_) {
    ESP_LOGE(TAG, "sensor_restart_escalating_reset");
    log_event("sensor_restart_escalating_reset");
    esp_restart();
  }
}

void Roode::sensor_task(void *param) {
  auto *self = static_cast<Roode *>(param);
  // Register this task with the watchdog when running on ESP32
#ifdef USE_ESP32
  esp_task_wdt_add(nullptr);
#endif
  for (;;) {
#ifdef USE_ESP32
    // Feed the watchdog to prevent unwanted resets
    esp_task_wdt_reset();
#endif
    self->use_sensor_task_ = true;
    // Yield the sensor bus to Core 0 whenever a calibration is in progress.
    // vTaskDelay keeps the watchdog fed via esp_task_wdt_reset() on the next iteration.
    if (self->calibration_in_progress_) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    // Mark the entire iteration as busy — this covers readDistance(), path_tracking()
    // (which can call run_zone_calibration()), and the auto-calibrate_zones() block.
    // Core 0 spin-waits on this flag before starting any calibration so that all
    // sensor access paths on Core 1 are included, not just the readDistance() call.
    self->sensor_task_reading_ = true;
    uint32_t now = millis();
    if (self->last_loop_update_ts_ != 0 && (now - self->last_loop_update_ts_ > self->restart_timeout_ms_) &&
        (now - self->last_sensor_restart_ts_ > self->restart_backoff_ms_)) {
      ESP_LOGW(TAG, "Sensor unresponsive >%ds, restarting...", self->restart_timeout_ms_ / 1000);
      self->restart_sensor();
    }
    unsigned long start = micros();
    VL53L1_Error status = self->current_zone->readDistance(self->distanceSensor);
    if (status == VL53L1_ERROR_NONE) {
      self->last_loop_update_ts_ = millis();
      self->restart_backoff_ms_ = self->restart_timeout_ms_;  // reset backoff on successful read
    }
    uint16_t dist = self->current_zone->getDistance();
    if (status == VL53L1_ERROR_NONE && (dist == 0 || dist > 4000)) {
      self->invalid_read_count_++;
    } else {
      self->invalid_read_count_ = 0;
    }
    // Similar recovery check for the asynchronous sensor task
    if (self->invalid_read_count_ > self->invalid_distance_limit_ &&
        (now - self->last_sensor_restart_ts_ > self->restart_backoff_ms_)) {
      ESP_LOGW(TAG, "Consecutive invalid distances, restarting...");
      self->restart_sensor();
    }
    bool zone_trig = self->current_zone->getMinDistance() < self->current_zone->threshold->max &&
                     self->current_zone->getMinDistance() > self->current_zone->threshold->min;
    bool zone_was_active = self->zone_active_prev_[self->current_zone->id];
    self->zone_active_prev_[self->current_zone->id] = zone_trig;
    if (!self->cpu_optimizations_active_ || zone_trig || zone_was_active)
      self->path_tracking(self->current_zone);
    self->handle_sensor_status();
    self->current_zone = self->current_zone == self->entry ? self->exit : self->entry;
    unsigned long end = micros();
    unsigned long delta = end - start;
    self->loop_time_sum_ += delta;
    self->loop_count_++;
    self->update_metrics();
    // Run periodic auto-calibration from the sensor task (the main loop() returns early in dual-core
    // mode, so calibration must be triggered here to actually execute on ESP32).
    // Guard: epoch > 100000 ensures NTP is synced (time(nullptr) returns 0 until then, which would
    // cause uint32_t underflow and fire calibration spuriously on every boot before NTP sync).
    // Guard: skip entirely if Core 0 has already claimed the bus for a manual calibration.
    uint32_t now_epoch = static_cast<uint32_t>(time(nullptr));
    if (!self->calibration_in_progress_ &&
        self->auto_calibration_interval_sec_ > 0 && now_epoch > 100000 &&
        now_epoch - self->last_calibration_ts_ >= self->auto_calibration_interval_sec_) {
      // Only calibrate when both zones are clear — calibrating with someone present corrupts the baseline.
      // Use debounced state — a momentary raw reading above threshold->max does not
      // mean the zone is truly empty (noise can produce false clearances).
      bool zones_clear = !self->zone_debounced_active_[0] && !self->zone_debounced_active_[1];
      if (zones_clear) {
        ESP_LOGI(TAG, "auto_calibration_running");
        // sensor_task_reading_ remains true during auto-cal so that Core 0's
        // suspend_sensor_task_for_calibration() correctly waits rather than proceeding
        // concurrently.  calibrate_zones() checks calibration_in_progress_ after each
        // slow phase and returns early if Core 0 claims the bus mid-calibration.
        self->calibrate_zones();
      } else {
        ESP_LOGD(TAG, "auto_calibration_deferred: zones occupied");
      }
    }
    // Release the sensor bus — Core 0 may now start calibration on next check.
    self->sensor_task_reading_ = false;
    vTaskDelay(pdMS_TO_TICKS(self->polling_interval_ms_));
  }
}
}  // namespace roode
}  // namespace esphome
