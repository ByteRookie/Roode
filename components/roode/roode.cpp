#include "roode.h"
#include "Arduino.h"
#include <string>
#include <optional>

namespace esphome {
namespace roode {

// When disabled, fallback diagnostics are omitted from the log to reduce noise.
bool Roode::log_fallback_events_ = false;
void Roode::log_event(const std::string &msg) {
  if (!log_fallback_events_) {
    if (msg == "interrupt_fallback" || msg == "interrupt_fallback_polling")
      return;
    if (msg == "int_pin_missed" || msg.rfind("int_pin_missed_sensor_", 0) == 0)
      return;
    if (msg == "xshut_toggled" || msg == "xshut_toggled_on" || msg == "xshut_toggled_off" ||
        msg == "xshut_pulse_off" || msg == "xshut_reinitialize" ||
        msg == "sensor.recovered_via_xshut" || msg.rfind("xshut_sensor_", 0) == 0 ||
        msg.rfind("xshut_pulse_off_sensor_", 0) == 0 || msg.rfind("xshut_reinitialize_sensor_", 0) == 0 ||
        (msg.rfind("sensor_", 0) == 0 && msg.find(".recovered_via_xshut") != std::string::npos))
      return;
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
    ESP_LOGE(TAG, "Roode cannot be setup without a valid VL53L1X sensor");
    return;
  }

  // Initialize filtering options before calibrating so threshold sampling uses
  // the configured window and mode
  entry->set_filter_window(filter_window_);
  entry->set_filter_mode(filter_mode_);
  exit->set_filter_window(filter_window_);
  exit->set_filter_mode(filter_mode_);

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
        int valid_count = 0;
        for (int s = 0; s < 5; s++) {
          z->readDistance(distanceSensor);
          if (abs((int)z->getDistance() - (int)z->threshold->idle) < (z->threshold->idle * 0.1))
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
    } else {
      calibrate_zones();
    }
  } else {
    calibrate_zones();
  }
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!force_single_core_) {
    log_event("use_dual_core");
    vTaskDelay(pdMS_TO_TICKS(200));
    BaseType_t res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
    multicore_retry_count_ = 0;
    while (res != pdPASS && multicore_retry_count_ < 2) {
      multicore_retry_count_++;
      log_event(std::string("retry_multicore_") + std::to_string(multicore_retry_count_));
      vTaskDelay(pdMS_TO_TICKS(200));
      res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
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

  std::string feature_list;
#ifdef CONFIG_IDF_TARGET_ESP32
  feature_list += use_sensor_task_ ? "dual_core," : "single_core,";
#else
  feature_list += "single_core,";
#endif
  feature_list += distanceSensor->get_xshut_state().has_value() ? "xshut," : "no_xshut,";
  feature_list += distanceSensor->is_interrupt_enabled() ? "interrupt," : "polling,";
  if (!feature_list.empty())
    feature_list.pop_back();
  if (enabled_features_sensor != nullptr)
    enabled_features_sensor->publish_state(feature_list);
  log_event(std::string("features_enabled: ") + feature_list);
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

void Roode::loop() {
  if (use_sensor_task_) {
    // When running on dual core the sensor loop runs in a separate task
    // Skip execution from main loop
    return;
  }
  unsigned long start = micros();
  this->current_zone->readDistance(distanceSensor);
  bool zone_trig = current_zone->getMinDistance() < current_zone->threshold->max &&
                   current_zone->getMinDistance() > current_zone->threshold->min;
  if (!cpu_optimizations_active_ || zone_trig)
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
  delay(polling_interval_ms_);
}

bool Roode::handle_sensor_status() {
  bool check_status = false;
  if (last_sensor_status != sensor_status && sensor_status == VL53L1_ERROR_NONE) {
    if (status_sensor != nullptr) {
      status_sensor->publish_state(sensor_status);
    }
    check_status = true;
  }
  if (sensor_status < 28 && sensor_status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Ranging failed with an error. status: %d", sensor_status);
    status_sensor->publish_state(sensor_status);
    check_status = false;
  }

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
    ESP_LOGW(TAG, "fsm_timeout_reset");
  }

  ESP_LOGV(TAG, "Zone %d distance %u (min=%u max=%u)", zone->id, zone->getMinDistance(),
           zone->threshold->min, zone->threshold->max);

  // PathTrack algorithm
  if (zone->getMinDistance() < zone->threshold->max && zone->getMinDistance() > zone->threshold->min) {
    // Someone is in the sensing area
    CurrentZoneStatus = SOMEONE;
    if (presence_sensor != nullptr) {
      presence_sensor->publish_state(true);
    }
    if (zone_triggered_start_[zone->id] == 0) {
      zone_triggered_start_[zone->id] = millis();
    }
  }
  if (CurrentZoneStatus == NOBODY) {
    zone_triggered_start_[zone->id] = 0;
  } else if (zone_triggered_start_[zone->id] != 0 &&
             millis() - zone_triggered_start_[zone->id] >= 10000 &&
             millis() - last_valid_crossing_ts_ >= 120000) {
    run_zone_calibration(zone->id);
    fail_safe_triggered_ = true;
    zone_triggered_start_[zone->id] = 0;
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

    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {
      ESP_LOGD(TAG, "Nobody anywhere, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1
      // 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is
        // always the case

        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // This an exit
          ESP_LOGI("Roode pathTracking", "Exit detected.");

          this->updateCounter(-1);
          last_valid_crossing_ts_ = millis();
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Exit");
          }
        } else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an entry
          ESP_LOGI("Roode pathTracking", "Entry detected.");
          this->updateCounter(1);
          last_valid_crossing_ts_ = millis();
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Entry");
          }
        }
      }

      PathTrackFillingSize = 1;
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
    if (CurrentZoneStatus == NOBODY && LeftPreviousStatus == NOBODY && RightPreviousStatus == NOBODY) {
      // nobody is in the sensing area
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
  expected_counter_ = next;
  auto call = this->people_counter->make_call();
  call.set_value(next);
  call.perform();
}
void Roode::recalibration() { calibrate_zones(); }

void Roode::run_zone_calibration(uint8_t zone_id) {
  ESP_LOGI(CALIBRATION, "Fail safe calibration triggered for zone %d", zone_id);
  Zone *z = zone_id == 0 ? entry : exit;
  z->reset_roi(zone_id == 0 ? (orientation_ == Parallel ? 167 : 195)
                             : (orientation_ == Parallel ? 231 : 60));
  z->calibrateThreshold(distanceSensor, 50);
  // Recalculate ROI sizes so thresholds remain consistent
  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  distanceSensor->set_ranging_mode(mode);

  calibration_data_[zone_id].baseline_mm = z->threshold->idle;
  calibration_data_[zone_id].threshold_min_mm = z->threshold->min;
  calibration_data_[zone_id].threshold_max_mm = z->threshold->max;
  calibration_data_[zone_id].last_calibrated_ts = millis();
  if (calibration_persistence_) {
    calibration_prefs_[zone_id].save(&calibration_data_[zone_id]);
  }

  // Publish the updated calibration data so Home Assistant sees the new
  // thresholds and ROI values immediately after a fail-safe recalibration
  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);
}

void Roode::apply_cpu_optimizations(float cpu) {
  if (cpu_optimizations_active_ || cpu <= 90.0f)
    return;
  ESP_LOGW(TAG, "CPU usage %.1f%% exceeded threshold, applying optimizations", cpu);
  polling_interval_ms_ = 30;
  filter_window_ = 3;
  entry->set_filter_window(3);
  exit->set_filter_window(3);
  filter_mode_ = FILTER_PERCENTILE10;
  entry->set_filter_mode(FILTER_PERCENTILE10);
  exit->set_filter_mode(FILTER_PERCENTILE10);
  cpu_optimizations_active_ = true;
}

void Roode::reset_cpu_optimizations(float cpu) {
  if (!cpu_optimizations_active_ || cpu > 50.0f)
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
    uint32_t total_heap = ESP.getHeapSize();
    float used_percent = 0;
    if (total_heap > 0) {
      uint32_t used = total_heap - ESP.getFreeHeap();
      used_percent = ((float) used / (float) total_heap) * 100.0f;
    }
    ram_free_sensor->publish_state(used_percent);
  }
  if (flash_free_sensor != nullptr) {
    uint32_t total_flash = ESP.getFlashChipSize();
    float used_percent = 0;
    if (total_flash > 0) {
      uint32_t used = total_flash - ESP.getFreeSketchSpace();
      used_percent = ((float) used / (float) total_flash) * 100.0f;
    }
    flash_free_sensor->publish_state(used_percent);
  }
  apply_cpu_optimizations(cpu);
  reset_cpu_optimizations(cpu);
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

  entry->reset_roi(orientation_ == Parallel ? 167 : 195);
  exit->reset_roi(orientation_ == Parallel ? 231 : 60);

  calibrateDistance();

  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  entry->calibrateThreshold(distanceSensor, 50);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->calibrateThreshold(distanceSensor, 50);

  publish_sensor_configuration(entry, exit, true);
  App.feed_wdt();
  publish_sensor_configuration(entry, exit, false);
  if (calibration_persistence_) {
    calibration_data_[0] = {entry->threshold->idle, entry->threshold->min, entry->threshold->max, millis()};
    calibration_data_[1] = {exit->threshold->idle, exit->threshold->min, exit->threshold->max, millis()};
    calibration_prefs_[0].save(&calibration_data_[0]);
    calibration_prefs_[1].save(&calibration_data_[1]);
  }
  ESP_LOGI(SETUP, "Finished calibrating sensor zones");
}

void Roode::calibrateDistance() {
  auto *const initial = distanceSensor->get_ranging_mode_override().value_or(Ranging::Longest);
  distanceSensor->set_ranging_mode(initial);

  entry->calibrateThreshold(distanceSensor, 50);
  exit->calibrateThreshold(distanceSensor, 50);

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

void Roode::sensor_task(void *param) {
  auto *self = static_cast<Roode *>(param);
  for (;;) {
    self->use_sensor_task_ = true;
    unsigned long start = micros();
    self->current_zone->readDistance(self->distanceSensor);
    bool zone_trig = self->current_zone->getMinDistance() < self->current_zone->threshold->max &&
                     self->current_zone->getMinDistance() > self->current_zone->threshold->min;
    if (!self->cpu_optimizations_active_ || zone_trig)
      self->path_tracking(self->current_zone);
    self->handle_sensor_status();
    self->current_zone = self->current_zone == self->entry ? self->exit : self->entry;
    unsigned long end = micros();
    unsigned long delta = end - start;
    self->loop_time_sum_ += delta;
    self->loop_count_++;
    self->update_metrics();
    vTaskDelay(pdMS_TO_TICKS(self->polling_interval_ms_));
  }
}
}  // namespace roode
}  // namespace esphome
