#include "roode.h"
#include "Arduino.h"
#include <string>
#include <optional>
#include <vector>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <cmath>
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

namespace esphome {
namespace roode {

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
    if (msg == "dual_core_success" || msg == "fallback_single_core" || msg == "force_single_core" ||
        msg == "interrupt_fallback_polling" || msg == "interrupt_recovered") {
      instance_->publish_feature_list();
    }
  }
}

static float deg_to_rad(float d) { return d * M_PI / 180.0f; }
static float rad_to_deg(float r) { return r * 180.0f / M_PI; }

static int calc_sun_time(const struct tm &date, float lat, float lon, bool sunrise) {
  int n = date.tm_yday + 1;
  float lngHour = lon / 15.0f;
  float t = n + ((sunrise ? 6.0f : 18.0f) - lngHour) / 24.0f;
  float M = (0.9856f * t) - 3.289f;
  float L = M + (1.916f * sin(deg_to_rad(M))) + (0.020f * sin(2 * deg_to_rad(M))) + 282.634f;
  while (L < 0)
    L += 360.0f;
  while (L >= 360)
    L -= 360.0f;
  float RA = rad_to_deg(atan(0.91764f * tan(deg_to_rad(L))));
  while (RA < 0)
    RA += 360.0f;
  while (RA >= 360)
    RA -= 360.0f;
  float Lquadrant = floor(L / 90.0f) * 90.0f;
  float RAquadrant = floor(RA / 90.0f) * 90.0f;
  RA = RA + (Lquadrant - RAquadrant);
  RA /= 15.0f;
  float sinDec = 0.39782f * sin(deg_to_rad(L));
  float cosDec = cos(asin(sinDec));
  float cosH = (cos(deg_to_rad(90.833f)) - sinDec * sin(deg_to_rad(lat))) / (cosDec * cos(deg_to_rad(lat)));
  if (cosH > 1.0f || cosH < -1.0f)
    return sunrise ? 21600 : 64800;  // fallback to default 6AM/6PM
  float H = sunrise ? 360.0f - rad_to_deg(acos(cosH)) : rad_to_deg(acos(cosH));
  H /= 15.0f;
  float T = H + RA - (0.06571f * t) - 6.622f;
  float UT = T - lngHour;
  while (UT < 0)
    UT += 24.0f;
  while (UT >= 24)
    UT -= 24.0f;
  return static_cast<int>(UT * 3600.0f);
}

static bool parse_ha_time(const std::string &value, int &out_sec) {
  int yy, MM, dd, hh, mm, ss;
  if (sscanf(value.c_str(), "%d-%d-%dT%d:%d:%d", &yy, &MM, &dd, &hh, &mm, &ss) < 6)
    return false;
  out_sec = hh * 3600 + mm * 60 + ss;
  return true;
}

bool Roode::fetch_sun_times_from_ha(int &rise, int &set) {
#ifdef USE_API
  // The public API for retrieving Home Assistant state has changed across
  // ESPHome releases. The previous implementation relied on
  // api::APIServer::get_state(), but newer versions removed that method.
  // To remain compatible we currently just return false here which causes
  // the caller to fall back to the internally calculated sunrise and sunset
  // times or the default values. This keeps compilation working on all
  // supported ESPHome versions without requiring the deprecated API.
  (void) rise;
  (void) set;
  return false;
#else
  (void) rise;
  (void) set;
  return false;
#endif
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
  if (sampling_size_number != nullptr)
    sampling_size_number->publish_state(samples);
  if (log_fallback_switch_ != nullptr)
    log_fallback_switch_->publish_state(log_fallback_events_);

  publish_feature_list();
  last_recalibrate_ts_ = static_cast<uint32_t>(time(nullptr));
  last_auto_recalibrate_ts_ = last_recalibrate_ts_;
  update_sun_times();
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
  if (log_fallback_switch_ != nullptr &&
      log_fallback_switch_->state != log_fallback_events_) {
    set_log_fallback_events(log_fallback_switch_->state);
  }
  if (sampling_size_number != nullptr &&
      fabs(sampling_size_number->state - samples) > 0.001f) {
    set_sampling_size(static_cast<uint8_t>(sampling_size_number->state));
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
  sample_lux();
  adjust_filtering();
  check_context_calibration();
  check_multicore_recovery();
  check_auto_recalibration();
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

  ESP_LOGV(TAG, "Zone %d distance %u (min=%u max=%u)", zone->id, zone->getMinDistance(), zone->threshold->min,
           zone->threshold->max);

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
  } else if (zone_triggered_start_[zone->id] != 0 && millis() - zone_triggered_start_[zone->id] >= 10000 &&
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
          if (!should_suppress_event())
            this->updateCounter(-1);
          last_valid_crossing_ts_ = millis();
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Exit");
          }
        } else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an entry
          ESP_LOGI("Roode pathTracking", "Entry detected.");
          if (!should_suppress_event())
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
void Roode::recalibration() { perform_recalibration(true); }

void Roode::perform_recalibration(bool manual) {
  calibrate_zones();
  last_recalibrate_ts_ = static_cast<uint32_t>(time(nullptr));
  if (!manual)
    last_auto_recalibrate_ts_ = last_recalibrate_ts_;
  if (manual)
    log_event("manual_recalibrate_triggered");
}

void Roode::check_auto_recalibration() {
  uint32_t now = static_cast<uint32_t>(time(nullptr));
  if (validate_temperature_sensor()) {
    float t = current_temperature_;
    if (!isnan(last_temperature_) && fabs(t - last_temperature_) >= max_temp_delta_for_recalib_) {
      if (now - last_recalibrate_ts_ < recalibrate_cooldown_sec_) {
        log_event("recalibrate_cooldown_active");
      } else {
        log_event("temp_triggered_recalibration");
        perform_recalibration(false);
      }
      last_temperature_ = t;
      return;
    }
    if (isnan(last_temperature_))
      last_temperature_ = t;
  }

  if (idle_recalib_interval_sec_ > 0 && now - last_valid_crossing_ts_ >= idle_recalib_interval_sec_) {
    if (now - last_recalibrate_ts_ < recalibrate_cooldown_sec_) {
      log_event("recalibrate_cooldown_active");
    } else {
      log_event("idle_triggered_recalibration");
      perform_recalibration(false);
    }
    last_valid_crossing_ts_ = now;
    return;
  }

  if (auto_recalibrate_interval_sec_ == 0)
    return;
  if (now - last_auto_recalibrate_ts_ < auto_recalibrate_interval_sec_)
    return;
  if (now - last_recalibrate_ts_ < recalibrate_cooldown_sec_) {
    log_event("recalibrate_cooldown_active");
    return;
  }
  log_event("auto_recalibrate_interval");
  perform_recalibration(false);
}

void Roode::run_zone_calibration(uint8_t zone_id) {
  ESP_LOGI(CALIBRATION, "Fail safe calibration triggered for zone %d", zone_id);
  Zone *z = zone_id == 0 ? entry : exit;
  z->reset_roi(zone_id == 0 ? (orientation_ == Parallel ? 167 : 195) : (orientation_ == Parallel ? 231 : 60));
  z->calibrateThreshold(distanceSensor, 50);
  // Recalculate ROI sizes so thresholds remain consistent
  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  distanceSensor->set_ranging_mode(mode);

  calibration_data_[zone_id].baseline_mm = z->threshold->idle;
  calibration_data_[zone_id].threshold_min_mm = z->threshold->min;
  calibration_data_[zone_id].threshold_max_mm = z->threshold->max;
  calibration_data_[zone_id].last_calibrated_ts = static_cast<uint32_t>(time(nullptr));
  if (calibration_persistence_) {
    calibration_prefs_[zone_id].save(&calibration_data_[zone_id]);
  }

  // Publish the updated calibration data so Home Assistant sees the new
  // thresholds and ROI values immediately after a fail-safe recalibration
  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);
  publish_feature_list();
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

void Roode::update_sun_times() {
  if (!use_sunrise_prediction_)
    return;
  time_t t = time(nullptr);
  struct tm tm_time;
  localtime_r(&t, &tm_time);
  if (tm_time.tm_yday == sun_times_day_)
    return;
  sun_times_day_ = tm_time.tm_yday;
  if (latitude_ != 0.0f || longitude_ != 0.0f) {
    sunrise_sec_ = calc_sun_time(tm_time, latitude_, longitude_, true);
    sunset_sec_ = calc_sun_time(tm_time, latitude_, longitude_, false);
  } else if (!fetch_sun_times_from_ha(sunrise_sec_, sunset_sec_)) {
    sunrise_sec_ = 21600;
    sunset_sec_ = 64800;
  }
}

bool Roode::validate_lux_sensor() {
  if (!use_light_sensor_ || lux_sensor_ == nullptr)
    return false;
  uint32_t now = millis() / 1000;
  if (!lux_sensor_operational_) {
    if (now - lux_sensor_retry_ts_ < 1800)
      return false;
    float v = lux_sensor_->state;
    if (std::isnan(v)) {
      lux_sensor_retry_ts_ = now;
      return false;
    }
    lux_sensor_operational_ = true;
    current_lux_ = v;
    return true;
  }
  float v = lux_sensor_->state;
  if (std::isnan(v)) {
    log_event("lux_sensor_error");
    lux_sensor_operational_ = false;
    lux_sensor_retry_ts_ = now;
    return false;
  }
  current_lux_ = v;
  return true;
}

bool Roode::validate_temperature_sensor() {
  if (!recalibrate_on_temp_change_ || temperature_sensor_ == nullptr)
    return false;
  uint32_t now = static_cast<uint32_t>(time(nullptr));
  if (!temperature_sensor_operational_) {
    if (now - temperature_sensor_retry_ts_ < 1800)
      return false;
    float v = temperature_sensor_->state;
    if (isnan(v)) {
      temperature_sensor_retry_ts_ = now;
      return false;
    }
    temperature_sensor_operational_ = true;
    current_temperature_ = v;
    return true;
  }
  float v = temperature_sensor_->state;
  if (isnan(v)) {
    log_event("temperature_sensor_error");
    temperature_sensor_operational_ = false;
    temperature_sensor_retry_ts_ = now;
    return false;
  }
  current_temperature_ = v;
  return true;
}

void Roode::sample_lux() {
  if (!validate_lux_sensor())
    return;
  uint32_t now = millis() / 1000;
  if (now - last_lux_sample_ts_ < lux_sample_interval_sec_)
    return;
  last_lux_sample_ts_ = now;
  float lux = current_lux_;
  lux_samples_.push_back(lux);
  size_t max_samples = lux_learning_window_sec_ / lux_sample_interval_sec_;
  while (lux_samples_.size() > max_samples)
    lux_samples_.pop_front();
  if (lux_samples_.size() < 60) {
    log_event("lux_model_bootstrapping");
    return;
  }
  std::vector<float> tmp(lux_samples_.begin(), lux_samples_.end());
  std::sort(tmp.begin(), tmp.end());
  size_t idx = (size_t) (0.95f * (tmp.size() - 1));
  lux_percentile95_ = tmp[idx];
  if (!lux_learning_complete_) {
    lux_learning_complete_ = true;
    log_event("lux_learning_complete");
  }
}

bool Roode::should_suppress_event() {
  if (!validate_lux_sensor() || lux_samples_.empty()) {
    light_control_offset_ = 0;
    return false;
  }
  float lux = current_lux_;
  uint32_t now = millis() / 1000;
  if (now - last_suppression_ts_ < suppression_window_sec_)
    return true;
  update_sun_times();
  time_t t = time(nullptr);
  struct tm tm_time;
  localtime_r(&t, &tm_time);
  int sec_of_day = tm_time.tm_hour * 3600 + tm_time.tm_min * 60 + tm_time.tm_sec;
  bool time_window = use_sunrise_prediction_ && (abs(sec_of_day - sunrise_sec_) <= (int) suppression_window_sec_ ||
                                                 abs(sec_of_day - sunset_sec_) <= (int) suppression_window_sec_);
  float dynamic_multiplier = 1.0f;
  if (lux_percentile95_ > 0)
    dynamic_multiplier =
        esphome::clamp(1 + alpha_ * ((lux - lux_percentile95_) / lux_percentile95_), base_multiplier_, max_multiplier_);
  float multiplier = dynamic_multiplier;
  if (time_window)
    multiplier = std::max({dynamic_multiplier, time_multiplier_, combined_multiplier_});
  float threshold = lux_percentile95_ * multiplier;
  if (lux > threshold) {
    light_control_offset_ = lux - threshold;
    if (time_window)
      log_event("sunlight_suppressed_event");
    else
      log_event("lux_outlier_detected");
    last_suppression_ts_ = now;
    return true;
  }
  light_control_offset_ = 0;
  return false;
}

void Roode::adjust_filtering() {
  if (!validate_lux_sensor())
    return;
  float lux = current_lux_;
  if (lux_percentile95_ == 0)
    return;
  uint8_t new_win = filter_window_;
  if (lux > lux_percentile95_ * 4)
    new_win = std::min<uint8_t>(9, default_filter_window_ + 2);
  else if (lux < lux_percentile95_ * 2)
    new_win = default_filter_window_;
  if (new_win != filter_window_) {
    filter_window_ = new_win;
    entry->set_filter_window(new_win);
    exit->set_filter_window(new_win);
    log_event("filter_window_changed");
  }
}

void Roode::check_context_calibration() {
  uint32_t now = millis() / 1000;
  if (manual_adjustment_count_ == 1)
    manual_adjust_window_start_ = now;
  if (manual_adjustment_count_ > 5 && now - manual_adjust_window_start_ <= 3600 && validate_lux_sensor()) {
    if (current_lux_ > lux_percentile95_) {
      log_event("manual_recalibrate_triggered");
      perform_recalibration(false);
      manual_adjustment_count_ = 0;
    }
  }
  if (now - manual_adjust_window_start_ > 3600)
    manual_adjustment_count_ = 0;
}

void Roode::check_multicore_recovery() {
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!use_sensor_task_ && !force_single_core_) {
    uint32_t now = millis();
    if (now - last_multicore_retry_ms_ >= 300000) {
      last_multicore_retry_ms_ = now;
      BaseType_t res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
      if (res == pdPASS) {
        use_sensor_task_ = true;
        log_event("dual_core_recovered");
      } else {
        log_event("dual_core_fallback");
      }
    }
  }
#endif
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

  calibration_data_[0] = {entry->threshold->idle, entry->threshold->min, entry->threshold->max,
                          static_cast<uint32_t>(time(nullptr))};
  calibration_data_[1] = {exit->threshold->idle, exit->threshold->min, exit->threshold->max,
                          static_cast<uint32_t>(time(nullptr))};

  if (calibration_persistence_) {
    calibration_prefs_[0].save(&calibration_data_[0]);
    calibration_prefs_[1].save(&calibration_data_[1]);
  }
  ESP_LOGI(SETUP, "Finished calibrating sensor zones");
  publish_feature_list();
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

    const char *tz = getenv("TZ");
    bool use_utc = (tz == nullptr || tz[0] == '\0');

    if (use_utc) {
      if (!gmtime_r(&t, &tm_time))
        return std::string("unknown");
    } else {
      if (!localtime_r(&t, &tm_time))
        return std::string("unknown");
    }

    char buf[16];
    int hour = tm_time.tm_hour % 12;
    if (hour == 0)
      hour = 12;
    snprintf(buf, sizeof(buf), "%d:%02d%cM%s", hour, tm_time.tm_min,
             tm_time.tm_hour >= 12 ? 'P' : 'A', use_utc ? " (UTC)" : "");
    return std::string(buf);
  };

  std::vector<std::pair<std::string, std::string>> features;
#ifdef CONFIG_IDF_TARGET_ESP32
  features.push_back({"cpu_mode", use_sensor_task_ ? "dual" : "single"});
  features.push_back({"cpu", ESP.getChipModel()});
  features.push_back({"cpu_cores", std::to_string(ESP.getChipCores())});
#else
  features.push_back({"cpu_mode", "single"});
  features.push_back({"cpu", "ESP8266"});
  features.push_back({"cpu_cores", "1"});
#endif
  features.push_back({"xshut", distanceSensor->get_xshut_state().has_value() ? "enabled" : "disabled"});
  features.push_back({"refresh", distanceSensor->is_interrupt_enabled() ? "interrupt" : "polling"});
  features.push_back({"ram", fmt_bytes(ESP.getHeapSize())});
  features.push_back({"flash", fmt_bytes(ESP.getFlashChipSize())});
  features.push_back({"calibration_value", std::to_string(entry->threshold->idle)});
  uint32_t last_cal_epoch = std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
  features.push_back({"calibration", fmt_time(last_cal_epoch)});

  std::string light_ctrl;
  bool lux_enabled = use_light_sensor_ && lux_sensor_ != nullptr;
  bool loc_enabled = use_sunrise_prediction_ && latitude_ != 0 && longitude_ != 0;
  if (!lux_enabled && !loc_enabled)
    light_ctrl = "disabled";
  else if (lux_enabled && loc_enabled)
    light_ctrl = "both";
  else if (lux_enabled)
    light_ctrl = "lux";
  else
    light_ctrl = "location";
  features.push_back({"light_control", light_ctrl});
  features.push_back(
      {"temp_control", (recalibrate_on_temp_change_ && temperature_sensor_ != nullptr) ? "enabled" : "disabled"});
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", light_control_offset_);
  features.push_back({"light_control_status", buf});
  if (auto_recalibrate_interval_sec_ > 0) {
    uint32_t next_cal = last_auto_recalibrate_ts_ + auto_recalibrate_interval_sec_;
    features.push_back({"schedule_calibration", fmt_time(next_cal)});
  } else {
    features.push_back({"schedule_calibration", "disabled"});
  }

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
