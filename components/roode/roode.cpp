#include "roode.h"
#include "Arduino.h"
#include <string>
#include <optional>
#include <algorithm>
#include <time.h>
#include <cmath>
#define MAX_LUX_SAMPLES 1440

namespace esphome {
namespace roode {

// When disabled, fallback diagnostics are omitted from the log to reduce noise.
bool Roode::log_fallback_events_ = false;
Roode *Roode::instance_ = nullptr;
const uint32_t Roode::temp_startup_delay_ms_;
const uint32_t Roode::lux_startup_delay_ms_;
const uint32_t Roode::temp_retry_interval_ms_;
const uint32_t Roode::lux_retry_interval_ms_;

struct LuxPersist {
  uint16_t count;
  uint16_t index;
  std::array<Roode::LuxSample, MAX_LUX_SAMPLES> samples;
};

void Roode::load_lux_samples() {
  LuxPersist data{};
  if (lux_pref_.load(&data)) {
    lux_samples_.clear();
    for (uint16_t i = 0; i < data.count && i < MAX_LUX_SAMPLES; i++) {
      lux_samples_.push_back(data.samples[(data.index + i) % MAX_LUX_SAMPLES]);
    }
    if (lux_samples_.size() >= 60)
      lux_learning_complete_logged_ = true;
  }
}

void Roode::save_lux_samples() {
  LuxPersist data{};
  data.count = std::min<uint16_t>(lux_samples_.size(), MAX_LUX_SAMPLES);
  data.index = 0;
  for (uint16_t i = 0; i < data.count; i++) {
    data.samples[i] = lux_samples_[i];
  }
  lux_pref_.save(&data);
}

static float deg_to_rad(float deg) { return deg * M_PI / 180.0f; }
static float rad_to_deg(float rad) { return rad * 180.0f / M_PI; }

void Roode::update_sun_times() {
  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  int day = t.tm_yday + 1;
  float lngHour = longitude_ / 15.0f;
  auto calc = [&](bool sunrise) -> time_t {
    float t_est = day + ((sunrise ? 6.0f : 18.0f) - lngHour) / 24.0f;
    float M = (0.9856f * t_est) - 3.289f;
    float L = fmod(M + (1.916f * sin(deg_to_rad(M))) + (0.020f * sin(2 * deg_to_rad(M))) + 282.634f, 360.0f);
    float RA = rad_to_deg(atan(0.91764f * tan(deg_to_rad(L))));
    RA = fmod(RA + 360.0f, 360.0f);
    float Lquadrant  = floor(L / 90.0f) * 90.0f;
    float RAquadrant = floor(RA / 90.0f) * 90.0f;
    RA = RA + Lquadrant - RAquadrant;
    RA /= 15.0f;
    float sinDec = 0.39782f * sin(deg_to_rad(L));
    float cosDec = cos(asin(sinDec));
    float cosH = (cos(deg_to_rad(90.833f)) - (sinDec * sin(deg_to_rad(latitude_)))) /
                 (cosDec * cos(deg_to_rad(latitude_)));
    if (cosH > 1 && sunrise) return 0;  // no sunrise
    if (cosH < -1 && !sunrise) return 0; // no sunset
    float H = sunrise ? 360.0f - rad_to_deg(acos(cosH)) : rad_to_deg(acos(cosH));
    H /= 15.0f;
    float T = H + RA - (0.06571f * t_est) - 6.622f;
    float UT = fmod(T - lngHour, 24.0f);
    struct tm out = t;
    out.tm_hour = int(UT);
    out.tm_min = int((UT - int(UT)) * 60.0f);
    out.tm_sec = 0;
    return mktime(&out);
  };
  next_sunrise_ts_ = calc(true);
  next_sunset_ts_ = calc(false);
}

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
  } else if (msg.rfind("use_interrupt_mode_", 0) == 0) {
    std::string level = msg.substr(sizeof("use_interrupt_mode_") - 1);
    out += " - INT pin initial " + level;
    if (level == "low")
      out += "; waiting for HIGH";
    else
      out += "; waiting for LOW";
  } else if (msg == "use_interrupt_mode") {
    out += " - using interrupt mode";
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
  save_lux_samples();
}
void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Sample size: %d", samples);
  LOG_UPDATE_INTERVAL(this);
  entry->dump_config();
  exit->dump_config();
}

void Roode::setup() {
  instance_ = this;
  ESP_LOGI(SETUP, "Booting Roode %s", VERSION);
  if (version_sensor != nullptr) {
    version_sensor->publish_state(VERSION);
  }
  ESP_LOGI(SETUP, "Using sampling with sampling size: %d", samples);

  boot_ts_ = millis();
  lux_sensor_ready_ = false;
  temp_sensor_ready_ = false;
  last_lux_fail_ts_ = boot_ts_;
  last_temp_fail_ts_ = boot_ts_;

  lux_pref_ = global_preferences->make_preference<LuxPersist>(0xB0);
  load_lux_samples();
  if (use_sunrise_prediction_ && (latitude_ != 0 || longitude_ != 0))
    update_sun_times();


  if (this->distanceSensor->is_failed()) {
    this->mark_failed();
    ESP_LOGE(TAG, "Roode cannot be setup without a valid VL53L1X sensor");
    return;
  }
  if (lux_sensor_ != nullptr && lux_sensor_->is_failed()) {
    ESP_LOGW(TAG, "Lux sensor failed to initialize, disabling lux features");
    lux_sensor_ = nullptr;
    use_light_sensor_ = false;
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
  if (temperature_sensor_ != nullptr) {
    float t = temperature_sensor_->state;
    if (!isnan(t)) {
      baseline_temp_ = t;
      temp_sensor_ready_ = true;
    } else {
      temp_sensor_ready_ = false;
      last_temp_fail_ts_ = millis();
      log_event("temp_read_failed");
    }
  }
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!force_single_core_) {
    log_event("use_dual_core");
    vTaskDelay(pdMS_TO_TICKS(200));
    BaseType_t res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 12288, this, 1, &sensor_task_handle_, 1);
    multicore_retry_count_ = 0;
    while (res != pdPASS && multicore_retry_count_ < 2) {
      multicore_retry_count_++;
      log_event(std::string("retry_multicore_") + std::to_string(multicore_retry_count_));
      vTaskDelay(pdMS_TO_TICKS(200));
      res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 12288, this, 1, &sensor_task_handle_, 1);
    }
    if (res == pdPASS) {
      use_sensor_task_ = true;
      log_event("dual_core_success");
      if (multicore_failed_) {
        log_event("dual_core_recovered");
        multicore_failed_ = false;
      }
    } else {
      log_event("dual_core_failed");
      multicore_failed_ = true;
      last_multicore_retry_ts_ = millis();
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
  if (people_counter != nullptr)
    expected_counter_ = people_counter->state;

  std::string feature_list;
#ifdef CONFIG_IDF_TARGET_ESP32
  feature_list += use_sensor_task_ ? "dual_core," : "single_core,";
#else
  feature_list += "single_core,";
#endif
  feature_list += distanceSensor->get_xshut_state().has_value() ? "xshut," : "no_xshut,";
  feature_list += distanceSensor->get_interrupt_state().has_value() ? "interrupt," : "polling,";
  feature_list += "Ram " + std::to_string(ESP.getHeapSize() / 1024) + "k,";
  feature_list += "Flash " + std::to_string(ESP.getFlashChipSize() / 1024 / 1024) + "M,";
  feature_list += "Cores " + std::to_string(ESP.getChipCores()) + ",";
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
  if (people_counter != nullptr && fabs(people_counter->state - expected_counter_) > 0.001f) {
    int diff = (int) roundf(people_counter->state - expected_counter_);
    manual_adjustment_count_ += abs(diff);
    expected_counter_ = people_counter->state;
    if (manual_adjustment_sensor != nullptr)
      manual_adjustment_sensor->publish_state(manual_adjustment_count_);
    if (diff != 0) {
      std::string sign = diff > 0 ? "+" : "";
      log_event("manual_adjust " + sign + std::to_string(diff) + " total=" + std::to_string(manual_adjustment_count_));
      manual_adjust_timestamps_.push_back(millis());
    }
  }
  if (lux_sensor_ != nullptr && !lux_sensor_->is_failed()) {
    uint32_t now = millis();
    if (!lux_sensor_ready_) {
      if (now - boot_ts_ >= lux_startup_delay_ms_ &&
          now - last_lux_fail_ts_ >= lux_retry_interval_ms_) {
        float val = lux_sensor_->state;
        if (!isnan(val)) {
          lux_sensor_ready_ = true;
          last_lux_sample_ts_ = now;
          lux_samples_.push_back({now, val});
          save_lux_samples();
        } else {
          last_lux_fail_ts_ = now;
          log_event("lux_read_failed");
        }
      }
    } else {
      if (now - last_lux_sample_ts_ >= lux_sample_interval_ms_) {
        float val = lux_sensor_->state;
        if (!isnan(val)) {
          lux_samples_.push_back({now, val});
          last_lux_sample_ts_ = now;
          save_lux_samples();
          if (lux_samples_.size() < 60 && !lux_bootstrap_logged_) {
            lux_bootstrap_logged_ = true;
            log_event("lux_model_bootstrapping");
          } else if (lux_samples_.size() == 60 && !lux_learning_complete_logged_) {
            lux_learning_complete_logged_ = true;
            log_event("lux_learning_complete");
          }
        } else {
          lux_sensor_ready_ = false;
          last_lux_fail_ts_ = now;
          log_event("lux_read_failed");
        }
      }
      while (!lux_samples_.empty() &&
             now - lux_samples_.front().ts > lux_learning_window_ms_) {
        lux_samples_.erase(lux_samples_.begin());
      }
      if (!lux_samples_.empty())
        save_lux_samples();
    }
  } else if (lux_sensor_ != nullptr && lux_sensor_->is_failed()) {
    log_event("lux_sensor_failed");
    lux_sensor_ = nullptr;
    use_light_sensor_ = false;
  }

  // context aware calibration
  if (manual_adjust_timestamps_.size() > 0) {
    uint32_t now = millis();
    manual_adjust_timestamps_.erase(
        std::remove_if(manual_adjust_timestamps_.begin(), manual_adjust_timestamps_.end(),
                       [now](uint32_t ts) { return now - ts > 3600000; }),
        manual_adjust_timestamps_.end());
    if (manual_adjust_timestamps_.size() > 5 && lux_sensor_ != nullptr && lux_sensor_ready_) {
      float lux = lux_sensor_->state;
      float pct95 = 0;
      if (!lux_samples_.empty()) {
        std::vector<float> vals;
        vals.reserve(lux_samples_.size());
        for (auto &p : lux_samples_)
          vals.push_back(p.lux);
        std::sort(vals.begin(), vals.end());
        pct95 = vals[(size_t) (vals.size() * 0.95f)];
      }
      if (lux > pct95 && now - last_recalibration_ts_ > recalibrate_cooldown_ms_) {
        log_event("manual_recalibrate_triggered");
        calibrate_zones();
        manual_adjust_timestamps_.clear();
        last_recalibration_ts_ = now;
      }
    }
  }
}

void Roode::loop() {
  if (!use_sensor_task_ && !force_single_core_ && multicore_failed_ &&
      millis() - last_multicore_retry_ts_ >= 300000) {
    last_multicore_retry_ts_ = millis();
    log_event("dual_core_fallback");
    BaseType_t res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 12288, this, 1, &sensor_task_handle_, 1);
    if (res == pdPASS) {
      use_sensor_task_ = true;
      log_event("dual_core_recovered");
      multicore_failed_ = false;
    }
  }
  if (use_sensor_task_) {
    // When running on dual core the sensor loop runs in a separate task
    // Skip execution from main loop
    return;
  }
  unsigned long start = micros();
  this->current_zone->readDistance(distanceSensor);
  bool zone_trig = current_zone->getMinDistance() < current_zone->threshold->max &&
                   current_zone->getMinDistance() > current_zone->threshold->min;
  if (zone_trig)
    record_motion_event();
  if (use_light_sensor_ && lux_sensor_ != nullptr && !lux_sensor_->is_failed() && lux_sensor_ready_ && !lux_samples_.empty()) {
    std::vector<float> vals;
    vals.reserve(lux_samples_.size());
    for (auto &p : lux_samples_)
      vals.push_back(p.lux);
    std::sort(vals.begin(), vals.end());
    float pct95 = vals[(size_t)(vals.size() * 0.95f)];
    float lux = lux_sensor_->state;
    if (isnan(lux)) {
      lux_sensor_ready_ = false;
      last_lux_fail_ts_ = millis();
      log_event("lux_read_failed");
    } else {
    float dynamic_multiplier = 1 + alpha_ * ((lux - pct95) / pct95);
    if (dynamic_multiplier < base_multiplier_)
      dynamic_multiplier = base_multiplier_;
    if (dynamic_multiplier > max_multiplier_)
      dynamic_multiplier = max_multiplier_;
    bool in_time_win = false;
    if (use_sunrise_prediction_ && next_sunrise_ts_ != 0) {
      time_t now_sec = time(nullptr);
      in_time_win = (std::abs(now_sec - next_sunrise_ts_) * 1000 < sunrise_sunset_window_ms_) ||
                    (std::abs(now_sec - next_sunset_ts_) * 1000 < sunrise_sunset_window_ms_);
      if (now_sec - next_sunrise_ts_ > 86400 || now_sec - next_sunset_ts_ > 86400)
        update_sun_times();
    }
    float mult = dynamic_multiplier;
    if (in_time_win)
      mult = std::max(mult, time_multiplier_);
    if (lux > pct95 * mult) {
      zone_trig = false;
      sunlight_suppressed_until_ = millis();
      if (in_time_win)
        log_event("sunlight_suppressed_event");
      else
        log_event("lux_outlier_detected");
    } else if (lux > pct95 && millis() < sunlight_suppressed_until_ + suppression_window_ms_) {
      zone_trig = false;
      log_event("sunlight_suppressed_event");
    }
      apply_adaptive_filtering(lux, pct95);
    }
  }
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
  maybe_auto_recalibrate();
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
void Roode::recalibration() {
  log_event("manual_recalibrate_triggered");
  calibrate_zones();
  last_recalibration_ts_ = millis();
  if (temperature_sensor_ != nullptr) {
    float t = temperature_sensor_->state;
    if (!isnan(t)) {
      baseline_temp_ = t;
      temp_sensor_ready_ = true;
    } else {
      temp_sensor_ready_ = false;
      last_temp_fail_ts_ = millis();
      log_event("temp_read_failed");
    }
  }
}

void Roode::run_zone_calibration(uint8_t zone_id) {
  ESP_LOGI(CALIBRATION, "Fail safe calibration triggered for zone %d", zone_id);
  log_event("idle_triggered_recalibration");
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
  last_recalibration_ts_ = millis();
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

void Roode::maybe_auto_recalibrate() {
  if (auto_recalibrate_interval_ms_ == 0)
    return;
  uint32_t now = millis();
  if (now - last_recalibration_ts_ < auto_recalibrate_interval_ms_)
    return;
  if (recalibrate_on_temp_change_ && temperature_sensor_ != nullptr && millis() - boot_ts_ >= temp_startup_delay_ms_) {
    if (!temp_sensor_ready_) {
      if (millis() - last_temp_fail_ts_ >= temp_retry_interval_ms_) {
        float cur_attempt = temperature_sensor_->state;
        if (!isnan(cur_attempt)) {
          baseline_temp_ = cur_attempt;
          temp_sensor_ready_ = true;
        } else {
          last_temp_fail_ts_ = millis();
          log_event("temp_read_failed");
        }
      }
    }
    float cur = temperature_sensor_->state;
    if (isnan(cur)) {
      temp_sensor_ready_ = false;
      last_temp_fail_ts_ = millis();
      log_event("temp_read_failed");
    } else if (fabs(cur - baseline_temp_) >= max_temp_delta_for_recalib_) {
      if (now - last_auto_recalib_ts_ < recalibrate_cooldown_ms_) {
        log_event("recalibrate_cooldown_active");
        return;
      }
      log_event("temp_triggered_recalibration");
      calibrate_zones();
      baseline_temp_ = cur;
      last_recalibration_ts_ = now;
      last_auto_recalib_ts_ = now;
      return;
    }
  }
  if (now - last_auto_recalib_ts_ < recalibrate_cooldown_ms_) {
    log_event("recalibrate_cooldown_active");
    return;
  }
  log_event("auto_recalibrate_interval");
  calibrate_zones();
  last_recalibration_ts_ = now;
  last_auto_recalib_ts_ = now;
}

void Roode::record_int_fallback() {
  uint32_t now = millis();
  if (int_fallback_window_start_ == 0 || now - int_fallback_window_start_ > 1800000) {
    int_fallback_window_start_ = now;
    int_fallback_count_ = 0;
  }
  int_fallback_count_++;
  if (int_fallback_count_ >= 3) {
    int_fallback_count_ = 0;
    log_event("interrupt_fallback");
    if (distanceSensor != nullptr) {
      distanceSensor->recheck_features();
      if (!distanceSensor->get_interrupt_state().has_value() &&
          distanceSensor->get_xshut_state().has_value()) {
        distanceSensor->reset_via_xshut();
        distanceSensor->recheck_features();
      }
    }
  }
}

void Roode::record_motion_event() {
  uint32_t now = millis();
  motion_events_.push_back(now);
  while (!motion_events_.empty() && now - motion_events_.front() > 60000) {
    motion_events_.erase(motion_events_.begin());
  }
}

void Roode::apply_adaptive_filtering(float lux, float pct95) {
  uint8_t new_window = default_filter_window_;
  if (lux_samples_.size() >= 60) {
    if (lux > pct95 * 6)
      new_window = std::min<uint8_t>(default_filter_window_ + 4, Zone::MAX_BUFFER_SIZE);
    else if (lux > pct95 * 4)
      new_window = std::min<uint8_t>(default_filter_window_ + 2, Zone::MAX_BUFFER_SIZE);
  }
  size_t freq = motion_events_.size();
  if (freq > 20)
    new_window = std::min<uint8_t>(new_window + 2, Zone::MAX_BUFFER_SIZE);
  else if (freq > 10)
    new_window = std::min<uint8_t>(new_window + 1, Zone::MAX_BUFFER_SIZE);
  if (new_window != filter_window_) {
    filter_window_ = new_window;
    entry->set_filter_window(new_window);
    exit->set_filter_window(new_window);
    log_event("filter_window_changed");
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
    if (zone_trig)
      self->record_motion_event();
    if (self->lux_sensor_ != nullptr && !self->lux_sensor_->is_failed()) {
      uint32_t now = millis();
      if (!self->lux_sensor_ready_) {
        if (now - self->boot_ts_ >= self->lux_startup_delay_ms_ &&
            now - self->last_lux_fail_ts_ >= self->lux_retry_interval_ms_) {
          float val = self->lux_sensor_->state;
          if (!isnan(val)) {
            self->lux_sensor_ready_ = true;
            self->last_lux_sample_ts_ = now;
            self->lux_samples_.push_back({now, val});
            self->save_lux_samples();
          } else {
            self->last_lux_fail_ts_ = now;
            log_event("lux_read_failed");
          }
        }
      } else {
        if (now - self->last_lux_sample_ts_ >= self->lux_sample_interval_ms_) {
          float val = self->lux_sensor_->state;
          if (!isnan(val)) {
            self->lux_samples_.push_back({now, val});
            self->last_lux_sample_ts_ = now;
            self->save_lux_samples();
            if (self->lux_samples_.size() < 60 && !self->lux_bootstrap_logged_) {
              self->lux_bootstrap_logged_ = true;
              log_event("lux_model_bootstrapping");
            } else if (self->lux_samples_.size() == 60 && !self->lux_learning_complete_logged_) {
              self->lux_learning_complete_logged_ = true;
              log_event("lux_learning_complete");
            }
          } else {
            self->lux_sensor_ready_ = false;
            self->last_lux_fail_ts_ = now;
            log_event("lux_read_failed");
          }
        }
        while (!self->lux_samples_.empty() &&
               now - self->lux_samples_.front().ts > self->lux_learning_window_ms_) {
          self->lux_samples_.erase(self->lux_samples_.begin());
        }
        if (!self->lux_samples_.empty())
          self->save_lux_samples();
      }
    }
    if (self->use_light_sensor_ && self->lux_sensor_ != nullptr && !self->lux_sensor_->is_failed() && self->lux_sensor_ready_ && !self->lux_samples_.empty()) {
      std::vector<float> vals;
      vals.reserve(self->lux_samples_.size());
      for (auto &p : self->lux_samples_)
        vals.push_back(p.lux);
      std::sort(vals.begin(), vals.end());
      float pct95 = vals[(size_t)(vals.size() * 0.95f)];
      float lux = self->lux_sensor_->state;
      if (isnan(lux)) {
        self->lux_sensor_ready_ = false;
        self->last_lux_fail_ts_ = millis();
        log_event("lux_read_failed");
      } else {
      float dynamic_multiplier = 1 + self->alpha_ * ((lux - pct95) / pct95);
      if (dynamic_multiplier < self->base_multiplier_)
        dynamic_multiplier = self->base_multiplier_;
      if (dynamic_multiplier > self->max_multiplier_)
        dynamic_multiplier = self->max_multiplier_;
      bool in_time_win = false;
      if (self->use_sunrise_prediction_ && self->next_sunrise_ts_ != 0) {
        time_t now_sec = time(nullptr);
        in_time_win = (std::abs(now_sec - self->next_sunrise_ts_) * 1000 < self->sunrise_sunset_window_ms_) ||
                      (std::abs(now_sec - self->next_sunset_ts_) * 1000 < self->sunrise_sunset_window_ms_);
        if (now_sec - self->next_sunrise_ts_ > 86400 || now_sec - self->next_sunset_ts_ > 86400)
          self->update_sun_times();
      }
      float mult = dynamic_multiplier;
      if (in_time_win)
        mult = std::max(mult, self->time_multiplier_);
      if (lux > pct95 * mult) {
        zone_trig = false;
        self->sunlight_suppressed_until_ = millis();
        if (in_time_win)
          log_event("sunlight_suppressed_event");
        else
          log_event("lux_outlier_detected");
      } else if (lux > pct95 && millis() < self->sunlight_suppressed_until_ + self->suppression_window_ms_) {
        zone_trig = false;
        log_event("sunlight_suppressed_event");
      }
      self->apply_adaptive_filtering(lux, pct95);
      }
    }
    if (!self->cpu_optimizations_active_ || zone_trig)
      self->path_tracking(self->current_zone);
    self->handle_sensor_status();
    self->current_zone = self->current_zone == self->entry ? self->exit : self->entry;
    unsigned long end = micros();
    unsigned long delta = end - start;
    self->loop_time_sum_ += delta;
    self->loop_count_++;
    self->update_metrics();
    self->maybe_auto_recalibrate();
    vTaskDelay(pdMS_TO_TICKS(self->polling_interval_ms_));
  }
}
}  // namespace roode
}  // namespace esphome
