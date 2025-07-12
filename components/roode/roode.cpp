#include "roode.h"
#include "Arduino.h"
#include <array>

namespace esphome {
namespace roode {

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

  calibrate_zones();
  loop_window_start_ = millis();
  loop_time_sum_ = 0;
  loop_count_ = 0;
}

void Roode::update() {
  if (distance_entry != nullptr) {
    distance_entry->publish_state(entry->getDistance());
  }
  if (distance_exit != nullptr) {
    distance_exit->publish_state(exit->getDistance());
  }
}

void Roode::loop() {
  unsigned long start = micros();
  this->current_zone->readDistance(distanceSensor);
  uint16_t reading = this->current_zone->getMinDistance();
  uint8_t zid = this->current_zone->id;
  if (!zone_first_reading_[zid] && abs((int) last_zone_reading_[zid] - (int) reading) < 2) {
    ESP_LOGV(TAG, "Skipping duplicate reading for zone %d: %d", zid, reading);
  } else {
    path_tracking(this->current_zone);
    last_zone_reading_[zid] = reading;
    zone_first_reading_[zid] = false;
  }
  handle_sensor_status();
  this->current_zone = this->current_zone == this->entry ? this->exit : this->entry;
  // ESP_LOGI("Experimental", "Entry zone: %d, exit zone: %d",
  // entry->getDistance(Roode::distanceSensor, Roode::sensor_status),
  // exit->getDistance(Roode::distanceSensor, Roode::sensor_status)); unsigned
  unsigned long end = micros();
  unsigned long delta = end - start;
  loop_time_sum_ += delta;
  loop_count_++;

  uint32_t now = millis();
  if (now - loop_window_start_ >= 30000) {
    if (loop_count_ > 0) {
      float avg_ms = (float) loop_time_sum_ / loop_count_ / 1000.0f;
      if (loop_time_sensor != nullptr)
        loop_time_sensor->publish_state(avg_ms);
      float cpu = ((float) loop_time_sum_ / ((now - loop_window_start_) * 1000.0f)) * 100.0f;
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
    loop_time_sum_ = 0;
    loop_count_ = 0;
    loop_window_start_ = now;
  }

  if (idle_loop_throttle_) {
    bool motion = abs((int) entry->getMinDistance() - (int) last_zone_reading_[0]) >= zone_motion_threshold_mm_ ||
                  abs((int) exit->getMinDistance() - (int) last_zone_reading_[1]) >= zone_motion_threshold_mm_;
    if (!motion) {
      ESP_LOGV(TAG, "Idle throttling loop for %d ms", idle_loop_delay_ms_);
      delay(idle_loop_delay_ms_);
    }
  }
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
  static int zone_state[2] = {NOBODY, NOBODY};
  static int zone_candidate[2] = {NOBODY, NOBODY};
  static uint8_t zone_consistency[2] = {0, 0};

  bool is_left = zone == (this->invert_direction_ ? this->exit : this->entry);
  uint8_t idx = is_left ? 0 : 1;

  int measured_status = (zone->getMinDistance() < zone->threshold->max && zone->getMinDistance() > zone->threshold->min)
                            ? SOMEONE
                            : NOBODY;

  if (measured_status != zone_candidate[idx]) {
    zone_candidate[idx] = measured_status;
    zone_consistency[idx] = 1;
  } else if (zone_state[idx] != measured_status) {
    zone_consistency[idx]++;
  }

  bool state_changed = false;
  if (zone_state[idx] != measured_status) {
    if (zone_consistency[idx] >= fsm_consistency_window_) {
      ESP_LOGD(TAG, "Zone %d state stabilized to %s after %d readings", zone->id,
               measured_status == SOMEONE ? "SOMEONE" : "NOBODY", zone_consistency[idx]);
      zone_state[idx] = measured_status;
      zone_consistency[idx] = 0;
      state_changed = true;
    } else {
      ESP_LOGV(TAG, "Zone %d state %s not stable yet (%d/%d)", zone->id,
               measured_status == SOMEONE ? "SOMEONE" : "NOBODY", zone_consistency[idx], fsm_consistency_window_);
      return;
    }
  } else {
    zone_consistency[idx] = 0;
  }

  if (zone_state[0] == SOMEONE || zone_state[1] == SOMEONE) {
    if (presence_sensor != nullptr)
      presence_sensor->publish_state(true);
  }

  if (state_changed) {
    if (PathTrackFillingSize < 4)
      PathTrackFillingSize++;
    int AllZonesCurrentStatus = 0;
    if (zone_state[0] == SOMEONE)
      AllZonesCurrentStatus += 1;
    if (zone_state[1] == SOMEONE)
      AllZonesCurrentStatus += 2;
    ESP_LOGD(TAG, "Event has occured, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);
    PathTrack[PathTrackFillingSize - 1] = AllZonesCurrentStatus;

    if (zone_state[0] == NOBODY && zone_state[1] == NOBODY) {
      if (PathTrackFillingSize == 4) {
        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          ESP_LOGI("Roode pathTracking", "Exit detected. entry=%d exit=%d", entry->getMinDistance(),
                   exit->getMinDistance());
          this->updateCounter(-1);
          if (entry_exit_event_sensor != nullptr)
            entry_exit_event_sensor->publish_state("Exit");
        } else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          ESP_LOGI("Roode pathTracking", "Entry detected. entry=%d exit=%d", entry->getMinDistance(),
                   exit->getMinDistance());
          this->updateCounter(1);
          if (entry_exit_event_sensor != nullptr)
            entry_exit_event_sensor->publish_state("Entry");
        }
      }
      PathTrackFillingSize = 1;
    }
  }

  if (presence_sensor != nullptr) {
    if (zone_state[0] == NOBODY && zone_state[1] == NOBODY)
      presence_sensor->publish_state(false);
  }
}
void Roode::updateCounter(int delta) {
  if (this->people_counter == nullptr) {
    return;
  }
  auto next = this->people_counter->state + (float) delta;
  ESP_LOGI(TAG, "Updating people count: %d", (int) next);
  auto call = this->people_counter->make_call();
  call.set_value(next);
  call.perform();
}
void Roode::recalibration() { calibrate_zones(); }

const RangingMode *Roode::determine_raning_mode(uint16_t average_entry_zone_distance,
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
  entry->calibrateThreshold(distanceSensor, number_attempts);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->calibrateThreshold(distanceSensor, number_attempts);

  publish_sensor_configuration(entry, exit, true);
  App.feed_wdt();
  publish_sensor_configuration(entry, exit, false);
  ESP_LOGI(SETUP, "Finished calibrating sensor zones");
}

void Roode::calibrateDistance() {
  auto *const initial = distanceSensor->get_ranging_mode_override().value_or(Ranging::Longest);
  distanceSensor->set_ranging_mode(initial);

  entry->calibrateThreshold(distanceSensor, number_attempts);
  exit->calibrateThreshold(distanceSensor, number_attempts);

  if (distanceSensor->get_ranging_mode_override().has_value()) {
    return;
  }
  auto *mode = determine_raning_mode(entry->threshold->idle, exit->threshold->idle);
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
}  // namespace roode
}  // namespace esphome
