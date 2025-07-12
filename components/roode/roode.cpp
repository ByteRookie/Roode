#include "roode.h"

namespace esphome {
namespace roode {
void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Sample size: %d", samples);
  LOG_UPDATE_INTERVAL(this);
  entry->dump_config();
  exit->dump_config();
}

Roode::~Roode() {
#ifdef ESP32
  if (sensor_task_handle_ != nullptr) {
    vTaskDelete(sensor_task_handle_);
  }
  if (zone_queue_ != nullptr) {
    vQueueDelete(zone_queue_);
  }
#endif
  delete entry;
  delete exit;
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

  entry->init_pref(0xA0);
  exit->init_pref(0xA1);

#ifdef ESP32
  zone_queue_ = xQueueCreate(4, sizeof(ZoneMsg));
#if CONFIG_FREERTOS_UNICORE
  xTaskCreatePinnedToCore(Roode::sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 0);
#else
  xTaskCreatePinnedToCore(Roode::sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
#endif
#endif

  bool loaded = false;
  if (calibration_persistence_) {
    loaded = entry->load_calibration(distanceSensor) && exit->load_calibration(distanceSensor);
  }
  if (!loaded) {
    calibrate_zones();
    if (calibration_persistence_) {
      entry->save_calibration();
      exit->save_calibration();
    }
  } else {
    ESP_LOGI(SETUP, "Loaded calibration from storage");
  }

  diag_last_ts_ = millis();
}

void Roode::update() {
  if (distance_entry != nullptr) {
    distance_entry->publish_state(entry->getDistance());
  }
  if (distance_exit != nullptr) {
    distance_exit->publish_state(exit->getDistance());
  }
  if (loop_time_sensor != nullptr) {
    loop_time_sensor->publish_state(loop_time_ms_);
  }
  if (cpu_usage_sensor != nullptr) {
    cpu_usage_sensor->publish_state(cpu_usage_);
  }
  if (ram_free_sensor != nullptr) {
    uint32_t total = 0;
#ifdef ESP32
    total = ESP.getHeapSize();
#else
    total = 81920;
#endif
    uint32_t used = total - ESP.getFreeHeap();
    float pct = ((float)(total - used) * 100.0f) / (float) total;
    ram_free_sensor->publish_state(pct);
  }
  if (flash_free_sensor != nullptr) {
    uint32_t total = ESP.getFlashChipSize();
    uint32_t used = ESP.getSketchSize();
    float pct = ((float)(total - used) * 100.0f) / (float) total;
    flash_free_sensor->publish_state(pct);
  }
}

void Roode::loop() {
  uint32_t start_us = micros();
#ifdef ESP32
  ZoneMsg msg;
  while (xQueueReceive(zone_queue_, &msg, 0) == pdTRUE) {
    path_tracking();
    handle_sensor_status();
    check_fail_safe(msg.zone);
  }
#else
  current_zone->readDistance(distanceSensor);
  path_tracking();
  handle_sensor_status();
  check_fail_safe(current_zone);
  current_zone = current_zone == entry ? exit : entry;
#endif
  uint32_t end_us = micros();
  float loop_ms = (end_us - start_us) / 1000.0f;
  float cpu = 0;
  uint32_t now_ms = millis();
  if (last_loop_ts_ != 0) {
    uint32_t cycle = now_ms - last_loop_ts_;
    if (cycle > 0) {
      cpu = (loop_ms * 100.0f) / cycle;
    }
  }
  last_loop_ts_ = now_ms;

  loop_time_accum_ += loop_ms;
  cpu_usage_accum_ += cpu;
  diag_sample_count_++;
  if (now_ms - diag_last_ts_ >= 30000 && diag_sample_count_ > 0) {
    loop_time_ms_ = loop_time_accum_ / diag_sample_count_;
    cpu_usage_ = cpu_usage_accum_ / diag_sample_count_;
    loop_time_accum_ = 0;
    cpu_usage_accum_ = 0;
    diag_sample_count_ = 0;
    diag_last_ts_ = now_ms;
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

void Roode::path_tracking() {
  bool entry_active =
      entry->getMinDistance() < entry->threshold->max && entry->getMinDistance() > entry->threshold->min;
  bool exit_active = exit->getMinDistance() < exit->threshold->max && exit->getMinDistance() > exit->threshold->min;

  if (presence_sensor != nullptr) {
    presence_sensor->publish_state(entry_active || exit_active);
  }

  uint32_t now = millis();
  switch (fsm_state_) {
    case FSMState::IDLE:
      if (entry_active) {
        fsm_state_ = FSMState::ENTRY_STARTED;
        fsm_start_zone_ = entry;
        fsm_state_ts_ = now;
      } else if (exit_active) {
        fsm_state_ = FSMState::ENTRY_STARTED;
        fsm_start_zone_ = exit;
        fsm_state_ts_ = now;
      }
      break;
    case FSMState::ENTRY_STARTED:
      if (entry_active && exit_active) {
        fsm_state_ = FSMState::BOTH_ACTIVE;
        fsm_state_ts_ = now;
      } else if (now - fsm_state_ts_ > 2500) {
        ESP_LOGW(TAG, "fsm_timeout_reset");
        fsm_state_ = FSMState::IDLE;
      }
      break;
    case FSMState::BOTH_ACTIVE:
      if (!entry_active && !exit_active) {
        int delta = (fsm_start_zone_ == entry) ? 1 : -1;
        if (invert_direction_)
          delta = -delta;
        updateCounter(delta);
        if (entry_exit_event_sensor != nullptr) {
          entry_exit_event_sensor->publish_state(delta > 0 ? "Entry" : "Exit");
        }
        fsm_state_ = FSMState::IDLE;
      } else if (now - fsm_state_ts_ > 3500) {
        ESP_LOGW(TAG, "fsm_timeout_reset");
        fsm_state_ = FSMState::IDLE;
      }
      break;
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
  last_person_ts = millis();
}
void Roode::recalibration() { calibrate_zones(); }

void Roode::check_fail_safe(Zone *zone) {
  if (zone->getMinDistance() < zone->threshold->max && zone->getMinDistance() > zone->threshold->min) {
    zone->last_triggered_ts = millis();
  }
  if (millis() - zone->last_triggered_ts >= 10000 && millis() - last_person_ts >= 120000) {
    ESP_LOGW(TAG, "Fail-safe recalibration zone %d", zone->id);
    zone->run_zone_calibration(distanceSensor, orientation_);
    if (calibration_persistence_) {
      zone->save_calibration();
    }
    fail_safe_triggered = true;
  } else {
    fail_safe_triggered = false;
  }
}

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
  if (calibration_persistence_) {
    entry->save_calibration();
    exit->save_calibration();
  }
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

#ifdef ESP32
void Roode::sensor_task(void *param) {
  auto *self = static_cast<Roode *>(param);
  for (;;) {
    self->current_zone->readDistance(self->distanceSensor);
    ZoneMsg msg{self->current_zone};
    xQueueSend(self->zone_queue_, &msg, portMAX_DELAY);
    self->current_zone = self->current_zone == self->entry ? self->exit : self->entry;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
#endif
}  // namespace roode
}  // namespace esphome
