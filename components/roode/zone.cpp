#include "zone.h"
#include "roode.h"
#include <algorithm>

namespace esphome {
namespace roode {

Zone::~Zone() {
  delete roi;
  delete roi_override;
  delete threshold;
}

void Zone::dump_config() const {
  ESP_LOGCONFIG(TAG, "   %s", id == 0U ? "Entry" : "Exit");
  ESP_LOGCONFIG(TAG, "     ROI: { width: %d, height: %d, center: %d }", roi->width, roi->height, roi->center);
  ESP_LOGCONFIG(TAG, "     Threshold: { min: %dmm (%d%%), max: %dmm (%d%%), idle: %dmm }", threshold->min,
                threshold->min_percentage.value_or((threshold->min * 100) / threshold->idle), threshold->max,
                threshold->max_percentage.value_or((threshold->max * 100) / threshold->idle), threshold->idle);
}

VL53L1_Error Zone::readDistance(TofSensor *distanceSensor) {
  last_sensor_status = sensor_status;

  auto result = distanceSensor->read_distance(roi, sensor_status);
  uint16_t dist_to_filter;
  if (!result.has_value() || sensor_status != VL53L1_ERROR_NONE) {
    dist_to_filter = threshold->idle;
    if (debug_mode_) {
      ESP_LOGD(TAG, "Zone %d: read failed (status %d), falling back to idle %dmm", id, (int)sensor_status, dist_to_filter);
    }
  } else {
    last_distance = result.value();
    dist_to_filter = result.value();
  }

  // If the sensor is out of range, feed the baseline idle distance into the filter so it clears out
  if (dist_to_filter > 4000 || dist_to_filter == 0) {
    if (debug_mode_ && dist_to_filter == 0) {
      ESP_LOGD(TAG, "Zone %d: read 0mm, falling back to idle %dmm", id, threshold->idle);
    }
    dist_to_filter = threshold->idle;
  }

  samples[sample_idx_] = dist_to_filter;
  sample_idx_ = (sample_idx_ + 1) % max_samples;
  if (sample_count_ < max_samples)
    sample_count_++;

  std::array<uint16_t, MAX_BUFFER_SIZE> tmp;
  std::copy_n(samples.begin(), sample_count_, tmp.begin());
  std::sort(tmp.begin(), tmp.begin() + sample_count_);

  switch (filter_mode_) {
    case FILTER_MEDIAN:
      min_distance = tmp[sample_count_ / 2];
      break;
    case FILTER_PERCENTILE10: {
      uint8_t idx = (sample_count_ * 10) / 100;
      if (idx >= sample_count_)
        idx = sample_count_ - 1;
      min_distance = tmp[idx];
      break;
    }
    case FILTER_MIN:
    default:
      min_distance = tmp[0];
      break;
  }

  return sensor_status;
}

/**
 * This sets the ROI for the zone to the given overrides or the standard default.
 * This is needed to do initial calibration of thresholds & ROI.
 */
void Zone::reset_roi(uint8_t default_center) {
  // Apply overrides or defaults only if current ROI is not yet set (e.g. not loaded from flash)
  if (roi->width == 0) roi->width = roi_override->width ? roi_override->width : 6;
  if (roi->height == 0) roi->height = roi_override->height ? roi_override->height : 16;
  if (roi->center == 0) roi->center = roi_override->center ? roi_override->center : default_center;
  if (debug_mode_) {
    ESP_LOGD(TAG, "%s ROI current state: { width: %d, height: %d, center: %d }", id == 0U ? "Entry" : "Exit", roi->width,
             roi->height, roi->center);
  }
}

void Zone::calibrateThreshold(TofSensor *distanceSensor, int number_attempts) {
  if (debug_mode_) {
    ESP_LOGD(CALIBRATION, "Beginning. zoneId: %d", id);
  }
  std::vector<int> zone_distances;
  zone_distances.reserve(number_attempts);
  int sum = 0;
  for (int i = 0; i < number_attempts; i++) {
#ifdef CONFIG_IDF_TARGET_ESP32
    Roode::i2c_lock();
#endif
    this->readDistance(distanceSensor);
#ifdef CONFIG_IDF_TARGET_ESP32
    Roode::i2c_unlock();
#endif
    if (sensor_status != VL53L1_ERROR_NONE) {
      if (debug_mode_) {
        ESP_LOGW(CALIBRATION, "Distance read failed during calibration. status: %d", sensor_status);
      }
      continue;
    }
    uint16_t dist = this->getDistance();
    if (dist > 0 && dist <= 4000) {
      zone_distances.push_back(dist);
      sum += dist;
    }
  };
  // Require at least 70% valid readings (or minimum 10) to proceed with calibration
  size_t min_valid = std::max((size_t)10, (size_t)(number_attempts * 7 / 10));
  if (zone_distances.size() < min_valid) {
    if (debug_mode_) {
      ESP_LOGW(CALIBRATION, "Calibration failed: only %d valid distances recorded (needed %d). Retaining previous baseline.",
               zone_distances.size(), min_valid);
    }
  } else {
    int avg = sum / zone_distances.size();
    threshold->idle = avg;
    uint8_t max_pct = threshold->max_percentage.value_or(80);
    uint8_t min_pct = threshold->min_percentage.value_or(0);
    threshold->max_percentage = max_pct;
    threshold->min_percentage = min_pct;
    threshold->max = (avg * max_pct) / 100;
    threshold->min = (avg * min_pct) / 100;
    if (debug_mode_) {
      ESP_LOGI(CALIBRATION, "Calibrated threshold for zone. zoneId: %d, idle: %d, min: %d (%d%%), max: %d (%d%%)", id,
               threshold->idle, threshold->min,
               threshold->min_percentage.value_or((threshold->min * 100) / threshold->idle), threshold->max,
               threshold->max_percentage.value_or((threshold->max * 100) / threshold->idle));
    }
  }
}

void Zone::roi_calibration(uint16_t entry_threshold, uint16_t exit_threshold, Orientation orientation) {
  // the value of the average distance is used for computing the optimal size of the ROI and consequently also the
  // center of the two zones
  int function_of_the_distance = 16 * (1 - (0.15 * 2) / (0.34 * (min(entry_threshold, exit_threshold) / 1000.0f)));
  int ROI_size = min(8, max(4, function_of_the_distance));
  // Use the calculated ROI size unless an override has been specified
  this->roi->width = this->roi_override->width ? this->roi_override->width : ROI_size;
  this->roi->height = this->roi_override->height ? this->roi_override->height : ROI_size * 2;
  if (this->roi_override->center) {
    this->roi->center = this->roi_override->center;
  } else {
    // now we set the position of the center of the two zones
    if (orientation == Parallel) {
      switch (this->roi->width) {
        case 4:
          this->roi->center = this->id == 0U ? 150 : 247;
          break;
        case 5:
        case 6:
          this->roi->center = this->id == 0U ? 159 : 239;
          break;
        case 7:
        case 8:
          this->roi->center = this->id == 0U ? 167 : 231;
          break;
      }
    } else {
      switch (this->roi->width) {
        case 4:
          this->roi->center = this->id == 0U ? 193 : 58;
          break;
        case 5:
        case 6:
          this->roi->center = this->id == 0U ? 194 : 59;
          break;
        case 7:
        case 8:
          this->roi->center = this->id == 0U ? 195 : 60;
          break;
      }
    }
  }
  if (debug_mode_) {
    ESP_LOGI(CALIBRATION, "Calibrated ROI for zone. zoneId: %d, width: %d, height: %d, center: %d", id, roi->width,
             roi->height, roi->center);
  }
}

uint16_t Zone::getDistance() const { return this->last_distance; }
uint16_t Zone::getMinDistance() const { return this->min_distance; }

void Zone::set_threshold_percentages(uint8_t min_percent, uint8_t max_percent) {
  threshold->min_percentage = min_percent;
  threshold->max_percentage = max_percent;
  if (threshold->idle > 0) {
    threshold->min = (threshold->idle * min_percent) / 100;
    threshold->max = (threshold->idle * max_percent) / 100;
  }
}
}  // namespace roode
}  // namespace esphome
