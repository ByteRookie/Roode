#include "zone.h"
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
  if (!result.has_value()) {
    return sensor_status;
  }

  last_distance = result.value();
  if (sensor_status != VL53L1_ERROR_NONE || result.value() == 0 || result.value() > 4000) {
    return sensor_status;
  }

  samples[sample_idx_] = result.value();
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
  roi->width = roi_override->width ?: 6;
  roi->height = roi_override->height ?: 16;
  roi->center = roi_override->center ?: default_center;
  ESP_LOGD(TAG, "%s ROI reset: { width: %d, height: %d, center: %d }", id == 0U ? "Entry" : "Exit", roi->width,
           roi->height, roi->center);
}

void Zone::calibrateThreshold(TofSensor *distanceSensor, int number_attempts) {
  ESP_LOGD(CALIBRATION, "Beginning. zoneId: %d", id);
  std::vector<int> zone_distances;
  zone_distances.reserve(number_attempts);
  int sum = 0;
  for (int i = 0; i < number_attempts; i++) {
    this->readDistance(distanceSensor);
    if (sensor_status != VL53L1_ERROR_NONE) {
      ESP_LOGW(CALIBRATION, "Distance read failed during calibration. status: %d", sensor_status);
      break;
    }
    zone_distances.push_back(this->getDistance());
    sum += zone_distances.back();
  };
  if (zone_distances.empty()) {
    threshold->idle = 0;
    ESP_LOGW(CALIBRATION, "Calibration failed: no valid distances recorded");
  } else {
    int avg = sum / zone_distances.size();
    threshold->idle = avg;
    threshold->max = avg * 0.80;
    threshold->min = avg * 0.15;
  }

  ESP_LOGI(CALIBRATION, "Calibrated threshold for zone. zoneId: %d, idle: %d, min: %d (%d%%), max: %d (%d%%)", id,
           threshold->idle, threshold->min,
           threshold->min_percentage.value_or((threshold->min * 100) / threshold->idle), threshold->max,
           threshold->max_percentage.value_or((threshold->max * 100) / threshold->idle));
}

void Zone::roi_calibration(uint16_t entry_threshold, uint16_t exit_threshold, Orientation orientation) {
  // the value of the average distance is used for computing the optimal size of the ROI and consequently also the
  // center of the two zones
  int function_of_the_distance = 16 * (1 - (0.15 * 2) / (0.34 * (min(entry_threshold, exit_threshold) / 1000)));
  int ROI_size = min(8, max(4, function_of_the_distance));
  this->roi->width = this->roi_override->width ?: ROI_size;
  this->roi->height = this->roi_override->height ?: ROI_size * 2;
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
  ESP_LOGI(CALIBRATION, "Calibrated ROI for zone. zoneId: %d, width: %d, height: %d, center: %d", id, roi->width,
           roi->height, roi->center);
}

int Zone::getOptimizedValues(const std::vector<int> &values, int sum) const {
  int size = values.size();
  int sum_squared = 0;
  int variance = 0;
  int sd = 0;
  int avg = sum / size;

  for (int i = 0; i < size; i++) {
    sum_squared = sum_squared + (values[i] * values[i]);
    App.feed_wdt();
  }
  variance = sum_squared / size - (avg * avg);
  sd = sqrt(variance);
  ESP_LOGD(CALIBRATION, "Zone AVG: %d", avg);
  ESP_LOGD(CALIBRATION, "Zone SD: %d", sd);
  return avg - sd;
}

uint16_t Zone::getDistance() const { return this->last_distance; }
uint16_t Zone::getMinDistance() const { return this->min_distance; }
}  // namespace roode
}  // namespace esphome

