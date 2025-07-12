#include "zone.h"

namespace esphome {
namespace roode {

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
  samples.insert(samples.begin(), result.value());
  if (samples.size() > max_samples) {
    samples.pop_back();
  }
  if (filter_mode == FilterMode::MEDIAN) {
    auto tmp = samples;
    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
    min_distance = tmp[tmp.size() / 2];
  } else {
    min_distance = *std::min_element(samples.begin(), samples.end());
  }

  return sensor_status;
}

void Zone::init_pref(uint32_t base_key) {
  this->pref_ = global_preferences->make_preference<CalibrationData>(base_key);
}

bool Zone::load_calibration(TofSensor *distanceSensor) {
  CalibrationData data{};
  if (!this->pref_.load(&data)) {
    return false;
  }
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    this->readDistance(distanceSensor);
    sum += this->getDistance();
  }
  uint16_t avg = sum / 5;
  if (abs((int) avg - data.baseline_mm) > data.baseline_mm * 0.1) {
    ESP_LOGW(TAG, "Stored calibration invalid for zone %d", id);
    return false;
  }
  threshold->idle = data.baseline_mm;
  threshold->min = data.threshold_min_mm;
  threshold->max = data.threshold_max_mm;
  last_calibrated_ts = data.last_calibrated_ts;
  ESP_LOGI(TAG, "Loaded calibration for zone %d", id);
  return true;
}

void Zone::save_calibration() {
  CalibrationData data{threshold->idle, threshold->min, threshold->max, (uint32_t) millis()};
  this->pref_.save(&data);
  last_calibrated_ts = data.last_calibrated_ts;
}

void Zone::run_zone_calibration(TofSensor *distanceSensor, Orientation orientation) {
  reset_roi(orientation == Parallel ? (id == 0U ? 167 : 231) : (id == 0U ? 195 : 60));
  std::vector<uint16_t> vals;
  for (int i = 0; i < 50; i++) {
    this->readDistance(distanceSensor);
    vals.push_back(this->getDistance());
  }
  uint32_t sum = 0;
  for (auto v : vals)
    sum += v;
  uint16_t average = sum / vals.size();
  threshold->idle = average;
  threshold->max = average * 0.8;
  threshold->min = average * 0.15;
  last_calibrated_ts = millis();
  ESP_LOGI(CALIBRATION, "Fail-safe calibration zone %d avg %d", id, average);
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
  std::vector<int> zone_distances(number_attempts);
  int sum = 0;
  for (int i = 0; i < number_attempts; i++) {
    this->readDistance(distanceSensor);
    zone_distances[i] = this->getDistance();
    sum += zone_distances[i];
  }
  threshold->idle = this->getOptimizedValues(zone_distances.data(), sum, number_attempts);

  if (threshold->max_percentage.has_value()) {
    threshold->max = (threshold->idle * threshold->max_percentage.value()) / 100;
  }
  if (threshold->min_percentage.has_value()) {
    threshold->min = (threshold->idle * threshold->min_percentage.value()) / 100;
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

int Zone::getOptimizedValues(int *values, int sum, int size) {
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
