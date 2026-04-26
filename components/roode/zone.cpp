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
                threshold->min_percentage.value_or(threshold->idle > 0 ? (threshold->min * 100) / threshold->idle : 0),
                threshold->max,
                threshold->max_percentage.value_or(threshold->idle > 0 ? (threshold->max * 100) / threshold->idle : 0),
                threshold->idle);
}

VL53L1_Error Zone::readDistance(TofSensor *distanceSensor) {
  last_sensor_status = sensor_status;

  auto result = distanceSensor->read_distance(roi, sensor_status);
  if (!result.has_value()) {
    return sensor_status;
  }

  // Always record the raw reading so getDistance() reflects what the sensor
  // actually returned (including 65535 "no target") — used for diagnostics.
  last_distance = result.value();

  if (sensor_status != VL53L1_ERROR_NONE || result.value() == 0 || result.value() > 4000) {
    // Count consecutive out-of-range / no-target reads.  After
    // kOorClearThreshold misses, zero min_distance so the zone is treated as
    // inactive instead of holding a stale person-present distance.
    if (++consecutive_oor_ >= kOorClearThreshold) {
      min_distance = 0;
      sample_count_ = 0;
      sample_idx_ = 0;
    }
    return sensor_status;
  }
  consecutive_oor_ = 0;

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
      uint8_t idx = (sample_count_ * 10 + 99) / 100;  // ceiling division → idx≥1 for n≥1
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
  // Apply overrides when provided, otherwise fall back to sensible defaults
  roi->width = roi_override->width ? roi_override->width : 6;
  roi->height = roi_override->height ? roi_override->height : 16;
  roi->center = roi_override->center ? roi_override->center : default_center;
  ESP_LOGD(TAG, "%s ROI reset: { width: %d, height: %d, center: %d }", id == 0U ? "Entry" : "Exit", roi->width,
           roi->height, roi->center);
}

void Zone::calibrateThreshold(TofSensor *distanceSensor, int number_attempts) {
  ESP_LOGD(CALIBRATION, "Beginning. zoneId: %d", id);
  static constexpr size_t kMaxCalibrationSamples = 64;
  std::array<uint16_t, kMaxCalibrationSamples> zone_distances{};
  size_t zone_count = 0;

  for (int i = 0; i < number_attempts; i++) {
    // Feed the watchdog every 10 samples — calibrateThreshold can run for several
    // seconds (number_attempts × sensor timing budget) and must not starve the WDT.
    if ((i % 10) == 0)
      App.feed_wdt();
    this->readDistance(distanceSensor);
    if (sensor_status != VL53L1_ERROR_NONE) {
      ESP_LOGW(CALIBRATION, "Distance read failed during calibration. status: %d", sensor_status);
      break;
    }
    // Use the filtered distance (getMinDistance) so the calibrated idle baseline
    // matches the same metric used during detection, preventing threshold mismatches.
    uint16_t d = this->getMinDistance();
    if (d == 0 || d > 4000)
      continue;  // skip invalid readings during calibration
    if (zone_count < zone_distances.size()) {
      zone_distances[zone_count++] = d;
    }
  }

  if (zone_count == 0) {
    ESP_LOGW(CALIBRATION, "Calibration failed: no valid distances for zone %d — keeping previous idle=%dmm", id,
             threshold->idle);
    return;
  }

  // Outlier rejection: compute the median and discard readings outside ±25% of it.
  // Protects the baseline when someone walks through the field of view mid-calibration.
  // If the tight band rejects more than half the samples (noisy environment),
  // fall back to a wider ±40% band before giving up to the raw median.
  std::array<uint16_t, kMaxCalibrationSamples> sorted = zone_distances;
  std::sort(sorted.begin(), sorted.begin() + zone_count);
  uint16_t median = sorted[zone_count / 2];
  uint16_t lo = static_cast<uint16_t>(median * 75 / 100);
  uint16_t hi = static_cast<uint16_t>(median * 125 / 100);
  int64_t sum = 0;
  int count = 0;
  for (size_t i = 0; i < zone_count; i++) {
    uint16_t d = zone_distances[i];
    if (d >= lo && d <= hi) {
      sum += d;
      count++;
    }
  }
  if (count > 0 && static_cast<size_t>(count) < zone_count / 2) {
    // Tight band rejected >50% of samples — retry with ±40% to handle noisy environments.
    ESP_LOGW(CALIBRATION, "Zone %d: tight outlier band kept only %d/%zu samples, retrying with ±40%%",
             id, count, zone_count);
    lo = static_cast<uint16_t>(median * 60 / 100);
    hi = static_cast<uint16_t>(median * 140 / 100);
    sum = 0;
    count = 0;
    for (size_t i = 0; i < zone_count; i++) {
      uint16_t d = zone_distances[i];
      if (d >= lo && d <= hi) {
        sum += d;
        count++;
      }
    }
  }
  if (count == 0) {
    // All readings were outliers — fall back to the raw median
    sum = median;
    count = 1;
  }
  int avg = static_cast<int>(sum / count);

  threshold->idle = static_cast<uint16_t>(avg);
  uint8_t max_pct = threshold->max_percentage.value_or(80);
  uint8_t min_pct = threshold->min_percentage.value_or(15);
  // Clamp percentages to safe operating ranges before computing absolute thresholds.
  // max_pct >= 96% means threshold->max >= idle, so any sub-idle reading triggers
  // zone active (root cause of the 768 false-detections/21-min incident).
  // Values outside the safe window are reset to the defaults (15% / 80%).
  if (max_pct > 95) max_pct = 95;
  if (max_pct < 51) max_pct = 80;
  if (min_pct < 2)  min_pct = 15;
  if (min_pct > 49) min_pct = 15;
  threshold->max_percentage = max_pct;
  threshold->min_percentage = min_pct;
  threshold->max = (avg * max_pct) / 100;
  threshold->min = (avg * min_pct) / 100;
  // Hard floor: never allow min < 100 mm regardless of idle distance.
  // Prevents near-zero thresholds from matching virtually all sensor readings.
  if (threshold->min < 100) threshold->min = 100;
  // Ensure max is always meaningfully above min. For very close-range sensors
  // (idle < ~125mm) the 100mm floor can push min above max, killing detection.
  if (threshold->max < threshold->min + 50) {
    threshold->max = threshold->min + 50;
    ESP_LOGW(CALIBRATION, "Zone %d: max_threshold raised to %dmm to stay above min floor", id, threshold->max);
  }

  ESP_LOGI(CALIBRATION,
           "Calibrated zone %d: idle=%dmm min=%dmm(%d%%) max=%dmm(%d%%) samples=%d/%d",
           id, threshold->idle, threshold->min,
           threshold->min_percentage.value_or((threshold->min * 100) / threshold->idle),
           threshold->max,
           threshold->max_percentage.value_or((threshold->max * 100) / threshold->idle),
           count, (int) zone_count);
}

void Zone::roi_calibration(uint16_t entry_threshold, uint16_t exit_threshold) {
  // the value of the average distance is used for computing the optimal size of the ROI and consequently also the
  // center of the two zones
  int function_of_the_distance = 16 * (1 - (0.15 * 2) / (0.34 * (std::min(entry_threshold, exit_threshold) / 1000)));
  int ROI_size = std::min(8, std::max(4, function_of_the_distance));
  // Use the calculated ROI size unless an override has been specified
  this->roi->width = this->roi_override->width ? this->roi_override->width : ROI_size;
  this->roi->height = this->roi_override->height ? this->roi_override->height : ROI_size * 2;
  if (this->roi_override->center) {
    this->roi->center = this->roi_override->center;
  } else {
    // now we set the position of the center of the two zones
    // Side/Perpendicular orientation was removed — always use Parallel (Above) centers.
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
  }
  ESP_LOGI(CALIBRATION, "Calibrated ROI for zone. zoneId: %d, width: %d, height: %d, center: %d", id, roi->width,
           roi->height, roi->center);
}

uint16_t Zone::getDistance() const { return this->last_distance; }
uint16_t Zone::getMinDistance() const { return this->min_distance; }

void Zone::set_threshold_percentages(uint8_t min_percent, uint8_t max_percent) {
  if (min_percent >= max_percent) {
    ESP_LOGW(TAG, "Zone %d: ignoring threshold update — min_pct(%d) >= max_pct(%d)", id, min_percent, max_percent);
    return;
  }
  threshold->min_percentage = min_percent;
  threshold->max_percentage = max_percent;
  if (threshold->idle > 0) {
    threshold->min = (threshold->idle * min_percent) / 100;
    threshold->max = (threshold->idle * max_percent) / 100;
    if (threshold->min < 100) threshold->min = 100;
    if (threshold->max < threshold->min + 50) threshold->max = threshold->min + 50;
  }
}
}  // namespace roode
}  // namespace esphome
