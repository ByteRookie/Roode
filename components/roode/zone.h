#pragma once
#include <math.h>
#include <array>
#include <vector>

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/optional.h"
#include "../vl53l1x/vl53l1x.h"
#include "orientation.h"

using TofSensor = esphome::vl53l1x::VL53L1X;
using esphome::vl53l1x::ROI;

static const char *const TAG = "Zone";
static const char *const CALIBRATION = "Zone calibration";
namespace esphome {
namespace roode {
enum FilterMode { FILTER_MIN, FILTER_MEDIAN, FILTER_PERCENTILE10 };
struct Threshold {
  /** Automatically determined idling distance (average of several measurements) */
  uint16_t idle;
  uint16_t min;
  optional<uint8_t> min_percentage{};
  uint16_t max;
  optional<uint8_t> max_percentage{};
  void set_min(uint16_t min) { this->min = min; }
  void set_min_percentage(uint8_t min) { this->min_percentage = min; }
  void set_max(uint16_t max) { this->max = max; }
  void set_max_percentage(uint8_t max) { this->max_percentage = max; }
};

class Zone {
 public:
  explicit Zone(uint8_t id) : id{id} {};
  ~Zone();
  void dump_config() const;
  VL53L1_Error readDistance(TofSensor *distanceSensor);
  void reset_roi(uint8_t default_center);
  void calibrateThreshold(TofSensor *distanceSensor, int number_attempts);
  void roi_calibration(uint16_t entry_threshold, uint16_t exit_threshold, Orientation orientation);
  void set_threshold_percentages(uint8_t min_percent, uint8_t max_percent);
  const uint8_t id;
  uint16_t getDistance() const;
  uint16_t getMinDistance() const;
  ROI *roi = new ROI();
  ROI *roi_override = new ROI();
  Threshold *threshold = new Threshold();
  void set_filter_mode(FilterMode mode) { filter_mode_ = mode; }
  void set_filter_window(uint8_t window) {
    max_samples = std::min<uint8_t>(window, MAX_BUFFER_SIZE);
    sample_idx_ = 0;
    sample_count_ = 0;
    samples.fill(0);
  }
  void set_max_samples(uint8_t max) { set_filter_window(max); };

 protected:
  VL53L1_Error last_sensor_status = VL53L1_ERROR_NONE;
  VL53L1_Error sensor_status = VL53L1_ERROR_NONE;
  uint16_t last_distance{0};
  uint16_t min_distance{0};
  static const uint8_t MAX_BUFFER_SIZE = 10;
  std::array<uint16_t, MAX_BUFFER_SIZE> samples{};
  uint8_t sample_idx_{0};
  uint8_t sample_count_{0};
  uint8_t max_samples{2};
  FilterMode filter_mode_{FILTER_MIN};
};
}  // namespace roode
}  // namespace esphome
