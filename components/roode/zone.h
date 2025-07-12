#pragma once
#include <math.h>

#include "esphome/core/application.h"
#include <array>
#include <algorithm>
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

enum class FilterMode { MINIMUM, MEDIAN };

constexpr uint8_t MAX_SAMPLE_SIZE = 16;
constexpr uint8_t CAL_SAMPLE_SIZE = 50;

class Zone {
 public:
  explicit Zone(uint8_t id) : id{id} {};
  ~Zone() {
    delete roi;
    delete roi_override;
    delete threshold;
  }
  void dump_config() const;
  VL53L1_Error readDistance(TofSensor *distanceSensor);
  void reset_roi(uint8_t default_center);
  void calibrateThreshold(TofSensor *distanceSensor, int number_attempts);
  void roi_calibration(uint16_t entry_threshold, uint16_t exit_threshold, Orientation orientation);
  const uint8_t id;
  uint16_t getDistance() const;
  uint16_t getMinDistance() const;
  ROI *roi = new ROI();
  ROI *roi_override = new ROI();
  Threshold *threshold = new Threshold();
  void set_max_samples(uint8_t max) {
    max_samples = max > MAX_SAMPLE_SIZE ? MAX_SAMPLE_SIZE : max;
    sample_index = 0;
    sample_count = 0;
    std::fill(samples.begin(), samples.end(), 0);
  };
  void set_filter_mode(FilterMode mode) { filter_mode = mode; };
  void init_pref(uint32_t base_key);
  bool load_calibration(TofSensor *distanceSensor);
  void save_calibration();
  void run_zone_calibration(TofSensor *distanceSensor, Orientation orientation);
  uint32_t last_triggered_ts{0};
  uint32_t last_calibrated_ts{0};

 protected:
  int getOptimizedValues(int *values, int sum, int size);
  VL53L1_Error last_sensor_status = VL53L1_ERROR_NONE;
  VL53L1_Error sensor_status = VL53L1_ERROR_NONE;
  uint16_t last_distance;
  uint16_t min_distance;
  std::array<uint16_t, MAX_SAMPLE_SIZE> samples{};
  uint8_t max_samples{1};
  uint8_t sample_index{0};
  uint8_t sample_count{0};
  FilterMode filter_mode{FilterMode::MINIMUM};
  ESPPreferenceObject pref_;
  struct CalibrationData {
    uint16_t baseline_mm;
    uint16_t threshold_min_mm;
    uint16_t threshold_max_mm;
    uint32_t last_calibrated_ts;
  };
};
}  // namespace roode
}  // namespace esphome
