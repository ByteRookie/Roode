#pragma once
#include <math.h>
#include <array>

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
  void roi_calibration(uint16_t entry_threshold, uint16_t exit_threshold);
  void set_threshold_percentages(uint8_t min_percent, uint8_t max_percent);
  const uint8_t id;
  uint16_t getDistance() const;
  uint16_t getMinDistance() const;
  ROI *roi = new ROI();
  ROI *roi_override = new ROI();
  Threshold *threshold = new Threshold();
  void set_filter_mode(FilterMode mode) { filter_mode_ = mode; }
  void set_filter_window(uint8_t window) {
    uint8_t new_max = std::min<uint8_t>(window, MAX_BUFFER_SIZE);
    // Do NOT wipe the sample buffer.  On dual-core ESP32 the sensor task reads
    // this buffer continuously; a full reset races with readDistance() and can
    // produce a frame of garbage distances that advance the FSM.
    // Instead just shrink the live count / write index so the buffer self-heals
    // within new_max readings — existing samples up to the new limit stay valid.
    max_samples = new_max;
    if (sample_count_ > max_samples)
      sample_count_ = max_samples;
    if (sample_idx_ >= max_samples)
      sample_idx_ = 0;
  }
  void set_max_samples(uint8_t max) { set_filter_window(max); };
  // Clear the sample ring-buffer so stale pre-calibration distances don't bleed
  // into the first detections after a recalibration.  Call this after any
  // calibrateThreshold() sequence completes.
  void reset_samples() {
    samples.fill(0);
    sample_idx_ = 0;
    sample_count_ = 0;
    min_distance = 0;
    last_distance = 0;
    consecutive_oor_ = 0;
  }

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
  // Counts consecutive out-of-range / no-target readings.  After
  // kOorClearThreshold consecutive misses, min_distance is zeroed so the
  // zone registers as inactive rather than holding a stale person-present
  // distance indefinitely (which would keep raw_active=true and make the
  // FSM cycle on timeout instead of settling back to IDLE).
  uint8_t consecutive_oor_{0};
  static const uint8_t kOorClearThreshold = 5;
};
}  // namespace roode
}  // namespace esphome
