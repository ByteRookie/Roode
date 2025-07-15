#pragma once
#include <math.h>
#include <string>
#include <vector>
#include "Arduino.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "../vl53l1x/vl53l1x.h"
#include "esphome/core/preferences.h"
#include <time.h>
#include "orientation.h"
#include "zone.h"

using namespace esphome::vl53l1x;
using TofSensor = esphome::vl53l1x::VL53L1X;

namespace esphome {
namespace roode {
#define NOBODY 0
#define SOMEONE 1
#define VERSION "1.5.1"
static const char *const TAG = "Roode";
static const char *const SETUP = "Setup";
static const char *const CALIBRATION = "Sensor Calibration";


/*
Use the VL53L1X_SetTimingBudget function to set the TB in milliseconds. The TB
values available are [15, 20, 33, 50, 100, 200, 500]. This function must be
called after VL53L1X_SetDistanceMode. Note: 15 ms only works with Short distance
mode. 100 ms is the default value. The TB can be adjusted to improve the
standard deviation (SD) of the measurement. Increasing the TB, decreases the SD
but increases the power consumption.
*/

static int delay_between_measurements = 0;
static int time_budget_in_ms = 0;

/*
Parameters which define the time between two different measurements in various
modes (https://www.st.com/resource/en/datasheet/vl53l1x.pdf) The timing budget
and inter-measurement period should not be called when the sensor is ranging.
The user has to stop the ranging, change these parameters, and restart ranging
The minimum inter-measurement period must be longer than the timing budget + 4
ms.
// Lowest possible is 15ms with the ULD API
(https://www.st.com/resource/en/user_manual/um2510-a-guide-to-using-the-vl53l1x-ultra-lite-driver-stmicroelectronics.pdf)
Valid values: [15,20,33,50,100,200,500]
*/
static int time_budget_in_ms_short = 15;  // max range: 1.3m
static int time_budget_in_ms_medium = 33;
static int time_budget_in_ms_medium_long = 50;
static int time_budget_in_ms_long = 100;
static int time_budget_in_ms_max = 200;  // max range: 4m

class Roode : public PollingComponent {
 public:
  struct LuxSample { uint32_t ts; float lux; };
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  ~Roode();
  /** Roode uses data from sensors */
  float get_setup_priority() const override { return setup_priority::PROCESSOR; };

  TofSensor *get_tof_sensor() { return this->distanceSensor; }
  void set_tof_sensor(TofSensor *sensor) { this->distanceSensor = sensor; }
  void set_invert_direction(bool dir) { invert_direction_ = dir; }
  void set_orientation(Orientation val) { orientation_ = val; }
  void set_sampling_size(uint8_t size) {
    samples = size;
    entry->set_max_samples(size);
    exit->set_max_samples(size);
  }
  void set_distance_entry(sensor::Sensor *distance_entry_) { distance_entry = distance_entry_; }
  void set_distance_exit(sensor::Sensor *distance_exit_) { distance_exit = distance_exit_; }
  void set_people_counter(number::Number *counter) { this->people_counter = counter; }
  void set_max_threshold_entry_sensor(sensor::Sensor *max_threshold_entry_sensor_) {
    max_threshold_entry_sensor = max_threshold_entry_sensor_;
  }
  void set_max_threshold_exit_sensor(sensor::Sensor *max_threshold_exit_sensor_) {
    max_threshold_exit_sensor = max_threshold_exit_sensor_;
  }
  void set_min_threshold_entry_sensor(sensor::Sensor *min_threshold_entry_sensor_) {
    min_threshold_entry_sensor = min_threshold_entry_sensor_;
  }
  void set_min_threshold_exit_sensor(sensor::Sensor *min_threshold_exit_sensor_) {
    min_threshold_exit_sensor = min_threshold_exit_sensor_;
  }
  void set_entry_roi_height_sensor(sensor::Sensor *roi_height_sensor_) { entry_roi_height_sensor = roi_height_sensor_; }
  void set_entry_roi_width_sensor(sensor::Sensor *roi_width_sensor_) { entry_roi_width_sensor = roi_width_sensor_; }
  void set_exit_roi_height_sensor(sensor::Sensor *roi_height_sensor_) { exit_roi_height_sensor = roi_height_sensor_; }
  void set_exit_roi_width_sensor(sensor::Sensor *roi_width_sensor_) { exit_roi_width_sensor = roi_width_sensor_; }
  void set_sensor_status_sensor(sensor::Sensor *status_sensor_) { status_sensor = status_sensor_; }
  void set_loop_time_sensor(sensor::Sensor *sens) { loop_time_sensor = sens; }
  void set_cpu_usage_sensor(sensor::Sensor *sens) { cpu_usage_sensor = sens; }
  void set_ram_free_sensor(sensor::Sensor *sens) { ram_free_sensor = sens; }
  void set_flash_free_sensor(sensor::Sensor *sens) { flash_free_sensor = sens; }
  void set_presence_sensor_binary_sensor(binary_sensor::BinarySensor *presence_sensor_) {
    presence_sensor = presence_sensor_;
  }
  void set_version_text_sensor(text_sensor::TextSensor *version_sensor_) { version_sensor = version_sensor_; }
  void set_entry_exit_event_text_sensor(text_sensor::TextSensor *entry_exit_event_sensor_) {
    entry_exit_event_sensor = entry_exit_event_sensor_;
  }
  void set_interrupt_status_sensor(sensor::Sensor *sens) { /* deprecated */ }
  void set_enabled_features_text_sensor(text_sensor::TextSensor *sensor_) { enabled_features_sensor = sensor_; }
  void set_manual_adjustment_sensor(sensor::Sensor *sens) { manual_adjustment_sensor = sens; }
  void set_log_fallback_events(bool val) { log_fallback_events_ = val; }
  void set_force_single_core(bool val) { force_single_core_ = val; }
  void set_calibration_persistence(bool val) { calibration_persistence_ = val; }
  void set_filter_mode(FilterMode mode) {
    filter_mode_ = mode;
    default_filter_mode_ = mode;
    entry->set_filter_mode(mode);
    exit->set_filter_mode(mode);
  }
  void set_filter_window(uint8_t window) {
    filter_window_ = window;
    default_filter_window_ = window;
    entry->set_filter_window(window);
    exit->set_filter_window(window);
  }
  void run_zone_calibration(uint8_t zone_id);
  void recalibration();
  void apply_cpu_optimizations(float cpu);
  void reset_cpu_optimizations(float cpu);
  void update_metrics();
  void maybe_auto_recalibrate();
  void record_motion_event();
  void apply_adaptive_filtering(float lux, float pct95);
  Zone *entry = new Zone(0);
  Zone *exit = new Zone(1);
  static void log_event(const std::string &msg);
  static Roode *instance_;
  void record_int_fallback();
  void set_auto_recalibrate_interval(uint32_t ms) { auto_recalibrate_interval_ms_ = ms; }
  void set_recalibrate_cooldown(uint32_t ms) { recalibrate_cooldown_ms_ = ms; }
  void set_recalibrate_on_temp_change(bool val) { recalibrate_on_temp_change_ = val; }
  void set_max_temp_delta_for_recalib(float val) { max_temp_delta_for_recalib_ = val; }
  void set_use_light_sensor(bool val) { use_light_sensor_ = val; }
  void set_lux_learning_window(uint32_t ms) { lux_learning_window_ms_ = ms; }
  void set_lux_sample_interval(uint32_t ms) { lux_sample_interval_ms_ = ms; }
  void set_use_sunrise_prediction(bool val) { use_sunrise_prediction_ = val; }
  void set_latitude(float val) { latitude_ = val; }
  void set_longitude(float val) { longitude_ = val; }
  void set_alpha(float val) { alpha_ = val; }
  void set_base_multiplier(float val) { base_multiplier_ = val; }
  void set_max_multiplier(float val) { max_multiplier_ = val; }
  void set_time_multiplier(float val) { time_multiplier_ = val; }
  void set_combined_multiplier(float val) { combined_multiplier_ = val; }
  void set_suppression_window(uint32_t ms) { suppression_window_ms_ = ms; }
  void set_sunrise_sunset_window(uint32_t ms) { sunrise_sunset_window_ms_ = ms; }
  void set_temperature_sensor(sensor::Sensor *sens) { temperature_sensor_ = sens; }
  void set_lux_sensor(sensor::Sensor *sens) { lux_sensor_ = sens; }

 protected:
  TofSensor *distanceSensor;
  Zone *current_zone = entry;
  sensor::Sensor *distance_entry{nullptr};
  sensor::Sensor *distance_exit{nullptr};
  number::Number *people_counter{nullptr};
  sensor::Sensor *max_threshold_entry_sensor{nullptr};
  sensor::Sensor *max_threshold_exit_sensor{nullptr};
  sensor::Sensor *min_threshold_entry_sensor{nullptr};
  sensor::Sensor *min_threshold_exit_sensor{nullptr};
  sensor::Sensor *exit_roi_height_sensor{nullptr};
  sensor::Sensor *exit_roi_width_sensor{nullptr};
  sensor::Sensor *entry_roi_height_sensor{nullptr};
  sensor::Sensor *entry_roi_width_sensor{nullptr};
  sensor::Sensor *status_sensor{nullptr};
  sensor::Sensor *loop_time_sensor{nullptr};
  sensor::Sensor *cpu_usage_sensor{nullptr};
  sensor::Sensor *ram_free_sensor{nullptr};
  sensor::Sensor *flash_free_sensor{nullptr};
  binary_sensor::BinarySensor *presence_sensor{nullptr};
  text_sensor::TextSensor *version_sensor{nullptr};
  text_sensor::TextSensor *entry_exit_event_sensor{nullptr};
  text_sensor::TextSensor *enabled_features_sensor{nullptr};
  sensor::Sensor *manual_adjustment_sensor{nullptr};

  struct CalibrationPrefs {
    uint16_t baseline_mm;
    uint16_t threshold_min_mm;
    uint16_t threshold_max_mm;
    uint32_t last_calibrated_ts;
  };
  CalibrationPrefs calibration_data_[2];
  ESPPreferenceObject calibration_prefs_[2];
  bool calibration_persistence_{false};
  bool fail_safe_triggered_{false};

  FilterMode filter_mode_{FILTER_MIN};
  uint8_t filter_window_{5};
  FilterMode default_filter_mode_{FILTER_MIN};
  uint8_t default_filter_window_{5};

  bool cpu_optimizations_active_{false};
  uint16_t polling_interval_ms_{10};

  static bool log_fallback_events_;
  int manual_adjustment_count_{0};
  float expected_counter_{0};
  bool force_single_core_{false};
  TaskHandle_t sensor_task_handle_{nullptr};
  uint8_t multicore_retry_count_{0};
  bool multicore_failed_{false};
  uint32_t last_multicore_retry_ts_{0};

  // Interrupt robustness tracking
  int int_fallback_count_{0};
  uint32_t int_fallback_window_start_{0};

  uint32_t auto_recalibrate_interval_ms_{21600000};
  uint32_t recalibrate_cooldown_ms_{1800000};
  uint32_t last_recalibration_ts_{0};
  uint32_t last_auto_recalib_ts_{0};
  bool recalibrate_on_temp_change_{true};
  float max_temp_delta_for_recalib_{8.0};
  sensor::Sensor *temperature_sensor_{nullptr};
  float baseline_temp_{0};

  bool temp_sensor_ready_{false};
  bool lux_sensor_ready_{false};
  uint32_t boot_ts_{0};
  uint32_t last_temp_fail_ts_{0};
  uint32_t last_lux_fail_ts_{0};
  uint8_t lux_fail_count_{0};
  bool lux_sensor_failed_{false};
  static const uint32_t temp_startup_delay_ms_{10000};
  static const uint32_t lux_startup_delay_ms_{10000};
  static const uint32_t temp_retry_interval_ms_{30000};
  static const uint32_t lux_retry_interval_ms_{30000};

  bool use_light_sensor_{false};
  sensor::Sensor *lux_sensor_{nullptr};
  std::vector<LuxSample> lux_samples_;
  bool lux_bootstrap_logged_{false};
  bool lux_learning_complete_logged_{false};
  uint32_t lux_learning_window_ms_{86400000};
  uint32_t lux_sample_interval_ms_{60000};
  uint32_t last_lux_sample_ts_{0};
  ESPPreferenceObject lux_pref_;
  uint32_t sunrise_sunset_window_ms_{1800000};
  time_t next_sunrise_ts_{0};
  time_t next_sunset_ts_{0};
  bool use_sunrise_prediction_{true};
  float latitude_{0};
  float longitude_{0};
  float alpha_{0.5f};
  float base_multiplier_{1.0f};
  float max_multiplier_{4.0f};
  float time_multiplier_{1.5f};
  float combined_multiplier_{3.0f};
  uint32_t suppression_window_ms_{1800000};
  uint32_t sunlight_suppressed_until_{0};

  std::vector<uint32_t> motion_events_;

  std::vector<uint32_t> manual_adjust_timestamps_;

  enum FSMState { STATE_IDLE, STATE_ENTRY_ACTIVE, STATE_BOTH_ACTIVE };
  FSMState state_{STATE_IDLE};
  uint32_t state_started_ts{0};

  unsigned long last_valid_crossing_ts_{0};
  unsigned long zone_triggered_start_[2]{0, 0};

  VL53L1_Error last_sensor_status = VL53L1_ERROR_NONE;
  VL53L1_Error sensor_status = VL53L1_ERROR_NONE;
  void path_tracking(Zone *zone);
  bool handle_sensor_status();
  void calibrateDistance();
  void calibrate_zones();
  const RangingMode *determine_ranging_mode(uint16_t average_entry_zone_distance, uint16_t average_exit_zone_distance);
  void publish_sensor_configuration(Zone *entry, Zone *exit, bool isMax);
  void updateCounter(int delta);
  Orientation orientation_{Parallel};
  uint8_t samples{2};
  bool invert_direction_{false};
  int number_attempts = 20;  // TO DO: make this configurable
  int short_distance_threshold = 1300;
  int medium_distance_threshold = 2000;
  int medium_long_distance_threshold = 2700;
  int long_distance_threshold = 3400;
  uint32_t loop_window_start_{0};
  uint64_t loop_time_sum_{0};
  uint32_t loop_count_{0};
  static void sensor_task(void *param);
  void load_lux_samples();
  void save_lux_samples();
  void update_sun_times();
  bool use_sensor_task_{false};
};

}  // namespace roode
}  // namespace esphome

