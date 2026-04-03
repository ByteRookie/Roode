#pragma once
#include <math.h>
#include <string>
#include <map>
#include <vector>
#include "Arduino.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#ifdef USE_SUN
#include "esphome/components/sun/sun.h"
#endif
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#ifdef USE_WEBSERVER
#include "esphome/components/web_server_base/web_server_base.h"
#endif
#include "esphome/components/switch/switch.h"
#include "../vl53l1x/vl53l1x.h"
#include "esphome/core/preferences.h"
#include "orientation.h"
#include "zone.h"

using namespace esphome::vl53l1x;
using TofSensor = esphome::vl53l1x::VL53L1X;

namespace esphome {
namespace roode {
#define NOBODY 0
#define SOMEONE 1
#define VERSION "1.8.0"
static const char *const TAG = "Roode";
static const char *const SETUP = "Setup";
static const char *const CALIBRATION = "Sensor Calibration";

struct RoodeSettings {
  uint8_t roi_width;
  uint8_t roi_height;
  uint8_t roi_center;
  uint8_t entry_center;
  uint8_t exit_center;
  uint16_t min_threshold;
  uint16_t max_threshold;
  uint8_t sampling;
  uint16_t polling_interval;
  bool invert_direction;
  FilterMode filter_mode;
  uint8_t filter_window;
  uint32_t active_sensors;
  bool debug_mode;
} __attribute__((packed));

enum ScanPhase { PHASE_IDLE, PHASE_EMPTY, PHASE_PERSON };

class Roode;

class PortalSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(Roode *parent) { parent_ = parent; }
  void write_state(bool state) override;
  void dump_config() override;

 protected:
  Roode *parent_;
};

class Roode : public PollingComponent {
 public:
  Roode() { instance_ = this; }
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
  void set_presence_binary_sensor(binary_sensor::BinarySensor *presence_sensor_) {
    // Overall sensor indicating that any zone is occupied
    presence_sensor = presence_sensor_;
  }
  void set_entry_presence_binary_sensor(binary_sensor::BinarySensor *sensor) {
    // Optional sensor dedicated to the entry zone
    entry_presence_sensor = sensor;
  }
  void set_exit_presence_binary_sensor(binary_sensor::BinarySensor *sensor) {
    // Optional sensor dedicated to the exit zone
    exit_presence_sensor = sensor;
  }
  void set_version_text_sensor(text_sensor::TextSensor *version_sensor_) { version_sensor = version_sensor_; }
  void set_entry_exit_event_text_sensor(text_sensor::TextSensor *entry_exit_event_sensor_) {
    entry_exit_event_sensor = entry_exit_event_sensor_;
  }
  void set_xshut_state_binary_sensor(binary_sensor::BinarySensor *sens) { xshut_state_binary_sensor = sens; }
  void set_sensor_xshut_state_binary_sensor(binary_sensor::BinarySensor *sens) { xshut_state_binary_sensor = sens; }
  void set_interrupt_status_sensor(sensor::Sensor *sens) { interrupt_status_sensor = sens; }
  void set_enabled_features_text_sensor(text_sensor::TextSensor *sensor_) { enabled_features_sensor = sensor_; }
  void set_sensor_status_text_sensor(text_sensor::TextSensor *sensor_) { status_text_sensor = sensor_; }
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
  void set_invalid_distance_limit(uint8_t limit) { invalid_distance_limit_ = limit; }
  void set_restart_timeout(uint32_t ms) { restart_timeout_ms_ = ms; }
  void set_portal_password(const std::string &password) { portal_password_ = password; }
  void set_portal_enabled(bool enabled) { portal_enabled_ = enabled; }
  void set_debug_mode(bool debug) { debug_mode_ = debug; }
  void set_lux_sensor(sensor::Sensor *lux) { lux_sensor_ = lux; }
#ifdef USE_SUN
  void set_sun(sun::Sun *s) { sun_ = s; }
#endif
  void set_cpu_optimization_thresholds(float activate, float deactivate) {
    cpu_opt_activate_threshold_ = activate;
    cpu_opt_deactivate_threshold_ = deactivate;
  }
  void set_portal_switch(switch_::Switch *sw) { portal_switch = sw; }
  void run_zone_calibration(uint8_t zone_id);
  void recalibration();
  void set_entry_threshold_percentages(uint8_t min, uint8_t max) { entry->set_threshold_percentages(min, max); }
  void set_exit_threshold_percentages(uint8_t min, uint8_t max) { exit->set_threshold_percentages(min, max); }
  void apply_cpu_optimizations(float cpu);
  void reset_cpu_optimizations(float cpu);
  void update_metrics();
  Zone *entry = new Zone(0);
  Zone *exit = new Zone(1);
  static void log_event(const std::string &msg);
#ifdef CONFIG_IDF_TARGET_ESP32
  static SemaphoreHandle_t i2c_mutex_;
  static bool i2c_lock(TickType_t timeout = portMAX_DELAY) {
    return i2c_mutex_ == nullptr || xSemaphoreTakeRecursive(i2c_mutex_, timeout) == pdTRUE;
  }
  static void i2c_unlock() {
    if (i2c_mutex_ != nullptr)
      xSemaphoreGiveRecursive(i2c_mutex_);
  }
#endif

 protected:
  TofSensor *distanceSensor;
  switch_::Switch *portal_switch{nullptr};
  bool portal_enabled_{true};
  bool sensor_enabled_{true};
  uint32_t active_sensors_{0xFFFFFFFF};
  bool debug_mode_{false};
  ScanPhase scan_phase_{PHASE_IDLE};
  sensor::Sensor *lux_sensor_{nullptr};
#ifdef USE_SUN
  sun::Sun *sun_{nullptr};
#endif
  bool debug_mode_{false};
  uint8_t default_filter_window_{5};

  bool cpu_optimizations_active_{false};
  uint16_t polling_interval_ms_{10};
  float cpu_opt_activate_threshold_{90.0f};
  float cpu_opt_deactivate_threshold_{50.0f};

  static bool log_fallback_events_;
  static Roode *instance_;
  int manual_adjustment_count_{0};
  float expected_counter_{0};
  bool force_single_core_{false};
  TaskHandle_t sensor_task_handle_{nullptr};
  uint8_t multicore_retry_count_{0};

  enum FSMState { STATE_IDLE, STATE_ENTRY_ACTIVE, STATE_BOTH_ACTIVE };
  FSMState state_{STATE_IDLE};
  uint32_t state_started_ts{0};

  unsigned long last_valid_crossing_ts_{0};
  unsigned long zone_triggered_start_[2]{0, 0};

  std::string last_status_text_{};

  // Thread safety flags for Core 1 -> Core 0 communication
  volatile bool presence_update_pending_{false};
  volatile bool presence_state_{false};
  volatile bool entry_presence_update_pending_{false};
  volatile bool entry_presence_state_{false};
  volatile bool exit_presence_update_pending_{false};
  volatile bool exit_presence_state_{false};
  volatile bool sensor_status_update_pending_{false};
  VL53L1_Error pending_sensor_status_{VL53L1_ERROR_NONE};
  volatile bool status_text_update_pending_{false};
  volatile char pending_status_text_[32] = {0};
  volatile bool feature_list_update_pending_{false};
  volatile bool calibration_save_pending_[2]{false, false};
  volatile bool entry_exit_event_pending_{false};
  volatile char pending_entry_exit_event_[16] = {0};
  volatile bool people_counter_update_pending_{false};
  float pending_people_counter_value_{0};
  volatile bool manual_adjustment_update_pending_{false};
  int pending_manual_adjustment_count_{0};
  volatile bool metrics_update_pending_{false};
  float pending_loop_time_{0};
  float pending_cpu_usage_{0};
  float pending_ram_free_{0};
  float pending_flash_free_{0};
  volatile bool config_update_pending_{false};
  uint16_t pending_max_th_[2]{0, 0};
  uint16_t pending_min_th_[2]{0, 0};
  uint8_t pending_roi_h_[2]{0, 0};
  uint8_t pending_roi_w_[2]{0, 0};

  int PathTrack[4] = {0, 0, 0, 0};
  int PathTrackFillingSize = 1;
  int LeftPreviousStatus = NOBODY;
  int RightPreviousStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  struct ScanSession {
    std::string id;
    uint32_t ts;
    uint8_t trials;
    uint32_t duration_sec;
    size_t size_bytes;
  };
  std::vector<ScanSession> sessions_;
  std::map<std::string, std::vector<uint16_t>> scan_results_;
  enum ScanState { SCAN_IDLE, SCANNING, SCAN_CANCELLED };
  volatile ScanState scan_state_{SCAN_IDLE};
  std::string active_scan_id_{""};
  uint8_t scan_progress_{0};
  std::string scan_step_{""};
  uint32_t scan_start_ts_{0};
  RoodeSettings recommended_settings_{0};
  ESPPreferenceObject settings_prefs_;
  bool has_recommended_settings_{false};

  VL53L1_Error last_sensor_status = VL53L1_ERROR_NONE;
  VL53L1_Error sensor_status = VL53L1_ERROR_NONE;
  void register_portal_routes_();
#ifdef USE_WEBSERVER
  bool portal_request_authorized_(web_server_idf::AsyncWebServerRequest *request) const;
  bool require_portal_auth_(web_server_idf::AsyncWebServerRequest *request, const char *content_type) const;
  void send_portal_login_(web_server_idf::AsyncWebServerRequest *request) const;
#endif
  VL53L1_Error read_and_track_zone_(Zone *zone, bool update_timestamp);
  bool portal_registered_{false};
  std::string portal_password_{};
  void path_tracking(Zone *zone);
  bool handle_sensor_status();
  void update_status_text(const std::string &status);
  void calibrateDistance();
  void calibrate_zones();
  void publish_feature_list();
  bool pause_sensor_task_if_needed_();
  void resume_sensor_task_if_needed_(bool paused);
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
  uint32_t last_loop_update_ts_{0};
  uint32_t last_sensor_restart_ts_{0};
  uint32_t restart_timeout_ms_{30000};
  uint8_t invalid_distance_limit_{10};
  uint8_t invalid_read_count_{0};
  uint8_t restart_attempt_count_{0};
  uint8_t max_restart_attempts_{3};
  static void sensor_task(void *param);
  bool use_sensor_task_{false};
  void restart_sensor();
};

}  // namespace roode
}  // namespace esphome
