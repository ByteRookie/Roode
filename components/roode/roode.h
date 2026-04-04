#pragma once
#include <math.h>
#include <string>
#include <map>
#include <vector>
#include "Arduino.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"
#ifdef USE_SUN
#include "esphome/components/sun/sun.h"
#endif
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#ifdef USE_WEBSERVER
#include "esphome/components/web_server_base/web_server_base.h"
#ifdef CONFIG_IDF_TARGET_ESP32
namespace roode_web = esphome::web_server_idf;
using RoodeHttpMethod = ::http_method;
#else
namespace roode_web {
  using AsyncWebServerRequest = ::AsyncWebServerRequest;
  using AsyncWebHandler = ::AsyncWebHandler;
}
using RoodeHttpMethod = WebRequestMethod;
#endif
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
  // VL53L1X Hardware
  uint8_t sensor_id;
  uint16_t i2c_address;
  uint32_t timeout;
  uint8_t ranging_mode;
  uint16_t offset;
  uint32_t crosstalk;
  
  // Roode Core
  uint8_t sampling;
  Orientation orientation;
  bool calibration_persistence;
  FilterMode filter_mode;
  uint8_t filter_window;
  bool log_fallback_events;
  bool force_single_core;
  uint8_t invalid_limit;
  uint16_t restart_timeout;
  float cpu_activate;
  float cpu_deactivate;
  uint32_t active_sensors;
  bool debug_mode;

  // Zones
  bool invert_direction;
  uint8_t entry_roi_height;
  uint8_t entry_roi_width;
  uint8_t entry_roi_center;
  uint16_t entry_min_threshold;
  uint16_t entry_max_threshold;
  uint8_t exit_roi_height;
  uint8_t exit_roi_width;
  uint8_t exit_roi_center;
  uint16_t exit_min_threshold;
  uint16_t exit_max_threshold;

  // Lux/Sun
  bool use_lux;
  bool use_sun;
} __attribute__((packed));

enum ScanPhase { PHASE_IDLE, PHASE_EMPTY, PHASE_PERSON };
enum ScanState { SCAN_IDLE, SCANNING, SCAN_CANCELLED };

struct HistoricalScan {
  uint32_t ts;
  float lux;
  std::vector<uint16_t> distances;
};

class Roode;

class PortalSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(Roode *parent) { parent_ = parent; }
  void setup() override;
  void write_state(bool state) override;
  void dump_config() override;
 protected:
  Roode *parent_;
};

struct CalibrationSession {
  uint32_t ts;
  HistoricalScan background;
  HistoricalScan person;
  bool complete{false};
};

class Roode : public PollingComponent {
 public:
  Roode() : PollingComponent(100) { instance_ = this; }
  ~Roode();

  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; };

  void set_tof_sensor(TofSensor *sensor) { distanceSensor = sensor; }
  void set_invert_direction(bool dir) { invert_direction_ = dir; }
  void set_orientation(Orientation val) { orientation_ = val; }
  void set_polling_interval(uint32_t ms) { polling_interval_ms_ = ms; }
  void set_sampling_size(uint8_t size) {
    samples = size;
    entry->set_max_samples(size);
    exit->set_max_samples(size);
  }

  void set_distance_entry(sensor::Sensor *s) { distance_entry = s; }
  void set_distance_exit(sensor::Sensor *s) { distance_exit = s; }
  void set_people_counter(number::Number *counter) { this->people_counter = counter; }
  void set_max_threshold_entry_sensor(sensor::Sensor *s) { max_threshold_entry_sensor = s; }
  void set_max_threshold_exit_sensor(sensor::Sensor *s) { max_threshold_exit_sensor = s; }
  void set_min_threshold_entry_sensor(sensor::Sensor *s) { min_threshold_entry_sensor = s; }
  void set_min_threshold_exit_sensor(sensor::Sensor *s) { min_threshold_exit_sensor = s; }
  void set_entry_roi_height_sensor(sensor::Sensor *s) { entry_roi_height_sensor = s; }
  void set_entry_roi_width_sensor(sensor::Sensor *s) { entry_roi_width_sensor = s; }
  void set_exit_roi_height_sensor(sensor::Sensor *s) { exit_roi_height_sensor = s; }
  void set_exit_roi_width_sensor(sensor::Sensor *s) { exit_roi_width_sensor = s; }
  void set_sensor_status_sensor(sensor::Sensor *s) { status_sensor = s; }
  void set_loop_time_sensor(sensor::Sensor *s) { loop_time_sensor = s; }
  void set_cpu_usage_sensor(sensor::Sensor *s) { cpu_usage_sensor = s; }
  void set_ram_free_sensor(sensor::Sensor *s) { ram_free_sensor = s; }
  void set_flash_free_sensor(sensor::Sensor *s) { flash_free_sensor = s; }
  void set_presence_binary_sensor(binary_sensor::BinarySensor *s) { presence_sensor = s; }
  void set_entry_presence_binary_sensor(binary_sensor::BinarySensor *s) { entry_presence_sensor = s; }
  void set_exit_presence_binary_sensor(binary_sensor::BinarySensor *s) { exit_presence_sensor = s; }
  void set_sensor_xshut_state_binary_sensor(binary_sensor::BinarySensor *s) { xshut_state_sensor = s; }
  void set_version_text_sensor(text_sensor::TextSensor *s) { version_sensor = s; }
  void set_entry_exit_event_text_sensor(text_sensor::TextSensor *s) { entry_exit_event_sensor = s; }
  void set_sensor_status_text_sensor(text_sensor::TextSensor *s) { status_text_sensor = s; }
  void set_manual_adjustment_sensor(sensor::Sensor *s) { manual_adjustment_sensor = s; }
  void set_interrupt_status_sensor(sensor::Sensor *s) { interrupt_status_sensor = s; }
  void set_enabled_features_text_sensor(text_sensor::TextSensor *s) { enabled_features_sensor = s; }

  void set_log_fallback_events(bool val) { log_fallback_events_ = val; }
  void set_calibration_persistence(bool val) { calibration_persistence_ = val; }
  void set_invalid_distance_limit(uint8_t limit) { invalid_distance_limit_ = limit; }
  void set_restart_timeout(uint32_t ms) { restart_timeout_ms_ = ms; }
  void set_cpu_optimization_thresholds(float activate, float deactivate) {
    cpu_opt_activate_threshold_ = activate;
    cpu_opt_deactivate_threshold_ = deactivate;
  }
  
  void set_filter_mode(FilterMode mode) {
    filter_mode_ = mode;
    entry->set_filter_mode(mode);
    exit->set_filter_mode(mode);
  }
  void set_filter_window(uint8_t window) {
    filter_window_ = window;
    entry->set_filter_window(window);
    exit->set_filter_window(window);
  }
  void set_portal_password(const std::string &password) { portal_password_ = password; }
  void set_portal_enabled(bool enabled) { portal_enabled_ = enabled; }
  void set_counting_enabled(bool enabled) { counting_enabled_ = enabled; }
  bool is_counting_enabled() const { return counting_enabled_; }
  void set_debug_mode(bool debug) { debug_mode_ = debug; }
  void set_lux_sensor(sensor::Sensor *lux) { lux_sensor_ = lux; }
#ifdef USE_SUN
  void set_sun(sun::Sun *s) { sun_ = s; }
#endif
  void set_portal_switch(switch_::Switch *sw) { portal_switch = sw; }
  void set_force_single_core(bool val) { force_single_core_ = val; }
  void set_auto_calibration_interval_sec(uint32_t sec) { auto_calibration_interval_sec_ = sec; }
  void set_manual_presence(bool val) { manual_presence_ = val; }
  void set_active_sensors(uint32_t mask) { active_sensors_ = mask; }
  bool is_portal_enabled() const { return portal_enabled_; }

  void run_zone_calibration(uint8_t zone_id);
  void recalibration();
  void update_metrics();
  void updateCounter(int delta);
  void publish_static_states_();
  void publish_threshold_and_roi_states_();
  static void log_event(const std::string &msg);

#ifdef CONFIG_IDF_TARGET_ESP32
  static SemaphoreHandle_t i2c_mutex_;
  static bool i2c_lock(TickType_t timeout = portMAX_DELAY) {
    return i2c_mutex_ == nullptr || xSemaphoreTakeRecursive(i2c_mutex_, timeout) == pdTRUE;
  }
  static void i2c_unlock() {
    if (i2c_mutex_ != nullptr) xSemaphoreGiveRecursive(i2c_mutex_);
  }
#endif

  Zone *entry = new Zone(0);
  Zone *exit = new Zone(1);

 protected:
  TofSensor *distanceSensor{nullptr};
  switch_::Switch *portal_switch{nullptr};
  bool portal_enabled_{true};
  bool counting_enabled_{true};
  bool debug_mode_{false};
  uint32_t active_sensors_{0xFFFFFFFF};
  Orientation orientation_{Parallel};
  uint8_t samples{2};
  bool invert_direction_{false};
  uint16_t polling_interval_ms_{10};
  
  sensor::Sensor *distance_entry{nullptr};
  sensor::Sensor *distance_exit{nullptr};
  number::Number *people_counter{nullptr};
  sensor::Sensor *max_threshold_entry_sensor{nullptr};
  sensor::Sensor *max_threshold_exit_sensor{nullptr};
  sensor::Sensor *min_threshold_entry_sensor{nullptr};
  sensor::Sensor *min_threshold_exit_sensor{nullptr};
  sensor::Sensor *entry_roi_height_sensor{nullptr};
  sensor::Sensor *entry_roi_width_sensor{nullptr};
  sensor::Sensor *exit_roi_height_sensor{nullptr};
  sensor::Sensor *exit_roi_width_sensor{nullptr};
  sensor::Sensor *status_sensor{nullptr};
  sensor::Sensor *loop_time_sensor{nullptr};
  sensor::Sensor *cpu_usage_sensor{nullptr};
  sensor::Sensor *ram_free_sensor{nullptr};
  sensor::Sensor *flash_free_sensor{nullptr};
  binary_sensor::BinarySensor *presence_sensor{nullptr};
  binary_sensor::BinarySensor *entry_presence_sensor{nullptr};
  binary_sensor::BinarySensor *exit_presence_sensor{nullptr};
  binary_sensor::BinarySensor *xshut_state_sensor{nullptr};
  text_sensor::TextSensor *version_sensor{nullptr};
  text_sensor::TextSensor *entry_exit_event_sensor{nullptr};
  text_sensor::TextSensor *status_text_sensor{nullptr};
  text_sensor::TextSensor *enabled_features_sensor{nullptr};
  sensor::Sensor *manual_adjustment_sensor{nullptr};
  sensor::Sensor *interrupt_status_sensor{nullptr};

  struct CalibrationPrefs {
    uint16_t baseline_mm;
    uint16_t threshold_min_mm;
    uint16_t threshold_max_mm;
    uint32_t last_calibrated_ts;
  } __attribute__((packed));

  CalibrationPrefs calibration_data_[2];
  ESPPreferenceObject calibration_prefs_[2];
  bool calibration_persistence_{false};
  bool manual_presence_{false};
  uint32_t last_calibration_ts_{0};
  uint32_t last_calibration_millis_{0};
  uint32_t auto_calibration_interval_sec_{0};
  uint8_t invalid_distance_limit_{10};
  uint32_t restart_timeout_ms_{30000};
  float cpu_opt_activate_threshold_{90.0f};
  float cpu_opt_deactivate_threshold_{50.0f};
  float lux_avg_{0.0f};
  std::vector<float> rolling_lux_;
  bool use_lux_{false};
  bool use_sun_{false};

  FilterMode filter_mode_{FILTER_MIN};
  uint8_t filter_window_{5};

  ScanPhase scan_phase_{PHASE_IDLE};
  volatile ScanState scan_state_{SCAN_IDLE};
  volatile uint8_t scan_progress_{0};
  sensor::Sensor *lux_sensor_{nullptr};
#ifdef USE_SUN
  sun::Sun *sun_{nullptr};
#endif

  std::vector<CalibrationSession> sessions_;
  CalibrationSession active_session_;

  enum FSMState { STATE_IDLE, STATE_ACTIVE };
  FSMState state_{STATE_IDLE};
  uint32_t state_started_ts{0};
  int PathTrack[16];
  int PathTrackFillingSize = 0;
  int LeftPreviousStatus = NOBODY;
  int RightPreviousStatus = NOBODY;
  int AllZonesCurrentStatus = 0;

  bool force_single_core_{false};
  // Set when thresholds are zero at boot; calibration is deferred until the
  // sensor task/loop runs (after VL53L1X setup() has completed).
  bool needs_initial_calibration_{false};
#ifdef CONFIG_IDF_TARGET_ESP32
  TaskHandle_t sensor_task_handle_{nullptr};
#endif
  bool use_sensor_task_{false};

  ESPPreferenceObject settings_prefs_;
  RoodeSettings recommended_settings_{0};
  bool has_recommended_settings_{false};

  volatile bool presence_update_pending_{false};
  volatile bool presence_state_{false};
  volatile bool people_counter_update_pending_{false};
  float pending_people_counter_value_{0};
  volatile bool calibration_update_pending_{false};

  std::string portal_password_{};
  bool portal_registered_{false};
  static bool log_fallback_events_;
  static Roode *instance_;

  void register_portal_routes_();
#ifdef USE_WEBSERVER
  bool portal_request_authorized_(roode_web::AsyncWebServerRequest *request) const;
  bool require_portal_auth_(roode_web::AsyncWebServerRequest *request, const char *content_type) const;
  void send_portal_login_(roode_web::AsyncWebServerRequest *request) const;
#endif
  VL53L1_Error read_and_track_zone_(Zone *zone, bool update_timestamp);
  void path_tracking(Zone *zone);
  bool handle_sensor_status();
  void update_status_text(const std::string &status);
#ifdef CONFIG_IDF_TARGET_ESP32
  static void sensor_task(void *param);
#endif
  void restart_sensor();
  uint32_t loop_window_start_{0};
  uint64_t loop_time_sum_{0};
  uint32_t loop_count_{0};
};

}  // namespace roode
}  // namespace esphome
