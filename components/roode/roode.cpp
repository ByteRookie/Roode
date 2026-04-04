#include "roode.h"
#include "roode_portal.h"
#include "Arduino.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp_task_wdt.h"
#endif
#include "esphome/components/web_server_base/web_server_base.h"
#ifdef USE_WEBSERVER
#include <ArduinoJson.h>
#endif
#include <string>
#include <optional>
#include <vector>
#include <algorithm>
#include <ctime>

namespace esphome {
namespace roode {

#ifdef USE_WEBSERVER
namespace {
class LambdaRequestHandler : public roode_web::AsyncWebHandler {
 public:
  enum MatchType { EXACT, PREFIX };
  using Callback = std::function<void(roode_web::AsyncWebServerRequest *request)>;
  LambdaRequestHandler(RoodeHttpMethod method, std::string uri, Callback callback, MatchType match_type = EXACT)
      : method_(method), uri_(std::move(uri)), callback_(std::move(callback)), match_type_(match_type) {}
  bool canHandle(roode_web::AsyncWebServerRequest *request) const override {
    if (request->method() != method_) return false;
    std::string request_url;
#ifdef CONFIG_IDF_TARGET_ESP32
    char url_buf[roode_web::AsyncWebServerRequest::URL_BUF_SIZE];
    request_url = request->url_to(url_buf);
#else
    request_url = request->url().c_str();
#endif
    return match_type_ == PREFIX ? request_url.rfind(uri_, 0) == 0 : request_url == uri_;
  }
  void handleRequest(roode_web::AsyncWebServerRequest *request) override { callback_(request); }
 private:
  RoodeHttpMethod method_; std::string uri_; Callback callback_; MatchType match_type_;
};
}
#endif

static const char *const portal_login_html = R"LOGIN(
<!doctype html><html><head><meta charset="utf-8" /><title>Login</title><style>body{background:#0b0f14;color:#e6edf3;font-family:sans-serif;display:grid;place-items:center;min-height:100vh}form{background:#121821;padding:24px;border-radius:16px;border:1px solid #1f2835}input{width:100%;padding:12px;margin:12px 0;background:transparent;color:inherit;border:1px solid #1f2835;border-radius:8px}button{width:100%;padding:12px;background:#57a6ff;color:#fff;border:0;border-radius:8px;cursor:pointer}</style></head><body><form id="f"><h1>Roode</h1><input id="t" type="password" placeholder="Password" /><button type="submit">Login</button></form><script>document.getElementById('f').onsubmit=(e)=>{e.preventDefault();location.href='/portal?token='+encodeURIComponent(document.getElementById('t').value);};</script></body></html>
)LOGIN";

Roode *Roode::instance_ = nullptr;
bool Roode::log_fallback_events_ = false;
#ifdef CONFIG_IDF_TARGET_ESP32
SemaphoreHandle_t Roode::i2c_mutex_ = nullptr;
#endif

static uint32_t session_id_counter = 1;

static const RangingMode *ranging_modes[] = {
  nullptr, // 0 = Auto
  Ranging::Shortest,
  Ranging::Short,
  Ranging::Medium,
  Ranging::Long,
  Ranging::Longer,
  Ranging::Longest
};

static uint8_t get_ranging_mode_index(const RangingMode *mode) {
  if (!mode) return 0;
  for (uint8_t i = 1; i < 7; i++) {
    if (ranging_modes[i] == mode) return i;
  }
  return 0;
}

void Roode::log_event(const std::string &msg) { if (instance_ && instance_->debug_mode_) ESP_LOGI(TAG, "%s", msg.c_str()); }
void Roode::dump_config() { ESP_LOGCONFIG(TAG, "Roode v%s", VERSION); }
Roode::~Roode() { instance_ = nullptr; }

void Roode::setup() {
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!i2c_mutex_) i2c_mutex_ = xSemaphoreCreateRecursiveMutex();
#endif
  settings_prefs_ = global_preferences->make_preference<RoodeSettings>(0xB0);
  bool loaded_from_flash = false;
  RoodeSettings s; if (settings_prefs_.load(&s)) {
    loaded_from_flash = true;
    samples = s.sampling; orientation_ = s.orientation; invert_direction_ = s.invert_direction;
    calibration_persistence_ = s.calibration_persistence; filter_mode_ = s.filter_mode; filter_window_ = s.filter_window;
    log_fallback_events_ = s.log_fallback_events; force_single_core_ = s.force_single_core;
    invalid_distance_limit_ = s.invalid_limit; restart_timeout_ms_ = s.restart_timeout;
    cpu_opt_activate_threshold_ = s.cpu_activate; cpu_opt_deactivate_threshold_ = s.cpu_deactivate;
    active_sensors_ = s.active_sensors; debug_mode_ = s.debug_mode;

    // Only load ROI from flash if it has valid non-zero values
    if (s.entry_roi_height > 0 && s.entry_roi_width > 0) {
      entry->roi->height = s.entry_roi_height; entry->roi->width = s.entry_roi_width; entry->roi->center = s.entry_roi_center;
    }
    if (s.exit_roi_height > 0 && s.exit_roi_width > 0) {
      exit->roi->height = s.exit_roi_height; exit->roi->width = s.exit_roi_width; exit->roi->center = s.exit_roi_center;
    }
    entry->threshold->min = s.entry_min_threshold; entry->threshold->max = s.entry_max_threshold;
    exit->threshold->min = s.exit_min_threshold; exit->threshold->max = s.exit_max_threshold;

    use_lux_ = s.use_lux; use_sun_ = s.use_sun;

    if (distanceSensor) {
      if (s.sensor_id > 0) distanceSensor->set_sensor_id(s.sensor_id);
      if (s.timeout > 0) distanceSensor->set_timeout(s.timeout);
      if (s.ranging_mode < 7) distanceSensor->set_ranging_mode_override(ranging_modes[s.ranging_mode]);
    }
    entry->set_debug_mode(debug_mode_); exit->set_debug_mode(debug_mode_);
  }

  // Apply YAML roi_override → roi for any zone whose roi is still zero (not loaded from flash).
  // This is the fix for the critical bug: roi starts as {0,0,0} and reset_roi() was never called,
  // causing every sensor.SetROI(0,0) to fail and return no valid readings.
  entry->reset_roi(orientation_ == Parallel ? 159 : 194);
  exit->reset_roi(orientation_ == Parallel ? 239 : 59);

  // If thresholds are zero, schedule calibration to run after VL53L1X setup()
  // completes. Roode has setup_priority::PROCESSOR=300, VL53L1X has DATA=200,
  // so VL53L1X setup() runs AFTER ours — we cannot call recalibration() here.
  if (entry->threshold->max == 0 || exit->threshold->max == 0) {
    needs_initial_calibration_ = true;
    ESP_LOGI(TAG, "No calibration data — will calibrate after sensor initializes");
  }

  // Publish initial states so HA entities show as available (not "unavailable")
  // immediately after boot, before the first update() tick.
  publish_static_states_();
  publish_threshold_and_roi_states_();

#ifdef CONFIG_IDF_TARGET_ESP32
  if (!force_single_core_) {
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 8192, this, 1, &sensor_task_handle_, 1);
    if (sensor_task_handle_) use_sensor_task_ = true;
  }
#endif
  register_portal_routes_();
}

void Roode::register_portal_routes_() {
#ifdef USE_WEBSERVER
  auto *base = web_server_base::global_web_server_base;
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/portal", [this](roode_web::AsyncWebServerRequest *r) {
    if (require_portal_auth_(r, "text/html")) r->send(200, "text/html", portal_html);
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/settings/current", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    JsonDocument doc; 
    doc["sa"] = samples; doc["or"] = (int)orientation_; doc["inv"] = invert_direction_;
    doc["cp"] = calibration_persistence_; doc["fm"] = (int)filter_mode_; doc["fw"] = filter_window_;
    doc["ul"] = use_lux_; doc["us"] = use_sun_;
    doc["sid"] = distanceSensor ? distanceSensor->get_sensor_id() : 1;
    doc["addr"] = distanceSensor ? distanceSensor->get_address() : 0x29;
    doc["to"] = distanceSensor ? distanceSensor->get_timeout() : 2000;
    doc["rm"] = distanceSensor ? get_ranging_mode_index(distanceSensor->get_ranging_mode()) : 0;
    doc["debug"] = debug_mode_; doc["sc"] = force_single_core_;
    doc["il"] = invalid_distance_limit_; doc["rt"] = (int)restart_timeout_ms_;
    doc["m"] = active_sensors_; doc["ce"] = counting_enabled_;
    if (lux_sensor_) doc["lux_val"] = lux_sensor_->state;
    doc["ts"] = (time(nullptr) > 1000000);
    doc["erh"] = entry->roi->height; doc["erw"] = entry->roi->width; doc["erc"] = entry->roi->center;
    doc["emin"] = entry->threshold->min; doc["emax"] = entry->threshold->max;
    doc["xrh"] = exit->roi->height; doc["xrw"] = exit->roi->width; doc["xrc"] = exit->roi->center;
    doc["xmin"] = exit->threshold->min; doc["xmax"] = exit->threshold->max;
    
    std::string out; serializeJson(doc, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/settings/update", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    RoodeSettings s; if (!settings_prefs_.load(&s)) memset(&s, 0, sizeof(s));
    if (r->hasParam("sa")) s.sampling = samples = atoi(r->getParam("sa")->value().c_str());
    if (r->hasParam("or")) s.orientation = orientation_ = (Orientation)atoi(r->getParam("or")->value().c_str());
    if (r->hasParam("inv")) s.invert_direction = invert_direction_ = (r->getParam("inv")->value() == "true");
    if (r->hasParam("cp")) s.calibration_persistence = calibration_persistence_ = (r->getParam("cp")->value() == "true");
    if (r->hasParam("fm")) s.filter_mode = filter_mode_ = (FilterMode)atoi(r->getParam("fm")->value().c_str());
    if (r->hasParam("fw")) s.filter_window = filter_window_ = atoi(r->getParam("fw")->value().c_str());
    if (r->hasParam("ul")) s.use_lux = use_lux_ = (r->getParam("ul")->value() == "true");
    if (r->hasParam("us")) s.use_sun = use_sun_ = (r->getParam("us")->value() == "true");
    if (r->hasParam("sid")) s.sensor_id = atoi(r->getParam("sid")->value().c_str());
    if (r->hasParam("to")) s.timeout = atoi(r->getParam("to")->value().c_str());
    if (r->hasParam("rm")) s.ranging_mode = atoi(r->getParam("rm")->value().c_str());
    if (r->hasParam("debug")) s.debug_mode = debug_mode_ = (r->getParam("debug")->value() == "true");
    if (r->hasParam("sc")) s.force_single_core = force_single_core_ = (r->getParam("sc")->value() == "true");
    if (r->hasParam("il")) s.invalid_limit = invalid_distance_limit_ = atoi(r->getParam("il")->value().c_str());
    if (r->hasParam("rt")) s.restart_timeout = restart_timeout_ms_ = atoi(r->getParam("rt")->value().c_str());
    if (r->hasParam("m")) s.active_sensors = active_sensors_ = strtoul(r->getParam("m")->value().c_str(), NULL, 10);
    if (r->hasParam("ce")) counting_enabled_ = (r->getParam("ce")->value() == "true");

    // ROI params (apply live and persist)
    if (r->hasParam("erh")) { entry->roi->height = atoi(r->getParam("erh")->value().c_str()); }
    if (r->hasParam("erw")) { entry->roi->width  = atoi(r->getParam("erw")->value().c_str()); }
    if (r->hasParam("erc")) { entry->roi->center = atoi(r->getParam("erc")->value().c_str()); }
    if (r->hasParam("xrh")) { exit->roi->height  = atoi(r->getParam("xrh")->value().c_str()); }
    if (r->hasParam("xrw")) { exit->roi->width   = atoi(r->getParam("xrw")->value().c_str()); }
    if (r->hasParam("xrc")) { exit->roi->center  = atoi(r->getParam("xrc")->value().c_str()); }
    // Threshold params (apply live and persist)
    if (r->hasParam("emin")) { entry->threshold->min = atoi(r->getParam("emin")->value().c_str()); }
    if (r->hasParam("emax")) { entry->threshold->max = atoi(r->getParam("emax")->value().c_str()); }
    if (r->hasParam("xmin")) { exit->threshold->min  = atoi(r->getParam("xmin")->value().c_str()); }
    if (r->hasParam("xmax")) { exit->threshold->max  = atoi(r->getParam("xmax")->value().c_str()); }

    s.entry_roi_height = entry->roi->height; s.entry_roi_width = entry->roi->width; s.entry_roi_center = entry->roi->center;
    s.entry_min_threshold = entry->threshold->min; s.entry_max_threshold = entry->threshold->max;
    s.exit_roi_height = exit->roi->height; s.exit_roi_width = exit->roi->width; s.exit_roi_center = exit->roi->center;
    s.exit_min_threshold = exit->threshold->min; s.exit_max_threshold = exit->threshold->max;
    s.cpu_activate = cpu_opt_activate_threshold_; s.cpu_deactivate = cpu_opt_deactivate_threshold_;

    if (distanceSensor) {
      if (s.sensor_id > 0) distanceSensor->set_sensor_id(s.sensor_id);
      if (s.timeout > 0) distanceSensor->set_timeout(s.timeout);
      if (s.ranging_mode < 7) distanceSensor->set_ranging_mode_override(ranging_modes[s.ranging_mode]);
    }
    entry->set_debug_mode(debug_mode_); exit->set_debug_mode(debug_mode_);
    settings_prefs_.save(&s); global_preferences->sync();
    publish_threshold_and_roi_states_();
    r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/start", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    scan_state_ = SCANNING; scan_progress_ = 0;
    scan_phase_ = r->hasParam("phase") ? (ScanPhase)atoi(r->getParam("phase")->value().c_str()) : PHASE_EMPTY;
    r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/status", [this](roode_web::AsyncWebServerRequest *r) {
    JsonDocument d; d["s"] = scan_state_ == SCANNING ? "Scanning" : "Idle"; d["p"] = scan_progress_;
    std::string out; serializeJson(d, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/cancel", [this](roode_web::AsyncWebServerRequest *r) {
    scan_state_ = SCAN_IDLE; scan_progress_ = 0;
    r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/recalibrate", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    recalibration();
    RoodeSettings cal_s;
    if (!settings_prefs_.load(&cal_s)) memset(&cal_s, 0, sizeof(cal_s));
    cal_s.entry_min_threshold = entry->threshold->min; cal_s.entry_max_threshold = entry->threshold->max;
    cal_s.exit_min_threshold  = exit->threshold->min;  cal_s.exit_max_threshold  = exit->threshold->max;
    settings_prefs_.save(&cal_s); global_preferences->sync();
    publish_threshold_and_roi_states_();
    JsonDocument d; d["ok"] = true; d["emin"] = entry->threshold->min; d["emax"] = entry->threshold->max;
    d["xmin"] = exit->threshold->min; d["xmax"] = exit->threshold->max; d["idle_e"] = entry->threshold->idle; d["idle_x"] = exit->threshold->idle;
    std::string out; serializeJson(d, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/sessions", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    JsonDocument doc; JsonArray arr = doc.to<JsonArray>();
    for (size_t i = 0; i < sessions_.size(); i++) {
      JsonObject o = arr.add<JsonObject>(); o["id"] = std::to_string(sessions_[i].id);
      o["bg_lux"] = sessions_[i].background.lux; o["p_lux"] = sessions_[i].person.lux;
      o["complete"] = sessions_[i].complete;
      if (sessions_[i].ts > 1000000) {
        char buf[32]; struct tm *tm = localtime((time_t *)&sessions_[i].ts);
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm); o["date"] = std::string(buf);
      } else {
        o["date"] = "Session #" + std::to_string(sessions_[i].id);
      }
    }
    std::string out; serializeJson(doc, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/delete", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    if (r->hasParam("id")) {
      uint32_t sid = strtoul(r->getParam("id")->value().c_str(), NULL, 10);
      auto it = std::remove_if(sessions_.begin(), sessions_.end(), [sid](const CalibrationSession& s){ return s.id == sid; });
      sessions_.erase(it, sessions_.end());
    }
    r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/roi/preview", [this](roode_web::AsyncWebServerRequest *r) {
    if (!has_recommended_settings_) { r->send(200, "application/json", "null"); return; }
    JsonDocument d; d["roi"] = recommended_settings_.entry_roi_center; d["con"] = (int)recommended_settings_.entry_max_threshold;
    std::string out; serializeJson(d, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/roi/apply", [this](roode_web::AsyncWebServerRequest *r) {
    entry->roi->center = exit->roi->center = recommended_settings_.entry_roi_center;
    entry->threshold->max = exit->threshold->max = recommended_settings_.entry_max_threshold;
    recalibration(); r->send(200, "application/json", "{\"ok\":true}");
  }));
#endif
}

void Roode::update() {
  if (lux_sensor_ && !std::isnan(lux_sensor_->state)) {
    rolling_lux_.push_back(lux_sensor_->state);
    if (rolling_lux_.size() > 60) rolling_lux_.erase(rolling_lux_.begin());
    float sum = 0; for (float f : rolling_lux_) sum += f;
    lux_avg_ = sum / rolling_lux_.size();
  }
  if (active_sensors_ & 0x01 && distance_entry) distance_entry->publish_state(entry->getDistance());
  if (active_sensors_ & 0x02 && distance_exit) distance_exit->publish_state(exit->getDistance());
  if (active_sensors_ & 0x100 && status_sensor) status_sensor->publish_state((float)entry->sensor_status);
  if (active_sensors_ & 0x800 && interrupt_status_sensor)
    interrupt_status_sensor->publish_state(distanceSensor && distanceSensor->is_interrupt_enabled() ? 1.0f : 0.0f);
  // Threshold and ROI sensors update every poll cycle so HA stays in sync with live values
  if (active_sensors_ & 0x8000) {
    if (max_threshold_entry_sensor) max_threshold_entry_sensor->publish_state(entry->threshold->max);
    if (min_threshold_entry_sensor) min_threshold_entry_sensor->publish_state(entry->threshold->min);
  }
  if (active_sensors_ & 0x10000) {
    if (max_threshold_exit_sensor) max_threshold_exit_sensor->publish_state(exit->threshold->max);
    if (min_threshold_exit_sensor) min_threshold_exit_sensor->publish_state(exit->threshold->min);
  }
  if (active_sensors_ & 0x20000) {
    if (entry_roi_height_sensor) entry_roi_height_sensor->publish_state(entry->roi->height);
    if (entry_roi_width_sensor) entry_roi_width_sensor->publish_state(entry->roi->width);
    if (exit_roi_height_sensor) exit_roi_height_sensor->publish_state(exit->roi->height);
    if (exit_roi_width_sensor) exit_roi_width_sensor->publish_state(exit->roi->width);
  }
  update_metrics();
}

void Roode::update_metrics() {
  uint32_t now = millis(); if (now - loop_window_start_ < 10000) return;
  loop_window_start_ = now;
#ifdef CONFIG_IDF_TARGET_ESP32
  if (active_sensors_ & 0x10 && ram_free_sensor)
    ram_free_sensor->publish_state((float)ESP.getFreeHeap() / (float)ESP.getHeapSize() * 100.0f);
  if (active_sensors_ & 0x20 && flash_free_sensor) {
    uint32_t free_sketch = ESP.getFreeSketchSpace();
    uint32_t total_sketch = free_sketch + ESP.getSketchSize();
    if (total_sketch > 0)
      flash_free_sensor->publish_state((float)free_sketch / (float)total_sketch * 100.0f);
  }
#endif
}

void Roode::loop() {
  if (use_sensor_task_) {
#ifdef CONFIG_IDF_TARGET_ESP32
    if (presence_update_pending_ && active_sensors_ & 0x04 && presence_sensor) { presence_sensor->publish_state(presence_state_); presence_update_pending_ = false; }
    if (people_counter_update_pending_ && active_sensors_ & 0x08 && people_counter) { auto c = people_counter->make_call(); c.set_value(pending_people_counter_value_); c.perform(); people_counter_update_pending_ = false; }
    if (calibration_update_pending_) { calibration_update_pending_ = false; publish_threshold_and_roi_states_(); }
    // sensor_task handles its own blocking — do not delay the main loop here
#endif
  } else {
    // Single-core: run deferred calibration on first iteration (after VL53L1X is ready)
    if (needs_initial_calibration_ && distanceSensor && !distanceSensor->is_failed()) {
      needs_initial_calibration_ = false;
      ESP_LOGI(TAG, "Running initial threshold calibration");
      recalibration();
      RoodeSettings cal_s; if (!settings_prefs_.load(&cal_s)) memset(&cal_s, 0, sizeof(cal_s));
      cal_s.entry_roi_height = entry->roi->height; cal_s.entry_roi_width = entry->roi->width; cal_s.entry_roi_center = entry->roi->center;
      cal_s.entry_min_threshold = entry->threshold->min; cal_s.entry_max_threshold = entry->threshold->max;
      cal_s.exit_roi_height = exit->roi->height; cal_s.exit_roi_width = exit->roi->width; cal_s.exit_roi_center = exit->roi->center;
      cal_s.exit_min_threshold = exit->threshold->min; cal_s.exit_max_threshold = exit->threshold->max;
      settings_prefs_.save(&cal_s); global_preferences->sync();
      publish_threshold_and_roi_states_();
    }
    read_and_track_zone_(entry, true); read_and_track_zone_(exit, false); delay(polling_interval_ms_);
  }
}

VL53L1_Error Roode::read_and_track_zone_(Zone *zone, bool ut) {
  if (use_lux_ && lux_sensor_ && !std::isnan(lux_sensor_->state)) {
    if (lux_sensor_->state < (lux_avg_ * 0.5f)) return VL53L1_ERROR_NONE;
  }
#ifdef USE_SUN
  if (use_sun_ && sun_ && sun_->elevation->state < 0.0f) return VL53L1_ERROR_NONE;
#endif
  if (!counting_enabled_) return VL53L1_ERROR_NONE;
  if (manual_presence_) return VL53L1_ERROR_NONE;

#ifdef CONFIG_IDF_TARGET_ESP32
  i2c_lock();
#endif
  VL53L1_Error st = zone->readDistance(distanceSensor);
#ifdef CONFIG_IDF_TARGET_ESP32
  i2c_unlock();
#endif
  path_tracking(zone); return st;
}

void Roode::path_tracking(Zone *z) {
  uint16_t dist = z->getMinDistance();
  int cur = (dist < z->threshold->max && dist > z->threshold->min) ? SOMEONE : NOBODY;
  if (z->id == 0) LeftPreviousStatus = cur; else RightPreviousStatus = cur;
  AllZonesCurrentStatus = (LeftPreviousStatus == SOMEONE ? 1 : 0) + (RightPreviousStatus == SOMEONE ? 2 : 0);

  // Per-zone presence sensors (safe to call directly in single-core; skip in sensor_task)
  if (!use_sensor_task_) {
    if (z->id == 0 && entry_presence_sensor) entry_presence_sensor->publish_state(cur == SOMEONE);
    if (z->id == 1 && exit_presence_sensor) exit_presence_sensor->publish_state(cur == SOMEONE);
  }

  if (state_ == STATE_IDLE && AllZonesCurrentStatus != 0) {
    state_ = STATE_ACTIVE; state_started_ts = millis();
    // Someone just entered the doorway — update overall presence
    if (use_sensor_task_) { presence_state_ = true; presence_update_pending_ = true; }
    else if (presence_sensor) presence_sensor->publish_state(true);
  }

  if (AllZonesCurrentStatus == 0) {
    if (state_ == STATE_ACTIVE) {
      if (PathTrackFillingSize >= 2) {
        int start_zone = 0;
        for (int i = 0; i < PathTrackFillingSize; i++) {
          if (PathTrack[i] == 1 || PathTrack[i] == 2) {
            start_zone = PathTrack[i];
            break;
          }
        }
        int end_zone = 0;
        for (int i = PathTrackFillingSize - 1; i >= 0; i--) {
          if (PathTrack[i] == 1 || PathTrack[i] == 2) {
            end_zone = PathTrack[i];
            break;
          }
        }

        if (start_zone != 0 && end_zone != 0 && start_zone != end_zone) {
          if (start_zone == 2 && end_zone == 1) {
            updateCounter(1);
          } else if (start_zone == 1 && end_zone == 2) {
            updateCounter(-1);
          }
        }
      }
      PathTrackFillingSize = 0;
      state_ = STATE_IDLE;
      // Doorway is clear — clear overall presence
      if (use_sensor_task_) { presence_state_ = false; presence_update_pending_ = true; }
      else if (presence_sensor) presence_sensor->publish_state(false);
    }
  } else if (PathTrackFillingSize == 0 || AllZonesCurrentStatus != PathTrack[PathTrackFillingSize - 1]) {
    if (PathTrackFillingSize < 16) PathTrack[PathTrackFillingSize++] = AllZonesCurrentStatus;
  }
}

void Roode::updateCounter(int d) {
  if (invert_direction_) d = -d;
  if (entry_exit_event_sensor) entry_exit_event_sensor->publish_state(d > 0 ? "entry" : "exit");
  if (!people_counter) return;
  float nv = people_counter->state + d;
  if (use_sensor_task_) { pending_people_counter_value_ = nv; people_counter_update_pending_ = true; }
  else { auto c = people_counter->make_call(); c.set_value(nv); c.perform(); }
}

void Roode::recalibration() { run_zone_calibration(0); run_zone_calibration(1); }
void Roode::run_zone_calibration(uint8_t id) { Zone *z = id == 0 ? entry : exit; z->calibrateThreshold(distanceSensor, 50); }

#ifdef CONFIG_IDF_TARGET_ESP32
void Roode::sensor_task(void *p) {
  auto *self = static_cast<Roode *>(p);
#ifdef CONFIG_IDF_TARGET_ESP32
  esp_task_wdt_add(nullptr);
#endif
  // Run deferred calibration on first iteration: by now VL53L1X setup() has completed
  if (self->needs_initial_calibration_ && self->distanceSensor && !self->distanceSensor->is_failed()) {
    self->needs_initial_calibration_ = false;
    ESP_LOGI(TAG, "Running initial threshold calibration");
    self->recalibration();
    RoodeSettings cal_s; if (!self->settings_prefs_.load(&cal_s)) memset(&cal_s, 0, sizeof(cal_s));
    cal_s.entry_roi_height = self->entry->roi->height; cal_s.entry_roi_width = self->entry->roi->width; cal_s.entry_roi_center = self->entry->roi->center;
    cal_s.entry_min_threshold = self->entry->threshold->min; cal_s.entry_max_threshold = self->entry->threshold->max;
    cal_s.exit_roi_height = self->exit->roi->height; cal_s.exit_roi_width = self->exit->roi->width; cal_s.exit_roi_center = self->exit->roi->center;
    cal_s.exit_min_threshold = self->exit->threshold->min; cal_s.exit_max_threshold = self->exit->threshold->max;
    self->settings_prefs_.save(&cal_s); global_preferences->sync();
    // Signal main task (Core 0) to publish — publish_state() is not thread-safe from Core 1
    self->calibration_update_pending_ = true;
  }

  for (;;) {
#ifdef CONFIG_IDF_TARGET_ESP32
    esp_task_wdt_reset();
#endif
    if (self->scan_state_ == SCANNING) {
      HistoricalScan h; h.ts = (uint32_t)time(nullptr); h.lux = self->lux_sensor_ ? self->lux_sensor_->state : 0;
      for (uint8_t i = 0; i < 64; i++) {
        ROI roi; roi.width = 4; roi.height = 4; roi.center = ((i/8)*2+1)*16 + ((i%8)*2+1);
        VL53L1_Error st; auto d = self->distanceSensor->read_distance(&roi, st); h.distances.push_back(d.value_or(0));
        self->scan_progress_ = (i * 100) / 64; vTaskDelay(pdMS_TO_TICKS(20));
      }
      
      if (self->scan_phase_ == PHASE_EMPTY) {
        self->active_session_.id = session_id_counter++; self->active_session_.ts = h.ts; self->active_session_.background = h; self->active_session_.complete = false;
      } else if (self->scan_phase_ == PHASE_PERSON) {
        self->active_session_.person = h; self->active_session_.complete = true;
        self->sessions_.push_back(self->active_session_);
        
        uint8_t bi = 0; int32_t md = 0;
        for(uint8_t i=0; i<64; i++) {
          int32_t delta = (int32_t)self->active_session_.background.distances[i] - (int32_t)self->active_session_.person.distances[i];
          if (delta > md) { md = delta; bi = i; }
        }
        self->recommended_settings_.entry_roi_center = ((bi/8)*2+1)*16 + ((bi%8)*2+1);
        self->recommended_settings_.entry_max_threshold = (uint16_t)md; self->has_recommended_settings_ = true;
      }
      self->scan_state_ = SCAN_IDLE; continue;
    }
    self->read_and_track_zone_(self->entry, true); self->read_and_track_zone_(self->exit, false);
    vTaskDelay(pdMS_TO_TICKS(self->polling_interval_ms_));
  }
}
#endif

void Roode::restart_sensor() { if (distanceSensor) distanceSensor->restart(); }

#ifdef USE_WEBSERVER
bool Roode::portal_request_authorized_(roode_web::AsyncWebServerRequest *r) const {
  if (portal_password_.empty()) return true;
#ifdef CONFIG_IDF_TARGET_ESP32
  auto h = r->get_header("X-Portal-Token"); if (h.has_value() && h.value() == portal_password_) return true;
#else
  if (r->hasHeader("X-Portal-Token") && r->header("X-Portal-Token").c_str() == portal_password_) return true;
#endif
  auto *t = r->getParam("token");
#ifdef CONFIG_IDF_TARGET_ESP32
  if (t && t->value() == portal_password_) return true;
#else
  if (t && t->value().c_str() == portal_password_) return true;
#endif
  return false;
}
bool Roode::require_portal_auth_(roode_web::AsyncWebServerRequest *r, const char *ct) const {
  if (!portal_enabled_) { r->send(403, "text/plain", "Portal disabled"); return false; }
  if (portal_request_authorized_(r)) return true;
  if (ct && std::string(ct) == "text/html") send_portal_login_(r); else r->send(401, "application/json", "{\"error\":\"unauthorized\"}");
  return false;
}
void Roode::send_portal_login_(roode_web::AsyncWebServerRequest *r) const { r->send(200, "text/html", portal_login_html); }
#endif

void Roode::publish_static_states_() {
  // Publish all static/initial sensor states so HA marks entities as available
  // immediately. Without an initial publish_state(), HA shows "unavailable".
  if (version_sensor) version_sensor->publish_state(VERSION);
  if (status_text_sensor) status_text_sensor->publish_state("Initializing");
  if (enabled_features_sensor) {
    std::string f;
    if (distanceSensor) {
      if (distanceSensor->is_interrupt_enabled()) f += "interrupt ";
      if (distanceSensor->get_xshut_state().has_value()) f += "xshut ";
    }
    enabled_features_sensor->publish_state(f.empty() ? "none" : f);
  }
  if (presence_sensor) presence_sensor->publish_state(false);
  if (entry_presence_sensor) entry_presence_sensor->publish_state(false);
  if (exit_presence_sensor) exit_presence_sensor->publish_state(false);
  if (xshut_state_sensor) xshut_state_sensor->publish_state(false);
  if (entry_exit_event_sensor) entry_exit_event_sensor->publish_state("");
  if (manual_adjustment_sensor) manual_adjustment_sensor->publish_state(0);
  if (interrupt_status_sensor) interrupt_status_sensor->publish_state(0);
  if (status_sensor) status_sensor->publish_state(0);
  if (loop_time_sensor) loop_time_sensor->publish_state(0);
  if (cpu_usage_sensor) cpu_usage_sensor->publish_state(0);
  if (ram_free_sensor) ram_free_sensor->publish_state(0);
  if (flash_free_sensor) flash_free_sensor->publish_state(0);
  if (distance_entry) distance_entry->publish_state(0);
  if (distance_exit) distance_exit->publish_state(0);
  // people_counter is a Number — use publish_state directly so it doesn't
  // trigger side-effects before the component is fully initialized.
  if (people_counter) people_counter->publish_state(0);
}

void Roode::publish_threshold_and_roi_states_() {
  if (max_threshold_entry_sensor) max_threshold_entry_sensor->publish_state(entry->threshold->max);
  if (min_threshold_entry_sensor) min_threshold_entry_sensor->publish_state(entry->threshold->min);
  if (max_threshold_exit_sensor) max_threshold_exit_sensor->publish_state(exit->threshold->max);
  if (min_threshold_exit_sensor) min_threshold_exit_sensor->publish_state(exit->threshold->min);
  if (entry_roi_height_sensor) entry_roi_height_sensor->publish_state(entry->roi->height);
  if (entry_roi_width_sensor) entry_roi_width_sensor->publish_state(entry->roi->width);
  if (exit_roi_height_sensor) exit_roi_height_sensor->publish_state(exit->roi->height);
  if (exit_roi_width_sensor) exit_roi_width_sensor->publish_state(exit->roi->width);
}

} }
