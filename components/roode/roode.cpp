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

void Roode::log_event(const std::string &msg) { if (instance_ && instance_->debug_mode_) ESP_LOGI(TAG, "%s", msg.c_str()); }
void Roode::dump_config() { ESP_LOGCONFIG(TAG, "Roode v%s", VERSION); }
Roode::~Roode() { instance_ = nullptr; }

void Roode::setup() {
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!i2c_mutex_) i2c_mutex_ = xSemaphoreCreateRecursiveMutex();
#endif
  settings_prefs_ = global_preferences->make_preference<RoodeSettings>(0xB0);
  RoodeSettings s; if (settings_prefs_.load(&s)) {
    samples = s.sampling; polling_interval_ms_ = s.polling_interval; invert_direction_ = s.invert_direction;
    filter_mode_ = s.filter_mode; filter_window_ = s.filter_window; active_sensors_ = s.active_sensors;
    debug_mode_ = s.debug_mode; entry->roi->center = exit->roi->center = s.roi_center;
    entry->set_debug_mode(debug_mode_); exit->set_debug_mode(debug_mode_);
    auto_calibration_interval_sec_ = s.auto_calibration_interval; restart_timeout_ms_ = s.restart_timeout;
    cpu_opt_activate_threshold_ = s.cpu_activate; cpu_opt_deactivate_threshold_ = s.cpu_deactivate;
    manual_presence_ = s.manual_presence; invalid_distance_limit_ = s.invalid_limit;
    lux_threshold_ = s.lux_threshold; sun_elevation_threshold_enabled_ = s.sun_elevation_threshold_enabled;
    sun_elevation_threshold_ = s.sun_elevation_threshold;
  }
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!force_single_core_) {
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
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
    JsonDocument doc; doc["sa"] = samples; doc["pi"] = polling_interval_ms_; doc["debug"] = debug_mode_;
    doc["m"] = active_sensors_; doc["status"] = (distanceSensor && !distanceSensor->is_failed()) ? "OK" : "Offline";
    doc["ts"] = (time(nullptr) > 1000000); doc["inv"] = invert_direction_; doc["fm"] = (int)filter_mode_;
    doc["fw"] = filter_window_; doc["min"] = entry->threshold->min; doc["max"] = entry->threshold->max;
    doc["ac"] = auto_calibration_interval_sec_; doc["il"] = invalid_distance_limit_; doc["rt"] = restart_timeout_ms_;
    doc["lt"] = lux_threshold_; doc["se"] = sun_elevation_threshold_enabled_; doc["st"] = sun_elevation_threshold_;
    doc["pres"] = manual_presence_; doc["rc"] = entry->roi->center; doc["rw"] = entry->roi->width; doc["rh"] = entry->roi->height;
    doc["ec"] = entry->roi->center; doc["xc"] = exit->roi->center;
    std::string out; serializeJson(doc, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/settings/update", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    RoodeSettings s;
    s.sampling = r->hasParam("sa") ? atoi(r->getParam("sa")->value().c_str()) : samples;
    s.polling_interval = r->hasParam("pi") ? atoi(r->getParam("pi")->value().c_str()) : polling_interval_ms_;
    s.debug_mode = r->hasParam("debug") ? (r->getParam("debug")->value() == "true") : debug_mode_;
    s.active_sensors = r->hasParam("m") ? strtoul(r->getParam("m")->value().c_str(), NULL, 10) : active_sensors_;
    s.invert_direction = r->hasParam("inv") ? (r->getParam("inv")->value() == "true") : invert_direction_;
    s.filter_mode = r->hasParam("fm") ? (FilterMode)atoi(r->getParam("fm")->value().c_str()) : filter_mode_;
    s.filter_window = r->hasParam("fw") ? atoi(r->getParam("fw")->value().c_str()) : filter_window_;
    s.auto_calibration_interval = r->hasParam("ac") ? strtoul(r->getParam("ac")->value().c_str(), NULL, 10) : auto_calibration_interval_sec_;
    s.invalid_limit = r->hasParam("il") ? atoi(r->getParam("il")->value().c_str()) : invalid_distance_limit_;
    s.restart_timeout = r->hasParam("rt") ? atoi(r->getParam("rt")->value().c_str()) : restart_timeout_ms_;
    s.lux_threshold = r->hasParam("lt") ? atof(r->getParam("lt")->value().c_str()) : lux_threshold_;
    s.sun_elevation_threshold_enabled = r->hasParam("se") ? (r->getParam("se")->value() == "true") : sun_elevation_threshold_enabled_;
    s.sun_elevation_threshold = r->hasParam("st") ? atof(r->getParam("st")->value().c_str()) : sun_elevation_threshold_;
    s.manual_presence = r->hasParam("pres") ? (r->getParam("pres")->value() == "true") : manual_presence_;
    
    s.roi_center = entry->roi->center; s.roi_width = entry->roi->width; s.roi_height = entry->roi->height;
    s.entry_center = entry->roi->center; s.exit_center = exit->roi->center;
    s.min_threshold = entry->threshold->min; s.max_threshold = entry->threshold->max;
    s.cpu_activate = cpu_opt_activate_threshold_; s.cpu_deactivate = cpu_opt_deactivate_threshold_;

    samples = s.sampling; polling_interval_ms_ = s.polling_interval; debug_mode_ = s.debug_mode;
    active_sensors_ = s.active_sensors; invert_direction_ = s.invert_direction;
    filter_mode_ = s.filter_mode; filter_window_ = s.filter_window;
    auto_calibration_interval_sec_ = s.auto_calibration_interval;
    invalid_distance_limit_ = s.invalid_limit; restart_timeout_ms_ = s.restart_timeout;
    lux_threshold_ = s.lux_threshold; sun_elevation_threshold_enabled_ = s.sun_elevation_threshold_enabled;
    sun_elevation_threshold_ = s.sun_elevation_threshold; manual_presence_ = s.manual_presence;
    entry->set_debug_mode(debug_mode_); exit->set_debug_mode(debug_mode_);

    settings_prefs_.save(&s); global_preferences->sync(); r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/presence/toggle", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    if (r->hasParam("state")) {
      manual_presence_ = (r->getParam("state")->value() == "true");
      RoodeSettings s; if (settings_prefs_.load(&s)) { s.manual_presence = manual_presence_; settings_prefs_.save(&s); global_preferences->sync(); }
    }
    r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/start", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    scan_phase_ = r->hasParam("phase") ? (ScanPhase)atoi(r->getParam("phase")->value().c_str()) : PHASE_EMPTY;
    scan_state_ = SCANNING; scan_progress_ = 0; r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/status", [this](roode_web::AsyncWebServerRequest *r) {
    JsonDocument d; d["s"] = scan_state_ == SCANNING ? "Scanning" : "Idle"; d["p"] = scan_progress_;
    std::string out; serializeJson(d, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/sessions", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    JsonDocument doc; JsonArray arr = doc.to<JsonArray>();
    for (size_t i = 0; i < sessions_.size(); i++) {
      JsonObject o = arr.add<JsonObject>(); o["id"] = std::to_string(i);
      o["bg_lux"] = sessions_[i].background.lux; o["p_lux"] = sessions_[i].person.lux;
      o["complete"] = sessions_[i].complete;
      char buf[32]; struct tm *tm = localtime((time_t *)&sessions_[i].ts);
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm); o["date"] = std::string(buf);
    }
    std::string out; serializeJson(doc, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/delete", [this](roode_web::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    if (r->hasParam("id")) {
      size_t idx = atoi(r->getParam("id")->value().c_str());
      if (idx < sessions_.size()) sessions_.erase(sessions_.begin() + idx);
    }
    r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/roi/preview", [this](roode_web::AsyncWebServerRequest *r) {
    if (!has_recommended_settings_) { r->send(200, "application/json", "null"); return; }
    JsonDocument d; d["roi"] = recommended_settings_.roi_center; d["con"] = (int)recommended_settings_.max_threshold;
    std::string out; serializeJson(d, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/roi/apply", [this](roode_web::AsyncWebServerRequest *r) {
    entry->roi->center = exit->roi->center = recommended_settings_.roi_center;
    recalibration(); r->send(200, "application/json", "{\"ok\":true}");
  }));
#endif
}

void Roode::update() {
  if (active_sensors_ & 0x01 && distance_entry) distance_entry->publish_state(entry->getDistance());
  if (active_sensors_ & 0x02 && distance_exit) distance_exit->publish_state(exit->getDistance());
  update_metrics();
}

void Roode::update_metrics() {
  uint32_t now = millis(); if (now - loop_window_start_ < 10000) return;
#ifdef CONFIG_IDF_TARGET_ESP32
  if (active_sensors_ & 0x10 && ram_free_sensor) ram_free_sensor->publish_state((float)ESP.getFreeHeap()/(float)ESP.getHeapSize()*100.0f);
#endif
  loop_window_start_ = now;
}

void Roode::loop() {
  if (use_sensor_task_) {
#ifdef CONFIG_IDF_TARGET_ESP32
    if (presence_update_pending_ && active_sensors_ & 0x04 && presence_sensor) { presence_sensor->publish_state(presence_state_); presence_update_pending_ = false; }
    if (people_counter_update_pending_ && active_sensors_ & 0x08 && people_counter) { auto c = people_counter->make_call(); c.set_value(pending_people_counter_value_); c.perform(); people_counter_update_pending_ = false; }
    vTaskDelay(pdMS_TO_TICKS(50));
#endif
  } else {
    read_and_track_zone_(entry, true); read_and_track_zone_(exit, false); delay(polling_interval_ms_);
  }
}

VL53L1_Error Roode::read_and_track_zone_(Zone *zone, bool ut) {
  if (lux_sensor_ && lux_sensor_->state < lux_threshold_) return VL53L1_ERROR_NONE;
#ifdef USE_SUN
  if (sun_elevation_threshold_enabled_ && sun_ && sun_->elevation->state < sun_elevation_threshold_) return VL53L1_ERROR_NONE;
#endif
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
  if (state_ == STATE_IDLE && AllZonesCurrentStatus != 0) { state_ = STATE_ACTIVE; state_started_ts = millis(); }
  if (AllZonesCurrentStatus == 0) {
    if (PathTrackFillingSize >= 3) {
      if (PathTrack[PathTrackFillingSize-3] == 2 && PathTrack[PathTrackFillingSize-2] == 3 && PathTrack[PathTrackFillingSize-1] == 1) updateCounter(1);
      else if (PathTrack[PathTrackFillingSize-3] == 1 && PathTrack[PathTrackFillingSize-2] == 3 && PathTrack[PathTrackFillingSize-1] == 2) updateCounter(-1);
    }
    PathTrackFillingSize = 0; state_ = STATE_IDLE;
  } else if (PathTrackFillingSize == 0 || AllZonesCurrentStatus != PathTrack[PathTrackFillingSize - 1]) {
    if (PathTrackFillingSize < 4) PathTrack[PathTrackFillingSize++] = AllZonesCurrentStatus;
  }
}

void Roode::updateCounter(int d) {
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
        self->active_session_.ts = h.ts;
        self->active_session_.background = h;
        self->active_session_.complete = false;
      } else if (self->scan_phase_ == PHASE_PERSON) {
        self->active_session_.person = h;
        self->active_session_.complete = true;
        self->sessions_.push_back(self->active_session_);
        
        // Calculate recommended ROI using background and person scans
        uint8_t bi = 0; int32_t md = 0;
        for(uint8_t i=0; i<64; i++) {
          int32_t delta = (int32_t)self->active_session_.background.distances[i] - (int32_t)self->active_session_.person.distances[i];
          if (delta > md) { md = delta; bi = i; }
        }
        self->recommended_settings_.roi_center = ((bi/8)*2+1)*16 + ((bi%8)*2+1);
        self->recommended_settings_.max_threshold = (uint16_t)md; self->has_recommended_settings_ = true;
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

} }
