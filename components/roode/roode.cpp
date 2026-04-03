#include "roode.h"
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
class LambdaRequestHandler : public web_server_idf::AsyncWebHandler {
 public:
  enum MatchType { EXACT, PREFIX };
  using Callback = std::function<void(web_server_idf::AsyncWebServerRequest *request)>;
  LambdaRequestHandler(http_method method, std::string uri, Callback callback, MatchType match_type = EXACT)
      : method_(method), uri_(std::move(uri)), callback_(std::move(callback)), match_type_(match_type) {}
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override {
    if (request->method() != method_) return false;
    char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    std::string request_url(request->url_to(url_buf));
    return match_type_ == PREFIX ? request_url.rfind(uri_, 0) == 0 : request_url == uri_;
  }
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override { callback_(request); }
 private:
  http_method method_; std::string uri_; Callback callback_; MatchType match_type_;
};
}
#endif

static const char *const portal_login_html = R"LOGIN(
<!doctype html><html><head><meta charset="utf-8" /><title>Login</title><style>body{background:#0b0f14;color:#e6edf3;font-family:sans-serif;display:grid;place-items:center;min-height:100vh}form{background:#121821;padding:24px;border-radius:16px;border:1px solid #1f2835}input{width:100%;padding:12px;margin:12px 0;background:transparent;color:inherit;border:1px solid #1f2835;border-radius:8px}button{width:100%;padding:12px;background:#57a6ff;color:#fff;border:0;border-radius:8px;cursor:pointer}</style></head><body><form id="f"><h1>Roode</h1><input id="t" type="password" placeholder="Password" /><button type="submit">Login</button></form><script>document.getElementById('f').onsubmit=(e)=>{e.preventDefault();location.href='/portal?token='+encodeURIComponent(document.getElementById('t').value);};</script></body></html>
)LOGIN";

static const char *const portal_html = R"PORTAL(
<!doctype html><html><head><meta charset="utf-8" /><title>Roode Portal</title><style>:root{--bg:#0b0f14;--panel:#121821;--text:#e6edf3;--muted:#9fb0c3;--acc:#57a6ff;--ok:#5bd19b;--err:#ff6b6b;--line:#1f2835}body{margin:0;background:var(--bg);color:var(--text);font-family:sans-serif}.wrap{max-width:800px;margin:24px auto;padding:0 16px}.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:16px}.tabs{display:flex;gap:4px;border-bottom:1px solid var(--line);margin-bottom:16px}.tab{padding:8px 16px;cursor:pointer}.tab.active{color:var(--acc);border-bottom:2px solid var(--acc)}.tab-content{display:none}.tab-content.active{display:block}.progress{height:8px;background:#0f1520;border-radius:4px;overflow:hidden;margin:12px 0}.progress-bar{height:100%;background:var(--acc);width:0%}.btns{display:flex;gap:8px}button{padding:8px 16px;background:var(--line);color:inherit;border:0;border-radius:8px;cursor:pointer}button.primary{background:var(--acc);color:#fff}.kv{display:grid;grid-template-columns:140px 1fr;gap:4px}.muted{color:var(--muted);font-size:12px}</style></head><body><div class="wrap"><div class="card"><h1>Roode Portal</h1><div id="st">Sensor: <b>Idle</b> | HA Time: <b>No Sync</b></div></div><div class="tabs"><div class="tab active" data-t="c">Calibration</div><div class="tab" data-t="s">Settings</div><div class="tab" data-t="e">Entities</div></div><div id="c" class="tab-content active"><div class="card"><h2>Phase Calibration</h2><select id="ph"><option value="1">Background</option><option value="2">Person</option></select><div class="progress"><div id="pb" class="progress-bar"></div></div><div class="btns"><button id="start" class="primary">Start Scan</button></div></div><div class="card"><h2>Recommendation</h2><div class="kv"><div>Center:</div><div id="rc">—</div><div>Contrast:</div><div id="ct">—</div></div><button id="apply" class="primary" style="margin-top:12px" disabled>Apply ROI</button></div></div><div id="s" class="tab-content"><div class="card"><form id="sf"><label><input type="checkbox" name="d"> Debug Mode</label><br/><br/>Sampling:<br/><input type="number" name="sa" /><br/><br/>Interval:<br/><input type="number" name="pi" /><br/><br/><button type="submit" class="primary">Save</button></form></div></div><div id="e" class="tab-content"><div class="card" id="el"></div><button id="se" class="primary">Save Entities</button></div></div><script>const T=new URLSearchParams(location.search).get('token')||'';const H=T?{'X-Portal-Token':T}:{};const $=s=>document.querySelector(s);document.querySelectorAll('.tab').forEach(t=>t.onclick=()=>{document.querySelectorAll('.tab,.tab-content').forEach(x=>x.classList.remove('active'));t.classList.add('active');$('#'+t.dataset.t).classList.add('active')});async function poll(){try{const [cur,stat,prev]=await Promise.all([fetch('/api/settings/current',{headers:H}).then(r=>r.json()),fetch('/api/scan/status',{headers:H}).then(r=>r.json()),fetch('/api/roi/preview',{headers:H}).then(r=>r.json())]);if(cur){$('#st').innerHTML=`Sensor: <b>${cur.status}</b> | HA Time: <b>${cur.ts?'Synced':'No Sync'}</b>`;const f=$('#sf');if(!f.dataset.d){f.d.checked=cur.debug;f.sa.value=cur.sa;f.pi.value=cur.pi}if(!$('#el').dataset.l){const m=['Zone 0','Zone 1','Presence','Counter','Metrics'];let h='';m.forEach((n,i)=>h+=`<label><input type="checkbox" data-b="${i}" ${cur.m&(1<<i)?'checked':''}> ${n}</label><br/>`);$('#el').innerHTML=h;$('#el').dataset.l=1}}if(stat){$('#pb').style.width=stat.p+'%';$('#start').disabled=stat.s==='Scanning'}if(prev&&prev.roi!==undefined){$('#rc').textContent=prev.roi;$('#ct').textContent=prev.con+'mm';$('#apply').disabled=false}}catch(e){}}setInterval(poll,2000);poll();$('#start').onclick=()=>{const p=new URLSearchParams();p.append('phase',$('#ph').value);if(T)p.append('token',T);fetch('/api/scan/start?'+p.toString(),{method:'POST',headers:H})};$('#apply').onclick=()=>fetch('/api/roi/apply',{method:'POST',headers:H});$('#sf').onsubmit=(e)=>{e.preventDefault();const p=new URLSearchParams();p.append('debug',e.target.d.checked);p.append('sa',e.target.sa.value);p.append('pi',e.target.pi.value);if(T)p.append('token',T);fetch('/api/settings/update?'+p.toString(),{method:'POST',headers:H})};$('#se').onclick=()=>{let m=0;document.querySelectorAll('#el input').forEach(i=>{if(i.checked)m|=(1<<i.dataset.b)});const p=new URLSearchParams();p.append('m',m);if(T)p.append('token',T);fetch('/api/settings/update?'+p.toString(),{method:'POST',headers:H})};</script></body></html>
)PORTAL";

Roode *Roode::instance_ = nullptr;
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
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/portal", [this](web_server_idf::AsyncWebServerRequest *r) {
    if (require_portal_auth_(r, "text/html")) r->send(200, "text/html", portal_html);
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/settings/current", [this](web_server_idf::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    JsonDocument doc; doc["sa"] = samples; doc["pi"] = polling_interval_ms_; doc["debug"] = debug_mode_;
    doc["m"] = active_sensors_; doc["status"] = (distanceSensor && !distanceSensor->is_failed()) ? "OK" : "Offline";
    doc["ts"] = (time(nullptr) > 1000000); std::string out; serializeJson(doc, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/settings/update", [this](web_server_idf::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    RoodeSettings s; s.sampling = r->hasParam("sa") ? atoi(r->getParam("sa")->value().c_str()) : samples;
    s.polling_interval = r->hasParam("pi") ? atoi(r->getParam("pi")->value().c_str()) : polling_interval_ms_;
    s.debug_mode = r->hasParam("debug") ? (r->getParam("debug")->value() == "true") : debug_mode_;
    s.active_sensors = r->hasParam("m") ? strtoul(r->getParam("m")->value().c_str(), NULL, 10) : active_sensors_;
    s.roi_center = entry->roi->center; s.roi_width = entry->roi->width; s.roi_height = entry->roi->height;
    s.filter_mode = filter_mode_; s.filter_window = filter_window_; s.invert_direction = invert_direction_;
    samples = s.sampling; polling_interval_ms_ = s.polling_interval; debug_mode_ = s.debug_mode; active_sensors_ = s.active_sensors;
    settings_prefs_.save(&s); global_preferences->sync(); r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/start", [this](web_server_idf::AsyncWebServerRequest *r) {
    if (!require_portal_auth_(r, "application/json")) return;
    scan_phase_ = r->hasParam("phase") ? (ScanPhase)atoi(r->getParam("phase")->value().c_str()) : PHASE_EMPTY;
    scan_state_ = SCANNING; scan_progress_ = 0; r->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/status", [this](web_server_idf::AsyncWebServerRequest *r) {
    JsonDocument d; d["s"] = scan_state_ == SCANNING ? "Scanning" : "Idle"; d["p"] = scan_progress_;
    std::string out; serializeJson(d, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/roi/preview", [this](web_server_idf::AsyncWebServerRequest *r) {
    if (!has_recommended_settings_) { r->send(200, "application/json", "null"); return; }
    JsonDocument d; d["roi"] = recommended_settings_.roi_center; d["con"] = (int)recommended_settings_.max_threshold;
    std::string out; serializeJson(d, out); r->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/roi/apply", [this](web_server_idf::AsyncWebServerRequest *r) {
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
  if (active_sensors_ & 0x10 && ram_free_sensor) ram_free_sensor->publish_state((float)ESP.getFreeHeap()/(float)ESP.getHeapSize()*100.0f);
  loop_window_start_ = now;
}

void Roode::loop() {
  if (use_sensor_task_) {
    if (presence_update_pending_ && active_sensors_ & 0x04 && presence_sensor) { presence_sensor->publish_state(presence_state_); presence_update_pending_ = false; }
    if (people_counter_update_pending_ && active_sensors_ & 0x08 && people_counter_sensor) { auto c = people_counter_sensor->make_call(); c.set_value(pending_people_counter_value_); c.perform(); people_counter_update_pending_ = false; }
    vTaskDelay(pdMS_TO_TICKS(50));
  } else {
    read_and_track_zone_(entry, true); read_and_track_zone_(exit, false); delay(polling_interval_ms_);
  }
}

VL53L1_Error Roode::read_and_track_zone_(Zone *zone, bool ut) {
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
  if (!people_counter_sensor) return;
  float nv = people_counter_sensor->state + d;
  if (use_sensor_task_) { pending_people_counter_value_ = nv; people_counter_update_pending_ = true; }
  else { auto c = people_counter_sensor->make_call(); c.set_value(nv); c.perform(); }
}

void Roode::recalibration() { run_zone_calibration(0); run_zone_calibration(1); }
void Roode::run_zone_calibration(uint8_t id) { Zone *z = id == 0 ? entry : exit; z->calibrateThreshold(distanceSensor, 50); }

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
      if (self->scan_phase_ == PHASE_EMPTY) self->background_scans_.push_back(h); else self->person_scans_.push_back(h);
      if (!self->background_scans_.empty() && !self->person_scans_.empty()) {
        uint8_t bi = 0; int32_t md = 0;
        for(uint8_t i=0; i<64; i++) {
          int32_t delta = (int32_t)self->background_scans_.back().distances[i] - (int32_t)self->person_scans_.back().distances[i];
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

void Roode::restart_sensor() { if (distanceSensor) distanceSensor->restart(); }

#ifdef USE_WEBSERVER
bool Roode::portal_request_authorized_(web_server_idf::AsyncWebServerRequest *r) const {
  if (portal_password_.empty()) return true;
  auto h = r->get_header("X-Portal-Token"); if (h.has_value() && h.value() == portal_password_) return true;
  auto *t = r->getParam("token"); if (t && t->value() == portal_password_) return true;
  return false;
}
bool Roode::require_portal_auth_(web_server_idf::AsyncWebServerRequest *r, const char *ct) const {
  if (!portal_enabled_) { r->send(403, "text/plain", "Portal disabled"); return false; }
  if (portal_request_authorized_(r)) return true;
  if (ct && std::string(ct) == "text/html") send_portal_login_(r); else r->send(401, "application/json", "{\"error\":\"unauthorized\"}");
  return false;
}
void Roode::send_portal_login_(web_server_idf::AsyncWebServerRequest *r) const { r->send(200, "text/html", portal_login_html); }
#endif

} }
