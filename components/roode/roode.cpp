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

static const char *const portal_login_html = R"LOGIN(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Roode Portal Login</title>
  <style>
    :root { --bg:#0b0f14; --panel:#121821; --text:#e6edf3; --muted:#9fb0c3; --acc:#57a6ff; --line:#1f2835; }
    *{box-sizing:border-box}
    body{margin:0;min-height:100vh;display:grid;place-items:center;background:var(--bg);color:var(--text);font:14px/1.5 system-ui,sans-serif}
    .card{width:min(420px,calc(100vw - 32px));background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:24px}
    h1{margin:0 0 8px;font-size:20px}
    input{width:100%;padding:12px 14px;border:1px solid var(--line);border-radius:10px;background:transparent;color:inherit;margin-bottom:12px}
    button{width:100%;padding:12px 14px;border:0;border-radius:10px;background:var(--acc);color:#fff;font-weight:700;cursor:pointer}
  </style>
</head>
<body>
  <form class="card" id="loginForm">
    <h1>Roode Portal</h1>
    <input id="token" type="password" placeholder="Password" autofocus />
    <button type="submit">Open Portal</button>
  </form>
  <script>
    document.getElementById('loginForm').onsubmit = (e) => {
      e.preventDefault();
      location.href = '/portal?token=' + encodeURIComponent(document.getElementById('token').value);
    };
  </script>
</body>
</html>
)LOGIN";

static const char *const portal_html = R"PORTAL(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Roode Calibration Portal</title>
  <style>
    :root { --bg:#0b0f14; --panel:#121821; --text:#e6edf3; --muted:#9fb0c3; --acc:#57a6ff; --ok:#5bd19b; --warn:#ffcc66; --err:#ff6b6b; --line:#1f2835; }
    *{box-sizing:border-box}
    body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 system-ui,sans-serif}
    .wrap{max-width:1000px;margin:24px auto;padding:0 16px}
    .hdr{display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}
    .banner{display:none;background:var(--err);color:#fff;padding:8px 12px;border-radius:8px;margin-bottom:12px}
    .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:16px;margin-bottom:16px}
    .card h2{margin:0 0 10px 0;font-size:14px;color:var(--acc)}
    .btns{display:flex;gap:8px;flex-wrap:wrap}
    button,.btn{appearance:none;border:1px solid var(--line);background:#0f1520;color:var(--text);padding:8px 12px;border-radius:10px;font-weight:600;cursor:pointer}
    .btn.primary{background:var(--acc);color:#fff}
    .progress{height:10px;border-radius:8px;background:#0f1520;border:1px solid var(--line);overflow:hidden;margin:10px 0}
    .progress>div{height:100%;background:linear-gradient(90deg,#4cc2ff,#57a6ff);width:0%}
    .tabs{display:flex;gap:4px;margin-bottom:16px;border-bottom:1px solid var(--line)}
    .tab{padding:8px 16px;cursor:pointer;border-radius:8px 8px 0 0}
    .tab.active{background:var(--panel);color:var(--acc);font-weight:600}
    .tab-content{display:none}
    .tab-content.active{display:block}
    .form-row{margin-bottom:12px}
    .form-row label{display:block;margin-bottom:4px;color:var(--muted);font-size:12px}
    .form-row input, .form-row select{width:100%;padding:8px;border:1px solid var(--line);border-radius:8px;background:transparent;color:inherit}
    .chk-row{display:flex;align-items:center;gap:8px;margin-bottom:8px;cursor:pointer}
    .kv{display:grid;grid-template-columns:140px 1fr;gap:4px}
    .k{color:var(--muted)}
  </style>
</head>
<body>
  <div class="wrap">
    <div id="errorBanner" class="banner"></div>
    <div class="hdr"><h1>Roode Portal</h1><div id="statusText">Status: <b>Idle</b></div></div>
    <div class="tabs">
      <div class="tab active" data-tab="cal">Calibration</div>
      <div class="tab" data-tab="set">Settings</div>
      <div class="tab" data-tab="ent">Entities</div>
    </div>
    <div id="cal" class="tab-content active">
      <div class="card">
        <h2>Phase-Based Calibration</h2>
        <p class="muted">1. Empty the room and scan Background. 2. Stand in the doorway and scan Person. 3. Apply recommended contrast ROI.</p>
        <div class="form-row">
          <label>Scan Target</label>
          <select id="scanTarget"><option value="1">Background (Empty Room)</option><option value="2">Person (Target Object)</option></select>
        </div>
        <div class="btns">
          <button class="btn primary" id="btnStart">Start Grid Scan</button>
          <button class="btn" id="btnCancel" style="display:none">Cancel</button>
        </div>
        <div class="progress"><div id="progressBar"></div></div>
        <div id="stepText" class="muted">Ready</div>
      </div>
      <div class="card">
        <h2>Recommendations</h2>
        <div class="kv"><div class="k">ROI Center</div><div id="pvRoi">—</div><div class="k">Contrast</div><div id="pvContrast">—</div></div>
        <button class="btn primary" id="btnApply" disabled style="margin-top:10px">Apply & Save ROI</button>
      </div>
    </div>
    <div id="set" class="tab-content">
      <div class="card">
        <h2>Core Settings</h2>
        <form id="settingsForm">
          <div class="chk-row"><input type="checkbox" name="debug_mode"> Debug Mode (High Verbosity Logs)</div>
          <div class="form-row"><label>Sampling Size</label><input type="number" name="sampling"></div>
          <div class="form-row"><label>Polling Interval (ms)</label><input type="number" name="polling_interval"></div>
          <button type="submit" class="btn primary">Save Settings</button>
        </form>
      </div>
    </div>
    <div id="ent" class="tab-content">
      <div class="card">
        <h2>Entity Visibility</h2>
        <div id="sensorList">
          <label class="chk-row"><input type="checkbox" data-bit="0"> Zone 0 Distance</label>
          <label class="chk-row"><input type="checkbox" data-bit="1"> Zone 1 Distance</label>
          <label class="chk-row"><input type="checkbox" data-bit="2"> Presence</label>
          <label class="chk-row"><input type="checkbox" data-bit="3"> Counter</label>
          <label class="chk-row"><input type="checkbox" data-bit="4"> CPU/Ram</label>
        </div>
        <button class="btn primary" id="btnSaveEntities" style="margin-top:10px">Update Entities</button>
      </div>
    </div>
  </div>
  <script>
    const TOKEN = new URLSearchParams(location.search).get('token') || '';
    const TOKEN_HEADER = TOKEN ? {'X-Portal-Token': TOKEN} : {};
    const $ = (s)=>document.querySelector(s);
    
    document.querySelectorAll('.tab').forEach(t => t.onclick = () => {
      document.querySelectorAll('.tab, .tab-content').forEach(el => el.classList.remove('active'));
      t.classList.add('active'); $('#' + t.dataset.tab).classList.add('active');
    });

    async function poll(){
      const [cur, stat, prev] = await Promise.all([
        fetch('/api/settings/current', {headers: TOKEN_HEADER}).then(r=>r.json()),
        fetch('/api/scan/status', {headers: TOKEN_HEADER}).then(r=>r.json()),
        fetch('/api/roi/preview', {headers: TOKEN_HEADER}).then(r=>r.json())
      ]);
      if(cur){
        const f = $('#settingsForm'); if(!f.dataset.dirty){
          f.debug_mode.checked = cur.debug_mode; f.sampling.value = cur.sampling; f.polling_interval.value = cur.polling_interval;
        }
        if(!$('#sensorList').dataset.loaded){
          document.querySelectorAll('#sensorList input').forEach(i => i.checked = (cur.active_sensors & (1 << i.dataset.bit)));
          $('#sensorList').dataset.loaded = 'true';
        }
      }
      if(stat){
        $('#statusText').innerHTML = 'Status: <b>' + stat.state + '</b>';
        $('#progressBar').style.width = stat.progress + '%';
        $('#stepText').textContent = stat.step;
        $('#btnCancel').style.display = stat.state==='Scanning' ? 'inline-block' : 'none';
      }
      if(prev){ $('#pvRoi').textContent = prev.roi_center; $('#pvContrast').textContent = prev.contrast; $('#btnApply').disabled = false; }
    }

    $('#btnStart').onclick = () => {
      const p = new URLSearchParams(); p.append('phase', $('#scanTarget').value); if(TOKEN) p.append('token', TOKEN);
      fetch('/api/scan/start?' + p.toString(), {method:'POST', headers: TOKEN_HEADER});
    };
    $('#btnApply').onclick = () => fetch('/api/roi/apply', {method:'POST', headers: TOKEN_HEADER});
    $('#settingsForm').onsubmit = (e) => {
      e.preventDefault(); const p = new URLSearchParams();
      p.append('debug_mode', e.target.debug_mode.checked); p.append('sampling', e.target.sampling.value);
      p.append('polling_interval', e.target.polling_interval.value); if(TOKEN) p.append('token', TOKEN);
      fetch('/api/settings/update?' + p.toString(), {method:'POST', headers: TOKEN_HEADER});
    };
    $('#btnSaveEntities').onclick = () => {
      let m = 0; document.querySelectorAll('#sensorList input').forEach(i => { if(i.checked) m |= (1 << i.dataset.bit); });
      const p = new URLSearchParams(); p.append('active_sensors', m); if(TOKEN) p.append('token', TOKEN);
      fetch('/api/settings/update?' + p.toString(), {method:'POST', headers: TOKEN_HEADER});
    };
    setInterval(poll, 2000); poll();
  </script>
</body>
</html>
)PORTAL";

bool Roode::log_fallback_events_ = false;
Roode *Roode::instance_ = nullptr;
#ifdef CONFIG_IDF_TARGET_ESP32
SemaphoreHandle_t Roode::i2c_mutex_ = xSemaphoreCreateRecursiveMutex();
#endif

void Roode::log_event(const std::string &msg) {
  if (debug_mode_) ESP_LOGI(TAG, "%s", msg.c_str());
}

void Roode::setup() {
  entry->set_filter_window(filter_window_); entry->set_filter_mode(filter_mode_);
  exit->set_filter_window(filter_window_); exit->set_filter_mode(filter_mode_);
  settings_prefs_ = global_preferences->make_preference<RoodeSettings>(0xB0);
  RoodeSettings s; if (settings_prefs_.load(&s)) {
    this->set_sampling_size(s.sampling); this->polling_interval_ms_ = std::max((uint16_t)10, s.polling_interval);
    this->set_invert_direction(s.invert_direction); this->set_filter_mode(s.filter_mode);
    this->set_filter_window(s.filter_window); this->active_sensors_ = s.active_sensors; this->debug_mode_ = s.debug_mode;
    entry->roi->center = s.roi_center; exit->roi->center = s.roi_center;
  }
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!force_single_core_) {
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
    if (sensor_task_handle_ != nullptr) use_sensor_task_ = true;
  }
#endif
  register_portal_routes_();
}

void Roode::register_portal_routes_() {
#ifdef USE_WEBSERVER
  auto *base = web_server_base::global_web_server_base;
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/portal", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (this->require_portal_auth_(request, "text/html")) request->send(200, "text/html", portal_html);
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/settings/current", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    JsonDocument doc; doc["sampling"] = samples; doc["polling_interval"] = polling_interval_ms_;
    doc["debug_mode"] = debug_mode_; doc["active_sensors"] = active_sensors_;
    std::string out; serializeJson(doc, out); request->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/settings/update", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    RoodeSettings s; s.sampling = request->hasParam("sampling") ? atoi(request->getParam("sampling")->value().c_str()) : samples;
    s.polling_interval = request->hasParam("polling_interval") ? atoi(request->getParam("polling_interval")->value().c_str()) : polling_interval_ms_;
    s.debug_mode = request->hasParam("debug_mode") ? (request->getParam("debug_mode")->value() == "true") : debug_mode_;
    s.active_sensors = request->hasParam("active_sensors") ? strtoul(request->getParam("active_sensors")->value().c_str(), NULL, 10) : active_sensors_;
    s.roi_center = entry->roi->center;
    this->set_sampling_size(s.sampling); this->polling_interval_ms_ = std::max((uint16_t)10, s.polling_interval);
    this->debug_mode_ = s.debug_mode; this->active_sensors_ = s.active_sensors;
    settings_prefs_.save(&s); global_preferences->sync(); request->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/start", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    this->scan_phase_ = request->hasParam("phase") ? (ScanPhase)atoi(request->getParam("phase")->value().c_str()) : PHASE_EMPTY;
    this->scan_state_ = SCANNING; this->scan_progress_ = 0; request->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/status", [this](web_server_idf::AsyncWebServerRequest *request) {
    JsonDocument doc; doc["state"] = scan_state_ == SCANNING ? "Scanning" : "Idle"; doc["progress"] = scan_progress_;
    doc["step"] = scan_phase_ == PHASE_EMPTY ? "Background Scan" : "Person Scan";
    std::string out; serializeJson(doc, out); request->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/roi/preview", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!has_recommended_settings_) { request->send(200, "application/json", "null"); return; }
    JsonDocument doc; doc["roi_center"] = recommended_settings_.roi_center; doc["contrast"] = "High";
    std::string out; serializeJson(doc, out); request->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/roi/apply", [this](web_server_idf::AsyncWebServerRequest *request) {
    entry->roi->center = exit->roi->center = recommended_settings_.roi_center;
    this->recalibration(); request->send(200, "application/json", "{\"ok\":true}");
  }));
#endif
}

void Roode::update() {
  if (active_sensors_ & 0x01 && distance_entry) distance_entry->publish_state(entry->getDistance());
  if (active_sensors_ & 0x02 && distance_exit) distance_exit->publish_state(exit->getDistance());
}

void Roode::loop() {
  if (use_sensor_task_) {
    if (presence_update_pending_ && active_sensors_ & 0x04) { presence_sensor->publish_state(presence_state_); presence_update_pending_ = false; }
    if (people_counter_update_pending_ && active_sensors_ & 0x08) { auto c = people_counter->make_call(); c.set_value(pending_people_counter_value_); c.perform(); people_counter_update_pending_ = false; }
    vTaskDelay(pdMS_TO_TICKS(50));
  } else {
    read_and_track_zone_(entry, true); read_and_track_zone_(exit, false); update_metrics();
    if (auto_calibration_interval_sec_ > 0 && (millis() - last_calibration_millis_ >= auto_calibration_interval_sec_ * 1000)) recalibration();
    delay(polling_interval_ms_);
  }
}

VL53L1_Error Roode::read_and_track_zone_(Zone *zone, bool update_timestamp) {
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
  int cur = (z->getMinDistance() < z->threshold->max && z->getMinDistance() > z->threshold->min) ? SOMEONE : NOBODY;
  if (z->id == 0) LeftPreviousStatus = cur; else RightPreviousStatus = cur;
  AllZonesCurrentStatus = (LeftPreviousStatus == SOMEONE ? 1 : 0) + (RightPreviousStatus == SOMEONE ? 2 : 0);
  if (state_ == STATE_IDLE && AllZonesCurrentStatus != 0) { state_ = STATE_ENTRY_ACTIVE; state_started_ts = millis(); }
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
  float nv = people_counter->state + d;
  if (use_sensor_task_) { pending_people_counter_value_ = nv; people_counter_update_pending_ = true; }
  else { auto c = people_counter->make_call(); c.set_value(nv); c.perform(); }
}

void Roode::recalibration() { run_zone_calibration(0); run_zone_calibration(1); }
void Roode::run_zone_calibration(uint8_t id) {
  Zone *z = id == 0 ? entry : exit;
  z->calibrateThreshold(distanceSensor, 50);
  if (calibration_persistence_) calibration_prefs_[id].save(&calibration_data_[id]);
}

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
      HistoricalScan h; h.ts = millis()/1000; h.lux = self->lux_sensor_ ? self->lux_sensor_->state : 0;
      for (uint8_t i = 0; i < 64; i++) {
        ROI roi; roi.width = 4; roi.height = 4; roi.center = ((i/8)*2+1)*16 + ((i%8)*2+1);
        VL53L1_Error st; auto d = self->distanceSensor->read_distance(&roi, st);
        h.distances.push_back(d.value_or(0)); self->scan_progress_ = (i * 100) / 64; vTaskDelay(pdMS_TO_TICKS(20));
      }
      if (self->scan_phase_ == PHASE_EMPTY) self->background_scans_.push_back(h); else self->person_scans_.push_back(h);
      if (!self->background_scans_.empty() && !self->person_scans_.empty()) {
        // Analysis: Find ROI with max distance delta between background and person
        uint8_t bi = 0; int32_t md = 0;
        for(uint8_t i=0; i<64; i++) {
          int32_t delta = (int32_t)self->background_scans_.back().distances[i] - (int32_t)self->person_scans_.back().distances[i];
          if (delta > md) { md = delta; bi = i; }
        }
        self->recommended_settings_.roi_center = ((bi/8)*2+1)*16 + ((bi%8)*2+1); self->has_recommended_settings_ = true;
      }
      self->scan_state_ = SCAN_IDLE; continue;
    }
    self->read_and_track_zone_(self->entry, true); self->read_and_track_zone_(self->exit, false);
    vTaskDelay(pdMS_TO_TICKS(self->polling_interval_ms_));
  }
}

void Roode::dump_config() {}
void Roode::update_metrics() {}
void Roode::restart_sensor() { distanceSensor->restart(); }
const RangingMode *Roode::determine_ranging_mode(uint16_t ae, uint16_t ax) { return esphome::vl53l1x::Ranging::Long; }
bool Roode::handle_sensor_status() { return true; }
void Roode::update_status_text(const std::string &s) {}
void Roode::publish_feature_list() {}
bool Roode::pause_sensor_task_if_needed_() { return false; }
void Roode::resume_sensor_task_if_needed_(bool p) {}
void Roode::publish_sensor_configuration(Zone *e, Zone *x, bool m) {}
void Roode::apply_cpu_optimizations(float c) {}
void Roode::reset_cpu_optimizations(float c) {}

#ifdef USE_WEBSERVER
bool Roode::require_portal_auth_(web_server_idf::AsyncWebServerRequest *r, const char *c) const { return true; }
#endif

} }
