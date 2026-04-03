#include "roode.h"
#include "Arduino.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp_task_wdt.h"  // Access to the ESP32 task watchdog
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
    :root {
      --bg:#0b0f14; --panel:#121821; --text:#e6edf3; --muted:#9fb0c3; --acc:#57a6ff; --line:#1f2835;
    }
    @media (prefers-color-scheme: light) {
      :root {
        --bg:#f6f8fa; --panel:#ffffff; --text:#24292f; --muted:#57606a; --acc:#0969da; --line:#d0d7de;
      }
    }
    *{box-sizing:border-box}
    body{margin:0;min-height:100vh;display:grid;place-items:center;background:var(--bg);color:var(--text);font:14px/1.5 system-ui,-apple-system,Segoe UI,Roboto,Inter,sans-serif}
    .card{width:min(420px,calc(100vw - 32px));background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:24px}
    h1{margin:0 0 8px;font-size:20px}
    p{margin:0 0 16px;color:var(--muted)}
    label{display:block;margin-bottom:8px;font-weight:600}
    input{width:100%;padding:12px 14px;border:1px solid var(--line);border-radius:10px;background:transparent;color:inherit}
    button{margin-top:12px;width:100%;padding:12px 14px;border:0;border-radius:10px;background:var(--acc);color:#fff;font-weight:700;cursor:pointer}
    .hint{margin-top:12px;font-size:12px;color:var(--muted)}
  </style>
</head>
<body>
  <form class="card" id="loginForm">
    <h1>Roode Calibration Portal</h1>
    <p>Enter the portal password to open calibration tools and live zone diagnostics.</p>
    <label for="token">Portal password</label>
    <input id="token" name="token" type="password" autocomplete="current-password" autofocus />
    <button type="submit">Open Portal</button>
    <div class="hint">The password stays on the URL so the portal can authenticate its API requests.</div>
  </form>
  <script>
    document.getElementById('loginForm').addEventListener('submit', function (event) {
      event.preventDefault();
      const token = document.getElementById('token').value || '';
      location.href = '/portal?token=' + encodeURIComponent(token);
    });
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
    :root {
      --bg:#0b0f14; --panel:#121821; --text:#e6edf3; --muted:#9fb0c3; --acc:#57a6ff;
      --ok:#5bd19b; --warn:#ffcc66; --err:#ff6b6b; --line:#1f2835;
    }
    @media (prefers-color-scheme: light) {
      :root {
        --bg:#f6f8fa; --panel:#ffffff; --text:#24292f; --muted:#57606a; --acc:#0969da;
        --ok:#1a7f37; --warn:#9a6700; --err:#cf222e; --line:#d0d7de;
      }
    }
    *{box-sizing:border-box}
    body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,Inter,sans-serif}
    .wrap{max-width:1000px;margin:24px auto;padding:0 16px}
    .hdr{display:flex;gap:16px;align-items:center;justify-content:space-between;margin-bottom:12px}
    .hdr h1{font-size:18px;margin:0;font-weight:600}
    .status{font-size:12px;color:var(--muted)}
    .banner{display:none;background:var(--err);color:#fff;padding:8px 12px;border-radius:8px;margin-bottom:12px}
    button[disabled],.disabled{opacity:0.5;pointer-events:none;cursor:not-allowed}
    .row{display:flex;gap:16px;flex-wrap:wrap}
    .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:16px;flex:1 1 300px;min-width:300px}
    .card h2{margin:0 0 10px 0;font-size:14px;font-weight:600;color:var(--acc)}
    .btns{display:flex;gap:8px;flex-wrap:wrap}
    button,.btn{appearance:none;border:1px solid var(--line);background:#0f1520;color:var(--text);padding:8px 12px;border-radius:10px;font-weight:600;cursor:pointer}
    @media (prefers-color-scheme: light){ button,.btn{background:#f6f8fa;color:var(--text)} }
    button:hover,.btn:hover{border-color:#39475e}
    .btn.primary{background:var(--acc);color:#fff;border-color:#3e7cc8}
    .btn.ghost{background:transparent}
    .kv{display:grid;grid-template-columns:160px 1fr;gap:6px 12px}
    .kv .k{color:var(--muted)}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12px}
    .progress{height:10px;border-radius:8px;background:#0f1520;border:1px solid var(--line);overflow:hidden}
    @media (prefers-color-scheme: light){.progress{background:#e6edf3}}
    .progress>div{height:100%;background:linear-gradient(90deg,#4cc2ff,#57a6ff);width:0%}
    table{width:100%;border-collapse:collapse}
    th,td{padding:8px 6px;border-bottom:1px solid var(--line);text-align:left}
    th{color:var(--acc);font-weight:600}
    .chips{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}
    .right{margin-left:auto}
    .hr{height:1px;background:var(--line);margin:12px 0}
    .muted{color:var(--muted)}
    .tabs{display:flex;gap:4px;margin-bottom:16px;border-bottom:1px solid var(--line);padding-bottom:1px}
    .tab{padding:8px 16px;cursor:pointer;border-radius:8px 8px 0 0;border:1px solid transparent;margin-bottom:-1px}
    .tab.active{background:var(--panel);border-color:var(--line);border-bottom-color:var(--panel);font-weight:600;color:var(--acc)}
    .tab-content{display:none}
    .tab-content.active{display:block}
    .form-row{margin-bottom:12px}
    .form-row label{display:block;margin-bottom:4px;color:var(--muted);font-size:12px}
    .form-row input, .form-row select{width:100%;padding:8px;border:1px solid var(--line);border-radius:8px;background:transparent;color:inherit}
    .chk-row{display:flex;align-items:center;gap:8px;margin-bottom:8px;cursor:pointer}
    .chk-row input{width:18px;height:18px;cursor:pointer}
  </style>
</head>
<body>
  <div class="wrap">
    <div id="errorBanner" class="banner"></div>
    <div class="hdr">
      <h1>Roode Portal</h1>
      <div class="status"><span id="statusText">Status: <b>Idle</b></span> · <span id="lastCal">Last calibration: —</span></div>
    </div>

    <div class="tabs">
      <div class="tab active" data-tab="calibration">Calibration</div>
      <div class="tab" data-tab="settings">Settings</div>
      <div class="tab" data-tab="entities">Entities</div>
      <div class="tab" data-tab="about">About</div>
    </div>

    <div id="calibration" class="tab-content active">
      <div class="row">
        <div class="card" style="flex:1 1 100%">
          <div class="btns">
            <button class="btn primary" id="btnStart">Start Auto Calibration</button>
            <button class="btn" id="btnCancel" style="display:none">Cancel Scan</button>
          </div>
          <div class="hr"></div>
          <div class="kv">
            <div class="k">Active Session</div><div id="activeSession">—</div>
            <div class="k">Step</div><div id="stepText">—</div>
            <div class="k">Progress</div>
            <div><div class="progress"><div id="progressBar" style="width:0%"></div></div></div>
          </div>
        </div>

        <div class="card">
          <h2>Current ROI & Thresh</h2>
          <div class="kv" id="currentSettings">
            <div class="k">ROI Size</div><div id="curRoiSize">—</div>
            <div class="k">ROI Center</div><div id="curRoiCenter">—</div>
            <div class="k">Entry Center</div><div id="curEntry">—</div>
            <div class="k">Exit Center</div><div id="curExit">—</div>
            <div class="k">Min Threshold</div><div id="curMin">—</div>
            <div class="k">Max Threshold</div><div id="curMax">—</div>
          </div>
          <div class="hr"></div>
          <div class="kv">
            <div class="k">Live Distance</div><div id="curLive">—</div>
            <div class="k">Zone State</div><div id="curZones">—</div>
            <div class="k">People Counter</div><div id="curCount">—</div>
          </div>
        </div>

        <div class="card" style="flex:1 1 380px">
          <h2>Recommended Settings</h2>
          <div class="kv" id="preview">
            <div class="k">ROI Size</div><div id="pvRoiSize">—</div>
            <div class="k">ROI Center</div><div id="pvRoi">—</div>
            <div class="k">Entry Center</div><div id="pvEntry">—</div>
            <div class="k">Exit Center</div><div id="pvExit">—</div>
            <div class="k">Min Threshold</div><div id="pvMin">—</div>
            <div class="k">Max Threshold</div><div id="pvMax">—</div>
          </div>
          <div class="chips">
            <button class="btn primary" id="btnApply" disabled>Apply & Save Recommended</button>
          </div>
        </div>
      </div>
    </div>

    <div id="settings" class="tab-content">
      <div class="row">
        <div class="card">
          <h2>Sensor Control</h2>
          <div class="btns">
            <button class="btn" id="btnToggleSensor">Toggle Sensor: <span id="sensorStatus">ON</span></button>
            <button class="btn" id="btnRestartSensor">Restart Sensor</button>
            <button class="btn" id="btnRecalibrate">Simple Recalibrate</button>
          </div>
          <div class="hr"></div>
          <form id="settingsForm">
            <div class="form-row">
              <label>Sampling Size</label>
              <input type="number" name="sampling" min="1" max="10" />
            </div>
            <div class="form-row">
              <label>Polling Interval (ms)</label>
              <input type="number" name="polling_interval" min="5" max="100" />
            </div>
            <div class="form-row">
              <label>Invert Direction</label>
              <select name="invert_direction">
                <option value="false">Normal</option>
                <option value="true">Inverted</option>
              </select>
            </div>
            <div class="form-row">
              <label>Filter Mode</label>
              <select name="filter_mode">
                <option value="0">Min</option>
                <option value="1">Median</option>
                <option value="2">Percentile 10</option>
              </select>
            </div>
            <div class="form-row">
              <label>Filter Window</label>
              <input type="number" name="filter_window" min="1" max="20" />
            </div>
            <button type="submit" class="btn primary">Save Detailed Settings</button>
          </form>
        </div>

        <div class="card">
          <h2>Manual Override</h2>
          <div class="form-row">
            <label>ROI Width (4-16)</label>
            <input type="number" id="manual_roi_w" min="4" max="16" />
          </div>
          <div class="form-row">
            <label>ROI Height (4-16)</label>
            <input type="number" id="manual_roi_h" min="4" max="16" />
          </div>
          <div class="form-row">
            <label>ROI Center (0-255)</label>
            <input type="number" id="manual_roi_c" min="0" max="255" />
          </div>
          <div class="btns">
            <button class="btn" id="btnApplyManualROI">Apply ROI</button>
          </div>
        </div>
      </div>
    </div>

    <div id="entities" class="tab-content">
      <div class="card">
        <h2>Active Sensors / Entities</h2>
        <p class="muted">Disable entities to reduce CPU usage and I2C traffic.</p>
        <div id="sensorList">
          <label class="chk-row"><input type="checkbox" data-bit="0" checked> Distance Entry (Zone 0)</label>
          <label class="chk-row"><input type="checkbox" data-bit="1" checked> Distance Exit (Zone 1)</label>
          <label class="chk-row"><input type="checkbox" data-bit="2" checked> Presence Binary Sensor</label>
          <label class="chk-row"><input type="checkbox" data-bit="3" checked> People Counter Number</label>
          <label class="chk-row"><input type="checkbox" data-bit="4" checked> CPU Usage & Metrics</label>
          <label class="chk-row"><input type="checkbox" data-bit="5" checked> ROI & Threshold Sensors</label>
          <label class="chk-row"><input type="checkbox" data-bit="6" checked> Version & Status Text</label>
        </div>
        <div class="hr"></div>
        <button class="btn primary" id="btnSaveEntities">Save Active Entities</button>
      </div>
    </div>

    <div id="about" class="tab-content">
      <div class="card">
        <h2>Roode Firmware</h2>
        <div class="kv">
          <div class="k">Version</div><div id="aboutVersion">—</div>
          <div class="k">Sensor Type</div><div>VL53L1X</div>
          <div class="k">Portal Status</div><div id="aboutPortal">Active</div>
        </div>
      </div>
    </div>

  </div>

  <script>
  const TOKEN = new URLSearchParams(location.search).get('token') || '';
  const TOKEN_HEADER = TOKEN ? {'X-Portal-Token': TOKEN} : {};
  const tokenParam = TOKEN ? `?token=${encodeURIComponent(TOKEN)}` : '';
  
  const $ = (s)=>document.querySelector(s);
  const showToast = (msg, ok=false)=>{
    const b = $('#errorBanner');
    b.textContent = msg || '';
    b.style.background = ok ? 'var(--ok)' : 'var(--err)';
    b.style.display = 'block';
    clearTimeout(b._hide);
    b._hide = setTimeout(()=>{ b.style.display='none'; }, 5000);
  };
  const showError = (msg)=>showToast(msg || 'Request failed', false);
  const showSuccess = (msg)=>showToast(msg || 'Success', true);

  document.querySelectorAll('.tab').forEach(t => {
    t.addEventListener('click', () => {
      document.querySelectorAll('.tab, .tab-content').forEach(el => el.classList.remove('active'));
      t.classList.add('active');
      $('#' + t.dataset.tab).classList.add('active');
    });
  });

  async function getJSON(url){
    try {
      const r = await fetch(url, {cache:'no-store', headers: TOKEN_HEADER});
      if(!r.ok) throw new Error(r.status + ' ' + r.statusText);
      return await r.json();
    } catch(e) { return null; }
  }
  async function postJSON(url, body){
    try {
      const headers = {'Content-Type':'application/json'};
      if(TOKEN) headers['X-Portal-Token'] = TOKEN;
      const r = await fetch(url, {method:'POST', headers, body: body?JSON.stringify(body):'{}'});
      return await r.json().catch(()=>({ok:true}));
    } catch(e) { return { ok:false }; }
  }

  async function poll(){
    const [cur, stat, prev] = await Promise.all([
      getJSON('/api/settings/current'),
      getJSON('/api/scan/status'),
      getJSON('/api/roi/preview')
    ]);
    if(cur){
      $('#curRoiSize').textContent = `${cur.roi_width} × ${cur.roi_height}`;
      $('#curRoiCenter').textContent = cur.roi_center;
      $('#curEntry').textContent = cur.entry_center;
      $('#curExit').textContent = cur.exit_center;
      $('#curMin').textContent = cur.min_threshold + '%';
      $('#curMax').textContent = cur.max_threshold + '%';
      $('#curLive').textContent = `${cur.distance_entry_mm} / ${cur.distance_exit_mm} mm`;
      $('#curZones').textContent = `${cur.entry_active ? 'Entry ON' : 'Entry OFF'} · ${cur.exit_active ? 'Exit ON' : 'Exit OFF'}`;
      $('#curCount').textContent = cur.people_counter;
      $('#aboutVersion').textContent = cur.firmware;
      $('#sensorStatus').textContent = cur.sensor_enabled ? 'ON' : 'OFF';
      $('#btnToggleSensor').style.borderColor = cur.sensor_enabled ? 'var(--ok)' : 'var(--err)';
      if(cur.last_calibration) $('#lastCal').textContent = 'Last: ' + new Date(cur.last_calibration * 1000).toLocaleString();
      
      const form = $('#settingsForm');
      if(!form.dataset.dirty){
        form.sampling.value = cur.sampling;
        form.polling_interval.value = cur.polling_interval;
        form.invert_direction.value = cur.invert_direction ? 'true' : 'false';
        form.filter_mode.value = cur.filter_mode;
        form.filter_window.value = cur.filter_window;
      }
      
      if(!$('#sensorList').dataset.loaded){
        const mask = cur.active_sensors || 0xFFFFFFFF;
        document.querySelectorAll('#sensorList input').forEach(i => {
          i.checked = (mask & (1 << parseInt(i.dataset.bit))) !== 0;
        });
        $('#sensorList').dataset.loaded = 'true';
      }
    }
    if(stat){
      $('#statusText').innerHTML = 'Status: <b>' + stat.state + '</b>';
      $('#activeSession').textContent = stat.id || '—';
      $('#stepText').textContent = stat.step || '—';
      $('#progressBar').style.width = (stat.progress || 0) + '%';
      $('#btnCancel').style.display = (stat.state === 'Scanning') ? 'inline-block' : 'none';
      $('#btnStart').disabled = (stat.state === 'Scanning');
    }
    if(prev){
      $('#pvRoiSize').textContent = `${prev.roi_width} × ${prev.roi_height}`;
      $('#pvRoi').textContent = prev.roi_center;
      $('#pvEntry').textContent = prev.entry_center;
      $('#pvExit').textContent = prev.exit_center;
      $('#pvMin').textContent = prev.min_threshold + '%';
      $('#pvMax').textContent = prev.max_threshold + '%';
      $('#btnApply').disabled = false;
    } else {
      $('#btnApply').disabled = true;
    }
  }

  $('#btnApplyManualROI').onclick = () => {
    const params = new URLSearchParams();
    params.append('width', $('#manual_roi_w').value);
    params.append('height', $('#manual_roi_h').value);
    params.append('center', $('#manual_roi_c').value);
    if(TOKEN) params.append('token', TOKEN);
    fetch('/api/roi/manual?' + params.toString(), {method:'POST', headers: TOKEN_HEADER})
      .then(r => r.ok ? showSuccess('Manual ROI applied') : showError());
  };
  $('#btnStart').onclick = () => postJSON('/api/scan/start');
  $('#btnCancel').onclick = () => postJSON('/api/scan/cancel');
  $('#btnApply').onclick = () => postJSON('/api/roi/apply').then(r => r.ok ? showSuccess('Applied') : showError());
  $('#btnToggleSensor').onclick = () => postJSON('/api/sensor/toggle');
  $('#btnRestartSensor').onclick = () => postJSON('/api/sensor/restart');
  $('#btnRecalibrate').onclick = () => postJSON('/api/sensor/recalibrate');
  
  $('#settingsForm').onsubmit = (e) => {
    e.preventDefault();
    const params = new URLSearchParams();
    params.append('sampling', e.target.sampling.value);
    params.append('polling_interval', e.target.polling_interval.value);
    params.append('invert_direction', e.target.invert_direction.value);
    params.append('filter_mode', e.target.filter_mode.value);
    params.append('filter_window', e.target.filter_window.value);
    if(TOKEN) params.append('token', TOKEN);
    fetch('/api/settings/update?' + params.toString(), {method:'POST', headers: TOKEN_HEADER})
      .then(r => r.ok ? showSuccess('Settings saved') : showError());
  };
  
  $('#btnSaveEntities').onclick = () => {
    let mask = 0;
    document.querySelectorAll('#sensorList input').forEach(i => {
      if(i.checked) mask |= (1 << parseInt(i.dataset.bit));
    });
    const params = new URLSearchParams();
    params.append('active_sensors', mask);
    if(TOKEN) params.append('token', TOKEN);
    fetch('/api/settings/update?' + params.toString(), {method:'POST', headers: TOKEN_HEADER})
      .then(r => r.ok ? showSuccess('Entities updated') : showError());
  };

  setInterval(poll, 2000);
  poll();
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
  if (!log_fallback_events_) {
    if (msg == "interrupt_fallback" || msg == "interrupt_fallback_polling") return;
    if (msg == "int_pin_missed" || msg.rfind("int_pin_missed_sensor_", 0) == 0) return;
  }
  ESP_LOGI(TAG, "%s", msg.c_str());
}

void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Orientation: %s", orientation_ == Parallel ? "Parallel" : "Perpendicular");
  ESP_LOGCONFIG(TAG, "  Invert Direction: %s", invert_direction_ ? "True" : "False");
  ESP_LOGCONFIG(TAG, "  Sampling Size: %d", samples);
}

Roode::~Roode() { instance_ = nullptr; }

void Roode::setup() {
  ESP_LOGI(SETUP, "Booting Roode %s", VERSION);
  if (version_sensor != nullptr) version_sensor->publish_state(VERSION);

  entry->set_filter_window(filter_window_);
  entry->set_filter_mode(filter_mode_);
  exit->set_filter_window(filter_window_);
  exit->set_filter_mode(filter_mode_);

  settings_prefs_ = global_preferences->make_preference<RoodeSettings>(0xB0);
  RoodeSettings settings;
  if (settings_prefs_.load(&settings)) {
    ESP_LOGI(TAG, "Loaded persisted settings");
    this->set_sampling_size(settings.sampling);
    this->polling_interval_ms_ = std::max((uint16_t)10, settings.polling_interval);
    this->set_invert_direction(settings.invert_direction);
    this->set_filter_mode(settings.filter_mode);
    this->set_filter_window(settings.filter_window);
    this->active_sensors_ = settings.active_sensors;
    entry->roi->width = settings.roi_width;
    entry->roi->height = settings.roi_height;
    entry->roi->center = settings.roi_center;
    exit->roi->width = settings.roi_width;
    exit->roi->height = settings.roi_height;
    exit->roi->center = settings.roi_center;
  }

  if (calibration_persistence_) {
    calibration_prefs_[0] = global_preferences->make_preference<CalibrationPrefs>(0xA0);
    calibration_prefs_[1] = global_preferences->make_preference<CalibrationPrefs>(0xA1);
    for (int i = 0; i < 2; i++) {
      if (calibration_prefs_[i].load(&calibration_data_[i])) {
        Zone *z = i == 0 ? entry : exit;
        z->threshold->idle = calibration_data_[i].baseline_mm;
        z->threshold->min = calibration_data_[i].threshold_min_mm;
        z->threshold->max = calibration_data_[i].threshold_max_mm;
      }
    }
  }

#ifdef CONFIG_IDF_TARGET_ESP32
  if (!force_single_core_) {
    xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
    if (sensor_task_handle_ != nullptr) use_sensor_task_ = true;
  }
#endif

  register_portal_routes_();
  update_status_text("ok");
}

#ifdef USE_WEBSERVER
bool Roode::portal_request_authorized_(web_server_idf::AsyncWebServerRequest *request) const {
  if (this->portal_password_.empty()) return true;
  auto header = request->get_header("X-Portal-Token");
  if (header.has_value() && header.value() == this->portal_password_) return true;
  auto *token = request->getParam("token");
  if (token != nullptr && token->value() == this->portal_password_) return true;
  return false;
}

void Roode::send_portal_login_(web_server_idf::AsyncWebServerRequest *request) const {
  request->send(200, "text/html", portal_login_html);
}

bool Roode::require_portal_auth_(web_server_idf::AsyncWebServerRequest *request, const char *content_type) const {
  if (!this->portal_enabled_) {
    request->send(403, "text/plain", "Portal disabled");
    return false;
  }
  if (this->portal_request_authorized_(request)) return true;
  if (content_type != nullptr && std::string(content_type) == "text/html") {
    this->send_portal_login_(request);
  } else {
    request->send(401, "application/json", "{\"error\":\"unauthorized\"}");
  }
  return false;
}

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

void Roode::register_portal_routes_() {
#ifdef USE_WEBSERVER
  auto *base = web_server_base::global_web_server_base;
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/portal", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (this->require_portal_auth_(request, "text/html")) request->send(200, "text/html", portal_html);
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/settings/current", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    JsonDocument doc;
    doc["roi_width"] = entry->roi->width; doc["roi_height"] = entry->roi->height; doc["roi_center"] = entry->roi->center;
    doc["entry_center"] = entry->roi->center; doc["exit_center"] = exit->roi->center;
    doc["min_threshold"] = entry->threshold->min_percentage.value_or(15);
    doc["max_threshold"] = entry->threshold->max_percentage.value_or(80);
    doc["sampling"] = samples; doc["polling_interval"] = polling_interval_ms_; doc["invert_direction"] = invert_direction_;
    doc["filter_mode"] = (int) filter_mode_; doc["filter_window"] = filter_window_;
    doc["firmware"] = VERSION; doc["last_calibration"] = last_calibration_ts_;
    doc["distance_entry_mm"] = entry->getDistance(); doc["distance_exit_mm"] = exit->getDistance();
    doc["entry_active"] = LeftPreviousStatus == SOMEONE; doc["exit_active"] = RightPreviousStatus == SOMEONE;
    doc["people_counter"] = people_counter != nullptr ? people_counter->state : 0.0f;
    doc["sensor_enabled"] = sensor_enabled_; doc["active_sensors"] = active_sensors_;
    std::string out; serializeJson(doc, out); request->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/sensor/toggle", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    this->sensor_enabled_ = !this->sensor_enabled_;
    request->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/settings/update", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    RoodeSettings s;
    s.sampling = request->hasParam("sampling") ? atoi(request->getParam("sampling")->value().c_str()) : samples;
    s.polling_interval = request->hasParam("polling_interval") ? atoi(request->getParam("polling_interval")->value().c_str()) : polling_interval_ms_;
    s.invert_direction = request->hasParam("invert_direction") ? (request->getParam("invert_direction")->value() == "true") : invert_direction_;
    s.filter_mode = request->hasParam("filter_mode") ? (FilterMode)atoi(request->getParam("filter_mode")->value().c_str()) : filter_mode_;
    s.filter_window = request->hasParam("filter_window") ? atoi(request->getParam("filter_window")->value().c_str()) : filter_window_;
    s.active_sensors = request->hasParam("active_sensors") ? strtoul(request->getParam("active_sensors")->value().c_str(), NULL, 10) : active_sensors_;
    s.roi_width = entry->roi->width; s.roi_height = entry->roi->height; s.roi_center = entry->roi->center;
    s.entry_center = entry->roi->center; s.exit_center = exit->roi->center;
    s.min_threshold = entry->threshold->min_percentage.value_or(15); s.max_threshold = entry->threshold->max_percentage.value_or(80);
    this->set_sampling_size(s.sampling); this->polling_interval_ms_ = std::max((uint16_t)10, s.polling_interval);
    this->set_invert_direction(s.invert_direction); this->set_filter_mode(s.filter_mode); this->set_filter_window(s.filter_window);
    this->active_sensors_ = s.active_sensors;
    settings_prefs_.save(&s); global_preferences->sync();
    request->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/roi/apply", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json") || !has_recommended_settings_) return;
    entry->roi->width = recommended_settings_.roi_width; entry->roi->height = recommended_settings_.roi_height;
    entry->roi->center = recommended_settings_.roi_center; exit->roi->width = recommended_settings_.roi_width;
    exit->roi->height = recommended_settings_.roi_height; exit->roi->center = recommended_settings_.roi_center;
    RoodeSettings s; s.roi_width = entry->roi->width; s.roi_height = entry->roi->height; s.roi_center = entry->roi->center;
    s.entry_center = entry->roi->center; s.exit_center = exit->roi->center;
    s.min_threshold = recommended_settings_.min_threshold; s.max_threshold = recommended_settings_.max_threshold;
    s.sampling = samples; s.polling_interval = polling_interval_ms_; s.invert_direction = invert_direction_;
    s.filter_mode = filter_mode_; s.filter_window = filter_window_; s.active_sensors = active_sensors_;
    settings_prefs_.save(&s); global_preferences->sync(); this->recalibration();
    request->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/roi/manual", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    if (request->hasParam("width")) entry->roi->width = exit->roi->width = atoi(request->getParam("width")->value().c_str());
    if (request->hasParam("height")) entry->roi->height = exit->roi->height = atoi(request->getParam("height")->value().c_str());
    if (request->hasParam("center")) entry->roi->center = exit->roi->center = atoi(request->getParam("center")->value().c_str());
    this->recalibration(); request->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/roi/preview", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json") || !has_recommended_settings_) { request->send(200, "application/json", "null"); return; }
    JsonDocument doc; doc["roi_width"] = recommended_settings_.roi_width; doc["roi_height"] = recommended_settings_.roi_height;
    doc["roi_center"] = recommended_settings_.roi_center; doc["min_threshold"] = recommended_settings_.min_threshold;
    doc["max_threshold"] = recommended_settings_.max_threshold;
    std::string out; serializeJson(doc, out); request->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/sensor/restart", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (this->require_portal_auth_(request, "application/json")) { this->restart_sensor(); request->send(200, "application/json", "{\"ok\":true}"); }
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/sensor/recalibrate", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (this->require_portal_auth_(request, "application/json")) { this->recalibration(); request->send(200, "application/json", "{\"ok\":true}"); }
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/status", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    JsonDocument doc; doc["state"] = scan_state_ == SCANNING ? "Scanning" : "Idle"; doc["id"] = active_scan_id_;
    doc["step"] = scan_step_; doc["progress"] = scan_progress_;
    std::string out; serializeJson(doc, out); request->send(200, "application/json", out.c_str());
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/start", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    active_scan_id_ = "scan_" + std::to_string(millis()); scan_state_ = SCANNING; scan_progress_ = 0; scan_step_ = "Initializing"; scan_start_ts_ = millis();
    request->send(200, "application/json", "{\"ok\":true}");
  }));
  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/cancel", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (this->require_portal_auth_(request, "application/json")) { scan_state_ = SCAN_CANCELLED; request->send(200, "text/plain", "Cancelled"); }
  }));
#endif
}

void Roode::update() {
  if (active_sensors_ & 0x01 && distance_entry != nullptr) distance_entry->publish_state(entry->getDistance());
  if (active_sensors_ & 0x02 && distance_exit != nullptr) distance_exit->publish_state(exit->getDistance());
}

VL53L1_Error Roode::read_and_track_zone_(Zone *zone, bool update_timestamp) {
  this->current_zone = zone;
#ifdef CONFIG_IDF_TARGET_ESP32
  i2c_lock();
#endif
  VL53L1_Error status = zone->readDistance(distanceSensor);
#ifdef CONFIG_IDF_TARGET_ESP32
  i2c_unlock();
#endif
  if (status == VL53L1_ERROR_NONE && update_timestamp) { last_loop_update_ts_ = millis(); invalid_read_count_ = 0; }
  else if (status != VL53L1_ERROR_NONE) invalid_read_count_++;
  path_tracking(zone); handle_sensor_status(); return status;
}

void Roode::loop() {
  if (use_sensor_task_) {
    if (presence_update_pending_ && active_sensors_ & 0x04) { presence_sensor->publish_state(presence_state_); presence_update_pending_ = false; }
    if (sensor_status_update_pending_) { status_sensor->publish_state(pending_sensor_status_); sensor_status_update_pending_ = false; }
    if (people_counter_update_pending_ && active_sensors_ & 0x08) { auto call = people_counter->make_call(); call.set_value(pending_people_counter_value_); call.perform(); people_counter_update_pending_ = false; }
    if (metrics_update_pending_ && active_sensors_ & 0x10) {
      if (loop_time_sensor) loop_time_sensor->publish_state(pending_loop_time_);
      if (cpu_usage_sensor) cpu_usage_sensor->publish_state(pending_cpu_usage_);
      metrics_update_pending_ = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Lower priority thread processing
  } else {
    unsigned long start = micros(); read_and_track_zone_(entry, true); read_and_track_zone_(exit, false);
    unsigned long end = micros(); loop_time_sum_ += (end - start); loop_count_++; update_metrics();
    if (auto_calibration_interval_sec_ > 0 && (millis() - last_calibration_millis_ >= auto_calibration_interval_sec_ * 1000)) recalibration();
    delay(polling_interval_ms_);
  }
}

bool Roode::handle_sensor_status() {
  if (distanceSensor->is_failed()) update_status_text("offline");
  else if (sensor_status == VL53L1_ERROR_NONE) update_status_text("ok");
  last_sensor_status = sensor_status; sensor_status = VL53L1_ERROR_NONE; return true;
}

void Roode::path_tracking(Zone *zone) {
  int CurrentZoneStatus = NOBODY;
  if (state_ != STATE_IDLE && millis() - state_started_ts > 3000) { state_ = STATE_IDLE; PathTrackFillingSize = 0; }
  if (zone->getMinDistance() < zone->threshold->max && zone->getMinDistance() > zone->threshold->min) CurrentZoneStatus = SOMEONE;
  
  if (zone->id == 0) { if (CurrentZoneStatus != LeftPreviousStatus) { LeftPreviousStatus = CurrentZoneStatus; } }
  else { if (CurrentZoneStatus != RightPreviousStatus) { RightPreviousStatus = CurrentZoneStatus; } }

  AllZonesCurrentStatus = (LeftPreviousStatus == SOMEONE ? 1 : 0) + (RightPreviousStatus == SOMEONE ? 2 : 0);
  if (state_ == STATE_IDLE && AllZonesCurrentStatus != 0) { state_ = STATE_ENTRY_ACTIVE; state_started_ts = millis(); }

  if (AllZonesCurrentStatus == 0) {
    if (PathTrackFillingSize >= 3) {
      if (PathTrack[PathTrackFillingSize-3] == 2 && PathTrack[PathTrackFillingSize-2] == 3 && PathTrack[PathTrackFillingSize-1] == 1) updateCounter(1);
      else if (PathTrack[PathTrackFillingSize-3] == 1 && PathTrack[PathTrackFillingSize-2] == 3 && PathTrack[PathTrackFillingSize-1] == 2) updateCounter(-1);
    }
    PathTrackFillingSize = 0; state_ = STATE_IDLE;
  } else {
    if (PathTrackFillingSize == 0 || AllZonesCurrentStatus != PathTrack[PathTrackFillingSize - 1]) {
      if (PathTrackFillingSize < 4) PathTrack[PathTrackFillingSize++] = AllZonesCurrentStatus;
    }
  }
}

void Roode::update_status_text(const std::string &status) {
  if (status == last_status_text_) return;
  if (status_text_sensor && active_sensors_ & 0x40) status_text_sensor->publish_state(status);
  last_status_text_ = status;
}

void Roode::recalibration() { run_zone_calibration(0); run_zone_calibration(1); }

void Roode::updateCounter(int delta) {
  if (!people_counter) return;
  float nv = people_counter->state + delta;
  if (use_sensor_task_) { pending_people_counter_value_ = nv; people_counter_update_pending_ = true; }
  else { auto c = people_counter->make_call(); c.set_value(nv); c.perform(); }
}

void Roode::run_zone_calibration(uint8_t zone_id) {
  Zone *z = zone_id == 0 ? entry : exit;
  z->reset_roi(zone_id == 0 ? (orientation_ == Parallel ? 167 : 195) : (orientation_ == Parallel ? 231 : 60));
  z->calibrateThreshold(distanceSensor, 50);
  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  distanceSensor->set_ranging_mode(determine_ranging_mode(entry->threshold->idle, exit->threshold->idle));
  calibration_data_[zone_id].baseline_mm = z->threshold->idle;
  calibration_data_[zone_id].threshold_min_mm = z->threshold->min;
  calibration_data_[zone_id].threshold_max_mm = z->threshold->max;
  last_calibration_millis_ = millis();
  if (calibration_persistence_) calibration_prefs_[zone_id].save(&calibration_data_[zone_id]);
}

void Roode::update_metrics() {
  uint32_t now = millis(); if (now - loop_window_start_ < 10000) return;
  float cpu = 0.0f, lt = 0.0f;
  if (loop_count_ > 0) { lt = (float) loop_time_sum_ / (float) loop_count_ / 1000.0f; cpu = (lt / (float) polling_interval_ms_) * 100.0f; }
  if (use_sensor_task_) { pending_loop_time_ = lt; pending_cpu_usage_ = cpu; metrics_update_pending_ = true; }
  loop_window_start_ = now; loop_time_sum_ = 0; loop_count_ = 0;
}

const RangingMode *Roode::determine_ranging_mode(uint16_t ae, uint16_t ax) {
  uint16_t m = std::max(ae, ax);
  if (m <= 1300) return esphome::vl53l1x::Ranging::Short;
  if (m <= 2000) return esphome::vl53l1x::Ranging::Medium;
  if (m <= 2700) return esphome::vl53l1x::Ranging::Long;
  return esphome::vl53l1x::Ranging::Longest;
}

void Roode::restart_sensor() { distanceSensor->restart(); }

void Roode::sensor_task(void *p) {
  auto *self = static_cast<Roode *>(p);
#ifdef CONFIG_IDF_TARGET_ESP32
  esp_task_wdt_add(nullptr);
#endif
  for (;;) {
#ifdef CONFIG_IDF_TARGET_ESP32
    esp_task_wdt_reset();
#endif
    if (!self->sensor_enabled_ || self->distanceSensor->is_failed()) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if (self->scan_state_ == SCANNING) {
      std::vector<uint16_t> rates, dists; rates.reserve(64); dists.reserve(64);
      for (uint8_t i = 0; i < 64; i++) {
        if (self->scan_state_ != SCANNING) break;
        ROI roi; roi.width = 4; roi.height = 4; roi.center = ((i/8)*2+1)*16 + ((i%8)*2+1);
        VL53L1_Error st; auto d = self->distanceSensor->read_distance(&roi, st); rates.push_back(self->distanceSensor->get_signal_rate().value_or(0));
        dists.push_back(d.value_or(0)); self->scan_progress_ = (i * 100) / 64; vTaskDelay(pdMS_TO_TICKS(20));
      }
      if (self->scan_state_ == SCANNING) {
        uint8_t bi = 0; uint16_t md = 0;
        for (uint8_t i = 0; i < 64; i++) { if (dists[i] > md && dists[i] < 4000) { md = dists[i]; bi = i; } }
        self->recommended_settings_.roi_width = 6; self->recommended_settings_.roi_height = 16;
        self->recommended_settings_.roi_center = ((bi/8)*2+1)*16 + ((bi%8)*2+1);
        self->recommended_settings_.min_threshold = 15; self->recommended_settings_.max_threshold = 85;
        self->has_recommended_settings_ = true; self->scan_state_ = SCAN_IDLE;
      }
      continue;
    }
    unsigned long start = micros(); self->read_and_track_zone_(self->entry, true); self->read_and_track_zone_(self->exit, false);
    unsigned long delta = micros() - start; self->loop_time_sum_ += delta; self->loop_count_++; self->update_metrics();
    vTaskDelay(pdMS_TO_TICKS(self->polling_interval_ms_));
  }
}
}
}
