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
      <div class="tab" data-tab="about">About</div>
    </div>

    <div id="calibration" class="tab-content active">
      <!-- Controls & live status -->
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

        <!-- Current Settings -->
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

        <!-- Recommended Settings -->
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

        <!-- Past Sessions -->
        <div class="card" style="flex:1 1 100%">
          <div style="display:flex;align-items:center;justify-content:space-between">
            <h2>Scan Sessions</h2>
          </div>
          <div style="overflow:auto">
            <table id="tblSessions">
              <thead>
                <tr>
                  <th>#</th><th>Date/Time</th><th>Session ID</th><th>Duration</th><th>Size</th><th>Actions</th>
                </tr>
              </thead>
              <tbody></tbody>
            </table>
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

  </div><!-- /wrap -->

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

  // Tabs
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
    const [cur, stat, list, prev] = await Promise.all([
      getJSON('/api/settings/current'),
      getJSON('/api/scan/status'),
      getJSON('/api/scan/sessions'),
      getJSON('/api/roi/preview')
    ]);
    if(cur){
      $('#curRoiSize').textContent = `${cur.roi_width} × ${cur.roi_height}`;
      $('#curRoiCenter').textContent = cur.roi_center_id || cur.roi_center;
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
      
      // Update settings form if not dirty
      const form = $('#settingsForm');
      if(!form.dataset.dirty){
        form.sampling.value = cur.sampling;
        form.polling_interval.value = cur.polling_interval;
        form.invert_direction.value = cur.invert_direction ? 'true' : 'false';
        form.filter_mode.value = cur.filter_mode;
        form.filter_window.value = cur.filter_window;
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
    if(list){
      const tbody = $('#tblSessions tbody');
      tbody.innerHTML = '';
      list.forEach((s, i) => {
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>${i+1}</td><td>${new Date(s.ts * 1000).toLocaleString()}</td><td>${s.id}</td><td>${s.duration_sec}s</td><td>${s.size_bytes}B</td><td><a href="#" data-view="${s.id}">View</a></td>`;
        tbody.appendChild(tr);
      });
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
  $('#settingsForm').oninput = (e) => e.target.form.dataset.dirty = 'true';

  setInterval(poll, 2000);
  poll();
  </script>
</body>
</html>
)PORTAL";

// When disabled, fallback diagnostics are omitted from the log to reduce noise.
bool Roode::log_fallback_events_ = false;
Roode *Roode::instance_ = nullptr;
#ifdef CONFIG_IDF_TARGET_ESP32
SemaphoreHandle_t Roode::i2c_mutex_ = xSemaphoreCreateRecursiveMutex();
#endif
void Roode::log_event(const std::string &msg) {
  if (!log_fallback_events_) {
    if (msg == "interrupt_fallback" || msg == "interrupt_fallback_polling")
      return;
    if (msg == "int_pin_missed" || msg.rfind("int_pin_missed_sensor_", 0) == 0)
      return;
    if (msg == "xshut_toggled" || msg == "xshut_toggled_on" || msg == "xshut_toggled_off" || msg == "xshut_pulse_off" ||
        msg == "xshut_reinitialize" || msg == "sensor.recovered_via_xshut" || msg.rfind("xshut_sensor_", 0) == 0 ||
        msg.rfind("xshut_pulse_off_sensor_", 0) == 0 || msg.rfind("xshut_reinitialize_sensor_", 0) == 0 ||
        (msg.rfind("sensor_", 0) == 0 && msg.find(".recovered_via_xshut") != std::string::npos))
      return;
  }

  static uint32_t last_int_log = 0;
  if (msg == "interrupt_fallback" || msg == "interrupt_fallback_polling" || msg == "int_pin_missed" ||
      msg.rfind("int_pin_missed_sensor_", 0) == 0) {
    uint32_t now = millis();
    if (last_int_log != 0 && (now - last_int_log) < 5000)
      return;
    last_int_log = now;
  }

  std::string out = msg;
  if (msg == "use_dual_core")
    out += " - launching task on core 1";
  else if (msg.rfind("retry_multicore_", 0) == 0)
    out += " - retry creating task";
  else if (msg == "dual_core_success")
    out += " - task running on core 1";
  else if (msg == "dual_core_failed")
    out += " - task creation failed";
  else if (msg == "fallback_single_core")
    out += " - switching to single core";
  else if (msg == "force_single_core")
    out += " - single core forced by config";
  else if (msg.rfind("xshut_sensor_", 0) == 0) {
    bool on = msg.find("_on") != std::string::npos;
    size_t start = sizeof("xshut_sensor_") - 1;
    size_t end = msg.find('_', start);
    std::string id = msg.substr(start, end - start);
    out += on ? " - sensor " + id + " ON" : " - sensor " + id + " OFF";
  } else if (msg == "xshut_toggled_on")
    out += " - XSHUT pin HIGH";
  else if (msg == "xshut_toggled_off")
    out += " - XSHUT pin LOW";
  else if (msg == "xshut_toggled")
    out += " - XSHUT pin toggled";
  else if (msg.rfind("xshut_pulse_off_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("xshut_pulse_off_sensor_") - 1);
    out += " - pulsing LOW for sensor " + id;
  } else if (msg == "xshut_pulse_off") {
    out += " - pulsing LOW";
  } else if (msg.rfind("xshut_reinitialize_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("xshut_reinitialize_sensor_") - 1);
    out += " - reinitializing sensor " + id;
  } else if (msg == "xshut_reinitialize")
    out += " - reinitializing all sensors";
  else if (msg == "sensor.recovered_via_xshut")
    out += " - sensor recovered via XSHUT pulse";
  else if (msg.rfind("sensor_", 0) == 0 && msg.find(".recovered_via_xshut") != std::string::npos) {
    size_t end = msg.find('.');
    std::string id = msg.substr(sizeof("sensor_") - 1, end - (sizeof("sensor_") - 1));
    out = "sensor " + id + " recovered via XSHUT pulse";
  }
  ESP_LOGI(TAG, "%s", out.c_str());
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
  if (version_sensor != nullptr) {
    version_sensor->publish_state(VERSION);
  }
  ESP_LOGI(SETUP, "Using sampling with sampling size: %d", samples);

  if (this->distanceSensor->is_failed()) {
    update_status_text("offline");
    ESP_LOGW(TAG, "Roode sensor is offline, but proceeding with portal registration.");
  }

  // Initialize filtering options before calibrating so threshold sampling uses
  // the configured window and mode
  entry->set_filter_window(filter_window_);
  entry->set_filter_mode(filter_mode_);
  exit->set_filter_window(filter_window_);
  exit->set_filter_mode(filter_mode_);

  settings_prefs_ = global_preferences->make_preference<RoodeSettings>(0xB0);
  RoodeSettings settings;
  if (settings_prefs_.load(&settings)) {
    ESP_LOGI(TAG, "Loaded persisted settings");
    this->set_sampling_size(settings.sampling);
    this->polling_interval_ms_ = settings.polling_interval;
    this->set_invert_direction(settings.invert_direction);
    this->set_filter_mode(settings.filter_mode);
    this->set_filter_window(settings.filter_window);
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
    bool loaded = true;
    for (int i = 0; i < 2; i++) {
      if (calibration_prefs_[i].load(&calibration_data_[i])) {
        Zone *z = i == 0 ? entry : exit;
        z->threshold->idle = calibration_data_[i].baseline_mm;
        z->threshold->min = calibration_data_[i].threshold_min_mm;
        z->threshold->max = calibration_data_[i].threshold_max_mm;
        int valid_count = 0;
        for (int s = 0; s < 5; s++) {
          z->readDistance(distanceSensor);
          if (abs((int) z->getDistance() - (int) z->threshold->idle) < (z->threshold->idle * 0.1))
            valid_count++;
        }
        if (valid_count < 5) {
          loaded = false;
          break;
        }
      } else {
        loaded = false;
        break;
      }
    }
    if (loaded) {
      entry->reset_roi(orientation_ == Parallel ? 167 : 195);
      exit->reset_roi(orientation_ == Parallel ? 231 : 60);
      entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
      exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
      auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
      distanceSensor->set_ranging_mode(mode);
      publish_sensor_configuration(entry, exit, true);
      publish_sensor_configuration(entry, exit, false);
    } else {
      calibrate_zones();
    }
  } else {
    calibrate_zones();
  }
  last_calibration_ts_ =
      std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
  last_calibration_millis_ = millis();
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!force_single_core_) {
    log_event("use_dual_core");
    vTaskDelay(pdMS_TO_TICKS(200));
    BaseType_t res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
    multicore_retry_count_ = 0;
    while (res != pdPASS && multicore_retry_count_ < 2) {
      multicore_retry_count_++;
      log_event(std::string("retry_multicore_") + std::to_string(multicore_retry_count_));
      vTaskDelay(pdMS_TO_TICKS(200));
      res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
    }
    if (res == pdPASS) {
      use_sensor_task_ = true;
      log_event("dual_core_success");
    } else {
      log_event("dual_core_failed");
      log_event("fallback_single_core");
      use_sensor_task_ = false;
    }
  } else {
    use_sensor_task_ = false;
    log_event("force_single_core");
  }
#else
  use_sensor_task_ = false;
#endif
  loop_window_start_ = millis();
  loop_time_sum_ = 0;
  loop_count_ = 0;
  if (status_sensor != nullptr)
    status_sensor->publish_state(sensor_status);

  if (loop_time_sensor != nullptr)
    loop_time_sensor->publish_state(0);
  if (cpu_usage_sensor != nullptr)
    cpu_usage_sensor->publish_state(0);
  if (ram_free_sensor != nullptr)
    ram_free_sensor->publish_state(0);
  if (flash_free_sensor != nullptr)
    flash_free_sensor->publish_state(0);
  manual_adjustment_count_ = 0;
  if (manual_adjustment_sensor != nullptr)
    manual_adjustment_sensor->publish_state(0);
  if (xshut_state_binary_sensor != nullptr) {
    auto val = distanceSensor->get_xshut_state();
    if (val.has_value())
      xshut_state_binary_sensor->publish_state(*val);
  }
  if (interrupt_status_sensor != nullptr) {
    auto val = distanceSensor->get_interrupt_state();
    if (val.has_value())
      interrupt_status_sensor->publish_state(*val ? 1 : 0);
  }
  if (people_counter != nullptr)
    expected_counter_ = people_counter->state;

  register_portal_routes_();
  publish_feature_list();
  update_status_text("ok");
}

#ifdef USE_WEBSERVER
bool Roode::portal_request_authorized_(web_server_idf::AsyncWebServerRequest *request) const {
  if (this->portal_password_.empty())
    return true;
  auto header = request->get_header("X-Portal-Token");
  if (header.has_value() && header.value() == this->portal_password_)
    return true;
  auto *token = request->getParam("token");
  if (token != nullptr && token->value() == this->portal_password_)
    return true;
  return false;
}

void Roode::send_portal_login_(web_server_idf::AsyncWebServerRequest *request) const {
  ESP_LOGD(TAG, "Portal auth required, serving login page");
  request->send(200, "text/html", portal_login_html);
}

bool Roode::require_portal_auth_(web_server_idf::AsyncWebServerRequest *request, const char *content_type) const {
  if (!this->portal_enabled_) {
    request->send(403, "text/plain", "Portal disabled");
    return false;
  }
  if (this->portal_request_authorized_(request))
    return true;

  ESP_LOGD(TAG, "Portal auth rejected for content-type=%s", content_type == nullptr ? "<null>" : content_type);
  if (content_type != nullptr && std::string(content_type) == "text/html") {
    this->send_portal_login_(request);
  } else if (content_type != nullptr && std::string(content_type) == "application/json") {
    request->send(409, content_type, "{\"error\":\"portal_auth_required\"}");
  } else {
    request->send(409, content_type == nullptr ? "text/plain" : content_type, "Unauthorized");
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
    if (request->method() != method_)
      return false;
    char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    std::string request_url(request->url_to(url_buf));
    bool match = match_type_ == PREFIX ? request_url.rfind(uri_, 0) == 0 : request_url == uri_;
    if (match) {
      ESP_LOGD(TAG, "Portal handler match: method=%d url=%s route=%s", (int) request->method(), request_url.c_str(),
               uri_.c_str());
    }
    return match;
  }

  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override {
    char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buf);
    ESP_LOGD(TAG, "Portal handler dispatch: method=%d url=%s", (int) request->method(), url.c_str());
    callback_(request);
  }

 private:
  http_method method_;
  std::string uri_;
  Callback callback_;
  MatchType match_type_;
};
}  // namespace
#endif

void Roode::register_portal_routes_() {
#ifdef USE_WEBSERVER
  if (this->portal_registered_)
    return;
  auto *base = web_server_base::global_web_server_base;
  auto *srv = base->get_server();
  if (srv == nullptr) {
    ESP_LOGW(TAG, "Web server base null; portal routes not registered");
    this->set_timeout("portal_retry", 250, [this]() { this->register_portal_routes_(); });
    return;
  }
  ESP_LOGI(TAG, "Registering portal routes on port %u", base->get_port());

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/portal", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "text/html"))
      return;
    ESP_LOGD(TAG, "Serving /portal");
    request->send(200, "text/html", portal_html);
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/portal/", [](web_server_idf::AsyncWebServerRequest *request) {
    std::string destination = "/portal";
    if (request->hasParam("token")) {
      destination += "?token=";
      destination += request->getParam("token")->value().c_str();
    }
    request->redirect(destination);
  }));

  base->add_handler_without_auth(
      new LambdaRequestHandler(HTTP_GET, "/api/settings/current", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    ESP_LOGD(TAG, "Serving /api/settings/current");
    JsonDocument doc;
    doc["roi_width"] = entry->roi->width;
    doc["roi_height"] = entry->roi->height;
    doc["roi_center"] = entry->roi->center;
    doc["entry_center"] = entry->roi->center;
    doc["exit_center"] = exit->roi->center;
    doc["min_threshold"] = entry->threshold->min_percentage.value_or(15);
    doc["max_threshold"] = entry->threshold->max_percentage.value_or(80);
    doc["sampling"] = samples;
    doc["polling_interval"] = polling_interval_ms_;
    doc["invert_direction"] = invert_direction_;
    doc["filter_mode"] = (int) filter_mode_;
    doc["filter_window"] = filter_window_;
    doc["firmware"] = VERSION;
    doc["last_calibration"] = last_calibration_ts_;
    doc["distance_entry_mm"] = entry->getDistance();
    doc["distance_exit_mm"] = exit->getDistance();
    doc["entry_active"] = LeftPreviousStatus == SOMEONE;
    doc["exit_active"] = RightPreviousStatus == SOMEONE;
    doc["people_counter"] = people_counter != nullptr ? people_counter->state : 0.0f;
    doc["sensor_enabled"] = sensor_enabled_;
    std::string output;
    serializeJson(doc, output);
    request->send(request->beginResponse(200, "application/json", output));
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/sensor/toggle", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    this->sensor_enabled_ = !this->sensor_enabled_;
    JsonDocument doc;
    doc["enabled"] = sensor_enabled_;
    std::string output;
    serializeJson(doc, output);
    request->send(request->beginResponse(200, "application/json", output));
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/settings/update", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    
    RoodeSettings settings;
    settings.sampling = request->hasParam("sampling") ? atoi(request->getParam("sampling")->value().c_str()) : samples;
    settings.polling_interval = request->hasParam("polling_interval") ? atoi(request->getParam("polling_interval")->value().c_str()) : polling_interval_ms_;
    settings.invert_direction = request->hasParam("invert_direction") ? (request->getParam("invert_direction")->value() == "true") : invert_direction_;
    settings.filter_mode = request->hasParam("filter_mode") ? (FilterMode)atoi(request->getParam("filter_mode")->value().c_str()) : filter_mode_;
    settings.filter_window = request->hasParam("filter_window") ? atoi(request->getParam("filter_window")->value().c_str()) : filter_window_;
    
    settings.roi_width = entry->roi->width;
    settings.roi_height = entry->roi->height;
    settings.roi_center = entry->roi->center;
    settings.entry_center = entry->roi->center;
    settings.exit_center = exit->roi->center;
    settings.min_threshold = entry->threshold->min_percentage.value_or(15);
    settings.max_threshold = entry->threshold->max_percentage.value_or(80);

    // Apply settings
    this->set_sampling_size(settings.sampling);
    this->polling_interval_ms_ = settings.polling_interval;
    this->set_invert_direction(settings.invert_direction);
    this->set_filter_mode(settings.filter_mode);
    this->set_filter_window(settings.filter_window);
    
    // Persist
    settings_prefs_.save(&settings);
    global_preferences->sync();

    request->send(200, "application/json", "{\"ok\":true}");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/sensor/restart", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    this->restart_sensor();
    request->send(200, "application/json", "{\"ok\":true}");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/sensor/recalibrate", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    this->recalibration();
    request->send(200, "application/json", "{\"ok\":true}");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/status", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    JsonDocument doc;
    doc["state"] = scan_state_ == SCANNING ? "Scanning" : "Idle";
    doc["id"] = active_scan_id_;
    doc["step"] = scan_step_;
    doc["progress"] = scan_progress_;
    std::string output;
    serializeJson(doc, output);
    request->send(request->beginResponse(200, "application/json", output));
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/start", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    if (scan_state_ == SCANNING) {
      request->send(409, "text/plain", "Scan already in progress");
      return;
    }
    active_scan_id_ = "scan_" + std::to_string(millis());
    scan_state_ = SCANNING;
    scan_progress_ = 0;
    scan_step_ = "Initializing";
    scan_start_ts_ = millis();
    JsonDocument doc;
    doc["id"] = active_scan_id_;
    std::string output;
    serializeJson(doc, output);
    request->send(request->beginResponse(200, "application/json", output));
  }));

  base->add_handler_without_auth(
      new LambdaRequestHandler(HTTP_POST, "/api/scan/cancel", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    if (scan_state_ == SCANNING) {
      scan_state_ = SCAN_CANCELLED;
    }
    request->send(200, "text/plain", "Cancelled");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/roi/apply", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    if (!has_recommended_settings_) {
      request->send(400, "application/json", "{\"ok\":false, \"msg\":\"No recommended settings available\"}");
      return;
    }
    // Apply recommended settings
    entry->roi->width = recommended_settings_.roi_width;
    entry->roi->height = recommended_settings_.roi_height;
    entry->roi->center = recommended_settings_.roi_center;
    exit->roi->width = recommended_settings_.roi_width;
    exit->roi->height = recommended_settings_.roi_height;
    exit->roi->center = recommended_settings_.roi_center;
    
    // Persist
    RoodeSettings settings;
    settings.roi_width = recommended_settings_.roi_width;
    settings.roi_height = recommended_settings_.roi_height;
    settings.roi_center = recommended_settings_.roi_center;
    settings.entry_center = recommended_settings_.entry_center;
    settings.exit_center = recommended_settings_.exit_center;
    settings.min_threshold = recommended_settings_.min_threshold;
    settings.max_threshold = recommended_settings_.max_threshold;
    settings.sampling = samples;
    settings.polling_interval = polling_interval_ms_;
    settings.invert_direction = invert_direction_;
    settings.filter_mode = filter_mode_;
    settings.filter_window = filter_window_;
    settings_prefs_.save(&settings);
    global_preferences->sync();

    // Trigger simple recalibration to get baselines
    this->recalibration();
    request->send(200, "application/json", "{\"ok\":true}");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/roi/manual", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    if (request->hasParam("width")) {
      uint8_t w = atoi(request->getParam("width")->value().c_str());
      entry->roi->width = w; exit->roi->width = w;
    }
    if (request->hasParam("height")) {
      uint8_t h = atoi(request->getParam("height")->value().c_str());
      entry->roi->height = h; exit->roi->height = h;
    }
    if (request->hasParam("center")) {
      uint8_t c = atoi(request->getParam("center")->value().c_str());
      entry->roi->center = c; exit->roi->center = c;
    }
    this->recalibration();
    request->send(200, "application/json", "{\"ok\":true}");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/roi/preview", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json")) return;
    if (!has_recommended_settings_) {
      request->send(200, "application/json", "null");
      return;
    }
    JsonDocument doc;
    doc["roi_width"] = recommended_settings_.roi_width;
    doc["roi_height"] = recommended_settings_.roi_height;
    doc["roi_center"] = recommended_settings_.roi_center;
    doc["entry_center"] = recommended_settings_.entry_center;
    doc["exit_center"] = recommended_settings_.exit_center;
    doc["min_threshold"] = recommended_settings_.min_threshold;
    doc["max_threshold"] = recommended_settings_.max_threshold;
    std::string output;
    serializeJson(doc, output);
    request->send(request->beginResponse(200, "application/json", output));
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_POST, "/api/scan/delete", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    // Dummy delete
    request->send(200, "application/json", "{\"ok\":true}");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/export/all", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    request->send(200, "application/json", "[]");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/sessions", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (const auto &s : sessions_) {
      JsonObject obj = arr.add<JsonObject>();
      obj["id"] = s.id;
      obj["ts"] = s.ts;
      obj["trials"] = s.trials;
      obj["duration_sec"] = s.duration_sec;
      obj["size_bytes"] = s.size_bytes;
    }
    std::string output;
    serializeJson(doc, output);
    request->send(request->beginResponse(200, "application/json", output));
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(
      HTTP_GET, "/api/scan/session/", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    char url_buf[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    std::string uri = request->url_to(url_buf);
    std::string id = uri.substr(uri.find_last_of('/') + 1);
    if (this->scan_results_.count(id)) {
      JsonDocument doc;
      doc["type"] = "passive_scan";
      doc["grid"] = 8;
      doc["ranging"] = "auto";
      JsonArray data = doc["data"].to<JsonObject>()["mcps"].to<JsonArray>();
      for (uint16_t val : this->scan_results_[id]) {
        data.add(val);
      }
      std::string output;
      serializeJson(doc, output);
      request->send(request->beginResponse(200, "application/json", output));
    } else {
      request->send(404, "text/plain", "Session not found");
    }
  },
      LambdaRequestHandler::PREFIX));

  this->portal_registered_ = true;
  ESP_LOGI(TAG, "Portal routes registered");
#endif
}

void Roode::update() {
  if (distance_entry != nullptr) {
    distance_entry->publish_state(entry->getDistance());
  }
  if (distance_exit != nullptr) {
    distance_exit->publish_state(exit->getDistance());
  }
  if (xshut_state_binary_sensor != nullptr) {
    auto val = distanceSensor->get_xshut_state();
    if (val.has_value())
      xshut_state_binary_sensor->publish_state(*val);
  }
  if (interrupt_status_sensor != nullptr) {
    auto val = distanceSensor->get_interrupt_state();
    if (val.has_value())
      interrupt_status_sensor->publish_state(*val ? 1 : 0);
  }
  if (people_counter != nullptr && fabs(people_counter->state - expected_counter_) > 0.001f) {
    int diff = (int) roundf(people_counter->state - expected_counter_);
    manual_adjustment_count_ += abs(diff);
    expected_counter_ = people_counter->state;
    if (manual_adjustment_sensor != nullptr)
      manual_adjustment_sensor->publish_state(manual_adjustment_count_);
    if (diff != 0) {
      std::string sign = diff > 0 ? "+" : "";
      log_event("manual_adjust " + sign + std::to_string(diff) + " total=" + std::to_string(manual_adjustment_count_));
    }
  }
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
  if (status == VL53L1_ERROR_NONE && update_timestamp) {
    last_loop_update_ts_ = millis();
    invalid_read_count_ = 0;
  } else if (status != VL53L1_ERROR_NONE) {
    invalid_read_count_++;
  }
  path_tracking(zone);
  handle_sensor_status();
  return status;
}

void Roode::loop() {
  if (use_sensor_task_) {
    // When running on dual core the sensor loop runs in a separate task
    // Skip execution from main loop
    uint32_t now = millis();
    if (presence_update_pending_) {
      presence_sensor->publish_state(presence_state_);
      presence_update_pending_ = false;
    }
    if (entry_presence_update_pending_) {
      entry_presence_sensor->publish_state(entry_presence_state_);
      entry_presence_update_pending_ = false;
    }
    if (exit_presence_update_pending_) {
      exit_presence_sensor->publish_state(exit_presence_state_);
      exit_presence_update_pending_ = false;
    }
    if (sensor_status_update_pending_) {
      status_sensor->publish_state(pending_sensor_status_);
      sensor_status_update_pending_ = false;
    }
    if (status_text_update_pending_) {
      status_text_sensor->publish_state((char *) pending_status_text_);
      last_status_text_ = (char *) pending_status_text_;
      status_text_update_pending_ = false;
    }
    if (feature_list_update_pending_) {
      publish_feature_list();
      feature_list_update_pending_ = false;
    }
    if (entry_exit_event_pending_) {
      entry_exit_event_sensor->publish_state((char *) pending_entry_exit_event_);
      entry_exit_event_pending_ = false;
    }
    if (people_counter_update_pending_) {
      auto call = this->people_counter->make_call();
      call.set_value(pending_people_counter_value_);
      call.perform();
      people_counter_update_pending_ = false;
    }
    if (manual_adjustment_update_pending_) {
      manual_adjustment_sensor->publish_state(pending_manual_adjustment_count_);
      manual_adjustment_update_pending_ = false;
    }
    if (metrics_update_pending_) {
      if (loop_time_sensor != nullptr)
        loop_time_sensor->publish_state(pending_loop_time_);
      if (cpu_usage_sensor != nullptr)
        cpu_usage_sensor->publish_state(pending_cpu_usage_);
      if (ram_free_sensor != nullptr)
        ram_free_sensor->publish_state(pending_ram_free_);
      if (flash_free_sensor != nullptr)
        flash_free_sensor->publish_state(pending_flash_free_);
      metrics_update_pending_ = false;
    }
    if (config_update_pending_) {
      publish_sensor_configuration(entry, exit, true);
      publish_sensor_configuration(entry, exit, false);
      config_update_pending_ = false;
    }
    for (int i = 0; i < 2; i++) {
      if (calibration_save_pending_[i]) {
        calibration_prefs_[i].save(&calibration_data_[i]);
        calibration_save_pending_[i] = false;
      }
    }
  } else {
    // Single core mode
    unsigned long start = micros();
    AnEventHasOccured = 0;
    AllZonesCurrentStatus = 0;
    read_and_track_zone_(entry, true);
    read_and_track_zone_(exit, false);
    unsigned long end = micros();
    unsigned long delta = end - start;
    loop_time_sum_ += delta;
    loop_count_++;
    update_metrics();
    uint32_t now = millis();
    if (auto_calibration_interval_sec_ > 0 &&
        (now - last_calibration_millis_ >= auto_calibration_interval_sec_ * 1000)) {
      run_zone_calibration(0);
      run_zone_calibration(1);
    }
  }
}

bool Roode::handle_sensor_status() {
  bool check_status = false;
  std::string text_state;
  if (distanceSensor->is_failed()) {
    text_state = "offline";
  } else if (sensor_status == VL53L1_ERROR_NONE) {
    text_state = "ok";
    if (last_sensor_status != sensor_status) {
      if (status_sensor != nullptr) {
        if (use_sensor_task_) {
          pending_sensor_status_ = sensor_status;
          sensor_status_update_pending_ = true;
        } else {
          status_sensor->publish_state(sensor_status);
        }
      }
      check_status = true;
    }
  } else if (sensor_status == VL53L1_ERROR_TIME_OUT) {
    text_state = "timeout";
    if (status_sensor != nullptr) {
      if (use_sensor_task_) {
        pending_sensor_status_ = sensor_status;
        sensor_status_update_pending_ = true;
      } else {
        status_sensor->publish_state(sensor_status);
      }
    }
  } else {
    text_state = "error";
    if (status_sensor != nullptr) {
      if (use_sensor_task_) {
        pending_sensor_status_ = sensor_status;
        sensor_status_update_pending_ = true;
      } else {
        status_sensor->publish_state(sensor_status);
      }
    }
  }

  update_status_text(text_state);
  last_sensor_status = sensor_status;
  sensor_status = VL53L1_ERROR_NONE;
  return check_status;
}

void Roode::path_tracking(Zone *zone) {
  int CurrentZoneStatus = NOBODY;

  uint32_t timeout = state_ == STATE_ENTRY_ACTIVE ? 2500 : 3500;
  if (state_ != STATE_IDLE && millis() - state_started_ts > timeout) {
    state_ = STATE_IDLE;
    PathTrackFillingSize = 0;
    PathTrack[0] = 0;
    ESP_LOGW(TAG, "fsm_timeout_reset");
  }

  ESP_LOGV(TAG, "Zone %d distance %u (min=%u max=%u)", zone->id, zone->getMinDistance(), zone->threshold->min,
           zone->threshold->max);

  // PathTrack algorithm
  if (zone->getMinDistance() < zone->threshold->max && zone->getMinDistance() > zone->threshold->min) {
    // Someone is in the sensing area
    CurrentZoneStatus = SOMEONE;
    if (presence_sensor != nullptr) {
      if (use_sensor_task_) {
        presence_state_ = true;
        presence_update_pending_ = true;
      } else {
        presence_sensor->publish_state(true);
      }
    }
    // Expose occupancy for the specific zone when configured
    if (zone->id == 0 && entry_presence_sensor != nullptr) {
      if (use_sensor_task_) {
        entry_presence_state_ = true;
        entry_presence_update_pending_ = true;
      } else {
        entry_presence_sensor->publish_state(true);
      }
    }
    if (zone->id == 1 && exit_presence_sensor != nullptr) {
      if (use_sensor_task_) {
        exit_presence_state_ = true;
        exit_presence_update_pending_ = true;
      } else {
        exit_presence_sensor->publish_state(true);
      }
    }
    if (zone_triggered_start_[zone->id] == 0) {
      zone_triggered_start_[zone->id] = millis();
    }
  }
  if (CurrentZoneStatus == NOBODY) {
    // Clear zone-specific occupancy sensors once motion has left the area
    if (zone->id == 0 && entry_presence_sensor != nullptr) {
      if (use_sensor_task_) {
        entry_presence_state_ = false;
        entry_presence_update_pending_ = true;
      } else {
        entry_presence_sensor->publish_state(false);
      }
    }
    if (zone->id == 1 && exit_presence_sensor != nullptr) {
      if (use_sensor_task_) {
        exit_presence_state_ = false;
        exit_presence_update_pending_ = true;
      } else {
        exit_presence_sensor->publish_state(false);
      }
    }
    zone_triggered_start_[zone->id] = 0;
  } else if (zone_triggered_start_[zone->id] != 0 && millis() - zone_triggered_start_[zone->id] >= 10000 &&
             millis() - last_valid_crossing_ts_ >= 120000) {
    ESP_LOGI(CALIBRATION, "Fail safe calibration triggered for zone %d", zone->id);
    run_zone_calibration(zone->id);
    fail_safe_triggered_ = true;
    zone_triggered_start_[zone->id] = 0;
  }

  bool zone_changed = false;
  // left zone
  if (zone == (this->invert_direction_ ? this->exit : this->entry)) {
    if (CurrentZoneStatus != LeftPreviousStatus) {
      LeftPreviousStatus = CurrentZoneStatus;
      zone_changed = true;
    }
  }
  // right zone
  else {
    if (CurrentZoneStatus != RightPreviousStatus) {
      RightPreviousStatus = CurrentZoneStatus;
      zone_changed = true;
    }
  }

  // if an event has occured
  if (zone_changed) {
    AllZonesCurrentStatus = (LeftPreviousStatus == SOMEONE ? 1 : 0) + (RightPreviousStatus == SOMEONE ? 2 : 0);
    ESP_LOGD(TAG, "Zone changed, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);

    if (state_ == STATE_IDLE && AllZonesCurrentStatus != 0) {
      state_ = STATE_ENTRY_ACTIVE;
      state_started_ts = millis();
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if (AllZonesCurrentStatus == 0) {
      ESP_LOGD(TAG, "Nobody anywhere, sequence length: %d", PathTrackFillingSize);
      if (PathTrackFillingSize >= 3) {
        // Entry: [..., 2, 3, 1, 0]
        if (PathTrackFillingSize >= 3 && PathTrack[PathTrackFillingSize-3] == 2 && 
            PathTrack[PathTrackFillingSize-2] == 3 && PathTrack[PathTrackFillingSize-1] == 1) {
          ESP_LOGI("Roode pathTracking", "Entry detected.");
          this->updateCounter(1);
          last_valid_crossing_ts_ = millis();
          if (entry_exit_event_sensor != nullptr) {
            if (use_sensor_task_) {
              strncpy((char *) pending_entry_exit_event_, "Entry", sizeof(pending_entry_exit_event_) - 1);
              pending_entry_exit_event_[sizeof(pending_entry_exit_event_) - 1] = '\0';
              entry_exit_event_pending_ = true;
            } else {
              entry_exit_event_sensor->publish_state("Entry");
            }
          }
        } 
        // Exit: [..., 1, 3, 2, 0]
        else if (PathTrackFillingSize >= 3 && PathTrack[PathTrackFillingSize-3] == 1 && 
                 PathTrack[PathTrackFillingSize-2] == 3 && PathTrack[PathTrackFillingSize-1] == 2) {
          ESP_LOGI("Roode pathTracking", "Exit detected.");
          this->updateCounter(-1);
          last_valid_crossing_ts_ = millis();
          if (entry_exit_event_sensor != nullptr) {
            if (use_sensor_task_) {
              strncpy((char *) pending_entry_exit_event_, "Exit", sizeof(pending_entry_exit_event_) - 1);
              pending_entry_exit_event_[sizeof(pending_entry_exit_event_) - 1] = '\0';
              entry_exit_event_pending_ = true;
            } else {
              entry_exit_event_sensor->publish_state("Exit");
            }
          }
        }
      }

      PathTrackFillingSize = 0;
      PathTrack[0] = 0;
      state_ = STATE_IDLE;
    } else {
      // update PathTrack if the status is different from the last recorded one
      if (PathTrackFillingSize == 0 || AllZonesCurrentStatus != PathTrack[PathTrackFillingSize - 1]) {
        if (PathTrackFillingSize < 4) {
          PathTrackFillingSize++;
          PathTrack[PathTrackFillingSize - 1] = AllZonesCurrentStatus;
        }
      }
    }
  }

  if (presence_sensor != nullptr) {
    if (LeftPreviousStatus == NOBODY && RightPreviousStatus == NOBODY) {
      if (use_sensor_task_) {
        presence_state_ = false;
        presence_update_pending_ = true;
      } else {
        presence_sensor->publish_state(false);
      }
    } else {
      if (use_sensor_task_) {
        presence_state_ = true;
        presence_update_pending_ = true;
      } else {
        presence_sensor->publish_state(true);
      }
    }
  }
}

void Roode::update_status_text(const std::string &status) {
  if (status == last_status_text_)
    return;

  if (status_text_sensor != nullptr) {
    if (use_sensor_task_) {
      strncpy((char *) pending_status_text_, status.c_str(), sizeof(pending_status_text_) - 1);
      pending_status_text_[sizeof(pending_status_text_) - 1] = '\0';
      status_text_update_pending_ = true;
    } else {
      status_text_sensor->publish_state(status);
      last_status_text_ = status;
    }
  }
}

void Roode::calibrateDistance() { calibrate_zones(); }

void Roode::calibrate_zones() {
  run_zone_calibration(0);
  run_zone_calibration(1);
}

void Roode::publish_feature_list() {
  std::string features = "ULD";
#ifdef USE_WEBSERVER
  features += ",Portal";
#endif
  if (calibration_persistence_)
    features += ",Persist";
  if (use_sensor_task_)
    features += ",DualCore";
  if (enabled_features_sensor != nullptr)
    enabled_features_sensor->publish_state(features);
}

bool Roode::pause_sensor_task_if_needed_() {
  if (!use_sensor_task_ || sensor_task_handle_ == nullptr)
    return false;
  vTaskSuspend(sensor_task_handle_);
  return true;
}

void Roode::resume_sensor_task_if_needed_(bool paused) {
  if (paused && sensor_task_handle_ != nullptr)
    vTaskResume(sensor_task_handle_);
}

const RangingMode *Roode::determine_ranging_mode(uint16_t average_entry_zone_distance,
                                                 uint16_t average_exit_zone_distance) {
  uint16_t max_dist = std::max(average_entry_zone_distance, average_exit_zone_distance);
  if (max_dist <= short_distance_threshold)
    return &RANGING_SHORT;
  if (max_dist <= medium_distance_threshold)
    return &RANGING_MEDIUM;
  if (max_dist <= medium_long_distance_threshold)
    return &RANGING_MEDIUM_LONG;
  if (max_dist <= long_distance_threshold)
    return &RANGING_LONG;
  return &RANGING_LONGEST;
}

void Roode::publish_sensor_configuration(Zone *entry, Zone *exit, bool isMax) {
  if (isMax) {
    if (max_threshold_entry_sensor != nullptr)
      max_threshold_entry_sensor->publish_state(entry->threshold->max);
    if (max_threshold_exit_sensor != nullptr)
      max_threshold_exit_sensor->publish_state(exit->threshold->max);
  } else {
    if (min_threshold_entry_sensor != nullptr)
      min_threshold_entry_sensor->publish_state(entry->threshold->min);
    if (min_threshold_exit_sensor != nullptr)
      min_threshold_exit_sensor->publish_state(exit->threshold->min);
  }
  if (entry_roi_height_sensor != nullptr)
    entry_roi_height_sensor->publish_state(entry->roi->height);
  if (entry_roi_width_sensor != nullptr)
    entry_roi_width_sensor->publish_state(entry->roi->width);
  if (exit_roi_height_sensor != nullptr)
    exit_roi_height_sensor->publish_state(exit->roi->height);
  if (exit_roi_width_sensor != nullptr)
    exit_roi_width_sensor->publish_state(exit->roi->width);
}

void Roode::updateCounter(int delta) {
  if (people_counter == nullptr)
    return;
  float new_val = people_counter->state + delta;
  if (use_sensor_task_) {
    pending_people_counter_value_ = new_val;
    people_counter_update_pending_ = true;
  } else {
    auto call = people_counter->make_call();
    call.set_value(new_val);
    call.perform();
  }
  expected_counter_ = new_val;
}

void Roode::run_zone_calibration(uint8_t zone_id) {
  ESP_LOGI(CALIBRATION, "Calibration triggered for zone %d", zone_id);
  bool paused = pause_sensor_task_if_needed_();
  Zone *z = zone_id == 0 ? entry : exit;
  z->reset_roi(zone_id == 0 ? (orientation_ == Parallel ? 167 : 195) : (orientation_ == Parallel ? 231 : 60));
  z->calibrateThreshold(distanceSensor, 50);
  // Recalculate ROI sizes so thresholds remain consistent
  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  distanceSensor->set_ranging_mode(mode);

  calibration_data_[zone_id].baseline_mm = z->threshold->idle;
  calibration_data_[zone_id].threshold_min_mm = z->threshold->min;
  calibration_data_[zone_id].threshold_max_mm = z->threshold->max;
  calibration_data_[zone_id].last_calibrated_ts = static_cast<uint32_t>(time(nullptr));
  last_calibration_millis_ = millis();
  if (calibration_persistence_) {
    if (use_sensor_task_) {
      calibration_save_pending_[zone_id] = true;  // Let Core 0 save it safely
    } else {
      calibration_prefs_[zone_id].save(&calibration_data_[zone_id]);
    }
  }

  // Publish the updated calibration data so Home Assistant sees the new
  // thresholds and ROI values immediately after a fail-safe recalibration
  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);
  last_calibration_ts_ =
      std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
  publish_feature_list();
  resume_sensor_task_if_needed_(paused);
}

void Roode::apply_cpu_optimizations(float cpu) {
  if (cpu_optimizations_active_ || cpu <= cpu_opt_activate_threshold_)
    return;
  ESP_LOGW(TAG, "CPU usage %.1f%% exceeded threshold, applying optimizations", cpu);
  polling_interval_ms_ = 30;

  // Avoid extremely small windows and accuracy-reducing filters.
  if (filter_window_ < 5) {
    filter_window_ = 5;
    entry->set_filter_window(5);
    exit->set_filter_window(5);
  }
  if (filter_mode_ != FILTER_MEDIAN) {
    filter_mode_ = FILTER_MEDIAN;
    entry->set_filter_mode(FILTER_MEDIAN);
    exit->set_filter_mode(FILTER_MEDIAN);
  }
  cpu_optimizations_active_ = true;
}

void Roode::reset_cpu_optimizations(float cpu) {
  if (!cpu_optimizations_active_ || cpu > cpu_opt_deactivate_threshold_)
    return;
  ESP_LOGI(TAG, "CPU usage %.1f%% stable, reverting optimizations", cpu);
  polling_interval_ms_ = 10;
  filter_window_ = default_filter_window_;
  entry->set_filter_window(default_filter_window_);
  exit->set_filter_window(default_filter_window_);
  filter_mode_ = default_filter_mode_;
  entry->set_filter_mode(default_filter_mode_);
  exit->set_filter_mode(default_filter_mode_);
  cpu_optimizations_active_ = false;
}

void Roode::update_metrics() {
  uint32_t now = millis();
  if (now - loop_window_start_ < 10000)
    return;
  float cpu = 0.0f;
  float loop_time = 0.0f;
  if (loop_count_ > 0) {
    loop_time = (float) loop_time_sum_ / (float) loop_count_ / 1000.0f;
    cpu = (loop_time / (float) polling_interval_ms_) * 100.0f;
  }
  if (use_sensor_task_) {
    pending_loop_time_ = loop_time;
    pending_cpu_usage_ = cpu;
    pending_ram_free_ = (float) ESP.getFreeHeap() / (float) ESP.getHeapSize() * 100.0f;
    pending_flash_free_ = 100.0f;  // TO DO
    metrics_update_pending_ = true;
  } else {
    if (loop_time_sensor != nullptr)
      loop_time_sensor->publish_state(loop_time);
    if (cpu_usage_sensor != nullptr)
      cpu_usage_sensor->publish_state(cpu);
    if (ram_free_sensor != nullptr)
      ram_free_sensor->publish_state((float) ESP.getFreeHeap() / (float) ESP.getHeapSize() * 100.0f);
  }
  apply_cpu_optimizations(cpu);
  reset_cpu_optimizations(cpu);
  loop_window_start_ = now;
  loop_time_sum_ = 0;
  loop_count_ = 0;
}

void Roode::recalibration() { calibrate_zones(); }

void Roode::restart_sensor() {
  uint32_t now = millis();
  if (now - last_sensor_restart_ts_ > restart_timeout_ms_)
    restart_attempt_count_ = 0;
  restart_attempt_count_++;
  ESP_LOGW(TAG, "sensor_restart_attempt_%u", restart_attempt_count_);
  log_event(std::string("sensor_restart_attempt_") + std::to_string(restart_attempt_count_));
  distanceSensor->restart();
  last_sensor_restart_ts_ = now;
  invalid_read_count_ = 0;
  if (restart_attempt_count_ >= max_restart_attempts_) {
    ESP_LOGE(TAG, "sensor_restart_escalating_reset");
    log_event("sensor_restart_escalating_reset");
    ESP.restart();
  }
}

void Roode::sensor_task(void *param) {
  auto *self = static_cast<Roode *>(param);
  // Register this task with the watchdog when running on ESP32
#ifdef CONFIG_IDF_TARGET_ESP32
  esp_task_wdt_add(nullptr);
#endif
  for (;;) {
#ifdef CONFIG_IDF_TARGET_ESP32
    // Feed the watchdog to prevent unwanted resets
    esp_task_wdt_reset();
#endif
    self->use_sensor_task_ = true;

    if (!self->sensor_enabled_ || self->distanceSensor->is_failed()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (self->scan_state_ == SCANNING) {
      self->scan_step_ = "Grid Scanning (8x8)";
      std::vector<uint16_t> rates;
      std::vector<uint16_t> distances;
      rates.reserve(64);
      distances.reserve(64);
      for (uint8_t y = 0; y < 8; y++) {
        for (uint8_t x = 0; x < 8; x++) {
          if (self->scan_state_ != SCANNING) break;
          ROI roi;
          roi.width = 4;
          roi.height = 4;
          roi.center = (y * 2 + 1) * 16 + (x * 2 + 1);
#ifdef CONFIG_IDF_TARGET_ESP32
          i2c_lock();
#endif
          VL53L1_Error status;
          auto dist = self->distanceSensor->read_distance(&roi, status);
          auto rate = self->distanceSensor->get_signal_rate();
#ifdef CONFIG_IDF_TARGET_ESP32
          i2c_unlock();
#endif
          rates.push_back(rate.value_or(0));
          distances.push_back(dist.value_or(0));
          self->scan_progress_ = (rates.size() * 100) / 64;
          vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (self->scan_state_ != SCANNING) break;
      }
      if (self->scan_state_ == SCANNING) {
        self->scan_step_ = "Analyzing results";
        // Simple analysis: find the SPAD with the longest distance and reasonable signal
        uint8_t best_idx = 0;
        uint16_t max_dist = 0;
        for (uint8_t i = 0; i < 64; i++) {
          if (distances[i] > max_dist && distances[i] < 4000) {
            max_dist = distances[i];
            best_idx = i;
          }
        }
        uint8_t bx = best_idx % 8;
        uint8_t by = best_idx / 8;
        
        self->recommended_settings_.roi_width = 6;
        self->recommended_settings_.roi_height = 16;
        self->recommended_settings_.roi_center = (by * 2 + 1) * 16 + (bx * 2 + 1);
        self->recommended_settings_.entry_center = self->recommended_settings_.roi_center;
        self->recommended_settings_.exit_center = self->recommended_settings_.roi_center; // To be refined
        self->recommended_settings_.min_threshold = 15;
        self->recommended_settings_.max_threshold = 85;
        self->has_recommended_settings_ = true;

        self->scan_state_ = SCAN_IDLE;
        self->scan_step_ = "Complete";
        self->scan_progress_ = 100;
        
        ScanSession s;
        s.id = self->active_scan_id_;
        s.ts = millis() / 1000;
        s.duration_sec = (millis() - self->scan_start_ts_) / 1000;
        s.size_bytes = rates.size() * 2 + distances.size() * 2;
        self->sessions_.push_back(s);
        self->scan_results_[self->active_scan_id_] = std::move(rates);
      } else {
        self->scan_state_ = SCAN_IDLE;
        self->scan_step_ = "Idle";
        self->scan_progress_ = 0;
      }
      continue;
    }

    uint32_t now = millis();
    if (self->last_loop_update_ts_ != 0 && (now - self->last_loop_update_ts_ > self->restart_timeout_ms_) &&
        (now - self->last_sensor_restart_ts_ > self->restart_timeout_ms_)) {
      ESP_LOGW(TAG, "Sensor unresponsive >%ds, restarting...", self->restart_timeout_ms_ / 1000);
#ifdef CONFIG_IDF_TARGET_ESP32
      i2c_lock();
#endif
      self->restart_sensor();
#ifdef CONFIG_IDF_TARGET_ESP32
      i2c_unlock();
#endif
    }
    unsigned long start = micros();
    self->AnEventHasOccured = 0;
    self->AllZonesCurrentStatus = 0;
    self->read_and_track_zone_(self->entry, true);
    self->read_and_track_zone_(self->exit, false);
    // Attempt to recover the sensor when repeated invalid distance values are observed
    if (self->invalid_read_count_ > self->invalid_distance_limit_ &&
        (now - self->last_sensor_restart_ts_ > self->restart_timeout_ms_)) {
      ESP_LOGW(TAG, "Consecutive invalid distances, restarting...");
#ifdef CONFIG_IDF_TARGET_ESP32
      i2c_lock();
#endif
      self->restart_sensor();
#ifdef CONFIG_IDF_TARGET_ESP32
      i2c_unlock();
#endif
    }
    unsigned long end = micros();
    unsigned long delta = end - start;
    self->loop_time_sum_ += delta;
    self->loop_count_++;
    self->update_metrics();
    if (self->auto_calibration_interval_sec_ > 0 &&
        (now - self->last_calibration_millis_ >= self->auto_calibration_interval_sec_ * 1000)) {
      self->run_zone_calibration(0);
      self->run_zone_calibration(1);
    }
    vTaskDelay(pdMS_TO_TICKS(self->polling_interval_ms_));
  }
}
}  // namespace roode
}  // namespace esphome
