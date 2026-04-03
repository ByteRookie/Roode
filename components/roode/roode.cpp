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
  </style>
</head>
<body>
  <div class="wrap">
    <div id="errorBanner" class="banner"></div>
    <div class="hdr">
      <h1>Roode Calibration Portal</h1>
      <div class="status"><span id="statusText">Status: <b>Idle</b></span> · <span id="lastCal">Last calibration: —</span></div>
    </div>

    <!-- Controls & live status -->
    <div class="row">
      <div class="card" style="flex:1 1 100%">
        <div class="btns">
          <button class="btn primary" id="btnStart">Start Calibration</button>
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
        <h2>Current Settings</h2>
        <div class="kv" id="currentSettings">
          <div class="k">ROI Size</div><div id="curRoiSize">—</div>
          <div class="k">ROI Center</div><div id="curRoiCenter">—</div>
          <div class="k">Entry Center</div><div id="curEntry">—</div>
          <div class="k">Exit Center</div><div id="curExit">—</div>
          <div class="k">Ranging Mode</div><div id="curMode">—</div>
          <div class="k">Min Threshold</div><div id="curMin">—</div>
          <div class="k">Max Threshold</div><div id="curMax">—</div>
          <div class="k">Sampling</div><div id="curSampling">—</div>
          <div class="k">Firmware</div><div id="curFw">—</div>
          <div class="k">Live Distance</div><div id="curLive">—</div>
          <div class="k">Zone State</div><div id="curZones">—</div>
          <div class="k">People Counter</div><div id="curCount">—</div>
        </div>
        <div class="chips">
          <button class="btn" id="btnCopyJSON">Copy as JSON</button>
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
          <div class="k">Ranging Mode</div><div id="pvMode">—</div>
          <div class="k">Min Threshold</div><div id="pvMin">—</div>
          <div class="k">Max Threshold</div><div id="pvMax">—</div>
          <div class="k">Sampling</div><div id="pvSampling">—</div>
          <div class="k">Firmware</div><div id="pvFw">—</div>
        </div>
        <div class="chips">
          <button class="btn" id="btnApply" disabled>Apply Settings</button>
          <a class="btn" id="dlResult" href="#" download>Save roi_result.json</a>
        </div>
      </div>

      <!-- Past Sessions -->
      <div class="card" style="flex:1 1 100%">
        <div style="display:flex;align-items:center;justify-content:space-between">
          <h2>Past Sessions</h2>
          <a class="btn ghost" id="btnExport" href="/api/export/all">Export All ▾</a>
        </div>
        <div style="overflow:auto">
          <table id="tblSessions">
            <thead>
              <tr>
                <th>#</th><th>Date/Time</th><th>Session ID</th><th>Trials</th>
                <th>Duration</th><th>Size</th><th>Actions</th>
              </tr>
            </thead>
            <tbody>
              <!-- Placeholder row (will be replaced by JS when data is available) -->
              <tr>
                <td>1</td><td>2025-08-26 17:00</td><td>calibration_2025-08-26_1700</td>
                <td>3</td><td>02:15</td><td>24.1KB</td>
                <td class="mono">
                  <a href="/api/scan/session/calibration_2025-08-26_1700">Download</a>
                  &nbsp;|&nbsp;<a href="#" data-view="calibration_2025-08-26_1700">View</a>
                  &nbsp;|&nbsp;<a href="#" data-del="calibration_2025-08-26_1700">Delete</a>
                </td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>

    </div><!-- /row -->
  </div><!-- /wrap -->

  <script>
  const TOKEN = new URLSearchParams(location.search).get('token') || '';
  const TOKEN_HEADER = TOKEN ? {'X-Portal-Token': TOKEN} : {};
  const tokenParam = TOKEN ? `?token=${encodeURIComponent(TOKEN)}` : '';
  if(TOKEN){
    const exp = document.getElementById('btnExport');
    if(exp) exp.href = '/api/export/all'+tokenParam;
  }
  // Helpers
  const $ = (s)=>document.querySelector(s);
  const showToast = (msg, ok=false)=>{
    const b = $('#errorBanner');
    if(!b) return;
    b.textContent = msg || '';
    b.style.background = ok ? 'var(--ok)' : 'var(--err)';
    b.style.display = 'block';
    clearTimeout(b._hide);
    b._hide = setTimeout(()=>{ b.style.display='none'; }, 5000);
  };
  const showError = (msg)=>showToast(msg || 'Request failed', false);
  const showSuccess = (msg)=>showToast(msg || 'Success', true);
  const fmtBytes = n => {
    if(n==null) return '—';
    const u=['B','KB','MB']; let i=0, v=n;
    while(v>=1024 && i<u.length-1){ v/=1024; i++; }
    return (i?v.toFixed(1):v|0) + ' ' + u[i];
  };
  const fmtDur = s => {
    if(!Number.isFinite(s)) return '—';
    const m = Math.floor(s/60), sec = s%60;
    return String(m).padStart(2,'0')+':'+String(sec).padStart(2,'0');
  };

  // State
  let current = null, preview = null, status = null, sessions = [];

  // Fetch JSON with graceful fallback
  async function getJSON(url){
    try {
      const r = await fetch(url, {cache:'no-store', headers: TOKEN_HEADER});
      if(!r.ok) throw new Error(r.status + ' ' + r.statusText);
      return await r.json();
    } catch(e) {
      showError(e.message);
      return null;
    }
  }
  async function postJSON(url, body){
    try {
      const headers = {'Content-Type':'application/json'};
      if(TOKEN) headers['X-Portal-Token'] = TOKEN;
      const r = await fetch(url, {method:'POST', headers, body: body?JSON.stringify(body):'{}'});
      if(!r.ok) throw new Error(r.status + ' ' + r.statusText);
      return await r.json().catch(()=>({ok:true}));
    } catch(e) {
      showError(e.message);
      return { ok:false };
    }
  }

  // Renderers
  function renderStatus(){
    const st = status || {state:'Idle', id:'—', step:'—', progress:0};
    $('#statusText').innerHTML = 'Status: <b>'+st.state+'</b>';
    $('#activeSession').textContent = st.id || '—';
    $('#stepText').textContent = st.step || '—';
    $('#progressBar').style.width = (st.progress||0)+'%';
    $('#btnCancel').style.display = (st.state==='Scanning') ? 'inline-block' : 'none';
  }
  function renderCurrent(){
    if(!current) return;
    $('#curRoiSize').textContent = (current.roi_width&&current.roi_height) ? `${current.roi_width} × ${current.roi_height}` : '—';
    $('#curRoiCenter').textContent = current.roi_center ? `(${current.roi_center.x}, ${current.roi_center.y})` : '—';
    $('#curEntry').textContent = current.entry_center ? `(${current.entry_center.x}, ${current.entry_center.y})` : '—';
    $('#curExit').textContent = current.exit_center ? `(${current.exit_center.x}, ${current.exit_center.y})` : '—';
    $('#curMode').textContent = current.ranging_mode || '—';
    $('#curMin').textContent = current.min_threshold!=null ? current.min_threshold+'%' : '—';
    $('#curMax').textContent = current.max_threshold!=null ? current.max_threshold+'%' : '—';
    $('#curSampling').textContent = current.sampling || '—';
    $('#curFw').textContent = current.firmware || '—';
    $('#curLive').textContent = (current.distance_entry_mm!=null && current.distance_exit_mm!=null)
      ? `${current.distance_entry_mm} mm / ${current.distance_exit_mm} mm`
      : '—';
    $('#curZones').textContent = `${current.entry_active ? 'Entry active' : 'Entry idle'} · ${current.exit_active ? 'Exit active' : 'Exit idle'}`;
    $('#curCount').textContent = current.people_counter!=null ? current.people_counter : '—';
    if(current.last_calibration) $('#lastCal').textContent = 'Last calibration: ' + new Date(current.last_calibration).toLocaleString();
  }
  function renderPreview(){
    const apply = $('#btnApply');
    if(!preview){ // clear
      $('#pvRoiSize').textContent = '—'; $('#pvRoi').textContent='—';
      $('#pvEntry').textContent='—'; $('#pvExit').textContent='—';
      $('#pvMode').textContent='—'; $('#pvMin').textContent='—';
      $('#pvMax').textContent='—'; $('#pvSampling').textContent='—'; $('#pvFw').textContent='—';
      $('#dlResult').removeAttribute('href');
      if(apply) apply.disabled = true;
      return;
    }
    const r = preview;
    if(r.roi){ $('#pvRoiSize').textContent = `${r.roi.width} × ${r.roi.height}`; $('#pvRoi').textContent = `(${r.roi.center.x}, ${r.roi.center.y})`; }
    if(r.zones){ $('#pvEntry').textContent = `(${r.zones.entry.x}, ${r.zones.entry.y})`; $('#pvExit').textContent = `(${r.zones.exit.x}, ${r.zones.exit.y})`; }
    $('#pvMode').textContent = r.ranging_mode || '—';
    if(r.thresholds){ $('#pvMin').textContent = r.thresholds.min+'%'; $('#pvMax').textContent = r.thresholds.max+'%'; }
    $('#pvSampling').textContent = r.sampling || '—';
    $('#pvFw').textContent = r.firmware || '—';
    if(r.download) $('#dlResult').href = r.download + (TOKEN ? (r.download.includes('?') ? '&' : '?') + 'token=' + encodeURIComponent(TOKEN) : '');
    if(apply) apply.disabled = false;
  }
  function renderSessions(){
    const tbody = document.querySelector('#tblSessions tbody');
    tbody.innerHTML = '';
    if(!sessions || !sessions.length){
      const tr = document.createElement('tr');
      tr.innerHTML = '<td colspan="7" class="muted">No sessions yet.</td>';
      tbody.appendChild(tr);
      return;
    }
    sessions.sort((a,b)=>new Date(b.ts)-new Date(a.ts)).forEach((s, i)=>{
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td>${i+1}</td>
        <td>${new Date(s.ts).toLocaleString()}</td>
        <td class="mono">${s.id}</td>
        <td>${s.trials ?? '—'}</td>
        <td>${fmtDur(s.duration_sec)}</td>
        <td>${fmtBytes(s.size_bytes)}</td>
        <td class="mono">
          <a href="/api/scan/session/${encodeURIComponent(s.id)}${tokenParam}">Download</a>
          &nbsp;|&nbsp;<a href="#" data-view="${s.id}">View</a>
          &nbsp;|&nbsp;<a href="#" data-del="${s.id}">Delete</a>
        </td>`;
      tbody.appendChild(tr);
    });
  }

  // Polling
  async function poll(){
    const [cur, stat, list, prev] = await Promise.all([
      getJSON('/api/settings/current'),
      getJSON('/api/scan/status'),
      getJSON('/api/scan/sessions'),
      getJSON('/api/roi/preview')
    ]);
    if(cur) current = cur;
    if(stat) status = stat;
    if(list) sessions = list;
    // Only show preview if we have one (typically after completion)
    preview = prev || null;

    renderStatus(); renderCurrent(); renderPreview(); renderSessions();
  }

  // Actions
  $('#btnStart').addEventListener('click', async (e)=>{
    const btn = e.target;
    btn.disabled = true;
    try {
      const r = await postJSON('/api/scan/start');
      if(r && r.id){ status = {state:'Scanning', id:r.id, step:'8×8 Scan', progress:0}; renderStatus(); }
    } finally {
      btn.disabled = false;
    }
  });
  $('#btnCancel').addEventListener('click', async (e)=>{
    const btn = e.target;
    btn.disabled = true;
    try {
      await postJSON('/api/scan/cancel');
      status = {state:'Idle', id:status?.id || '—', step:'—', progress:0};
      renderStatus();
    } finally {
      btn.disabled = false;
    }
  });
  $('#btnApply').addEventListener('click', async (e)=>{
    const btn = e.target;
    btn.disabled = true;
    try {
      const r = await postJSON('/api/roi/apply');
      if(r && r.ok){
        showSuccess('Settings applied');
      } else {
        showError('Apply failed');
      }
    } finally {
      btn.disabled = false;
    }
  });
  $('#btnCopyJSON').addEventListener('click', ()=>{
    if(!current) return;
    navigator.clipboard.writeText(JSON.stringify(current, null, 2));
  });
  document.querySelector('#tblSessions').addEventListener('click', async (e)=>{
    const a = e.target.closest('a'); if(!a) return;
    if(a.dataset.view){ e.preventDefault();
      a.classList.add('disabled');
      try {
        // Prefer a queryable preview endpoint per session if available:
        const prev = await getJSON(`/api/roi/preview?session_id=${encodeURIComponent(a.dataset.view)}`);
        preview = prev || preview; renderPreview();
        document.querySelector('#preview')?.scrollIntoView({behavior:'smooth', block:'start'});
      } finally {
        a.classList.remove('disabled');
      }
    }
    if(a.dataset.del){ e.preventDefault();
      a.classList.add('disabled');
      try {
        await postJSON('/api/scan/delete', { session_id: a.dataset.del });
        sessions = (sessions||[]).filter(s => s.id !== a.dataset.del);
        renderSessions();
      } finally {
        a.classList.remove('disabled');
      }
    }
  });

  // Kick off
  poll();
  setInterval(poll, 1500);
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
  } else if (msg == "xshut_reinitialize") {
    out += " - reinitializing";
  } else if (msg.rfind("sensor_", 0) == 0 && msg.find("_addr") != std::string::npos) {
    size_t start = sizeof("sensor_") - 1;
    size_t end = msg.find('_', start);
    std::string id = msg.substr(start, end - start);
    std::string addr = msg.substr(msg.find("0x") + 2);
    out += " - address 0x" + addr + " for sensor " + id;
  } else if (msg.rfind("sensor_", 0) == 0 && msg.find(".recovered_via_xshut") != std::string::npos) {
    std::string id = msg.substr(sizeof("sensor_") - 1, msg.find('.') - (sizeof("sensor_") - 1));
    out += " - sensor " + id + " recovered via XSHUT";
  } else if (msg == "sensor.recovered_via_xshut") {
    out += " - sensor recovered via XSHUT";
  } else if (msg == "interrupt_fallback_polling" || msg == "interrupt_fallback")
    out += " - INT pin timeout, polling";
  else if (msg == "int_pin_missed")
    out += " - INT miss";
  else if (msg.rfind("int_pin_missed_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("int_pin_missed_sensor_") - 1);
    out += " - INT miss sensor " + id;
  } else if (msg.rfind("manual_adjust", 0) == 0)
    out += " - user corrected";
  const char *color = "\033[32m";  // green by default
  if (msg.find("fail") != std::string::npos || msg.find("fallback") != std::string::npos ||
      msg.find("missed") != std::string::npos)
    color = "\033[31m";  // red for errors
  else if (msg.find("retry") != std::string::npos || msg.find("manual_adjust") != std::string::npos ||
           msg.find("reinitialize") != std::string::npos || msg.find("pulse_off") != std::string::npos)
    color = "\033[33m";  // yellow for informational

  std::string colored = std::string(color) + out + "\033[0m";
  ESP_LOGI(TAG, "%s", colored.c_str());
  if (instance_ != nullptr) {
    if (msg.find("reinitialize") != std::string::npos) {
      instance_->update_status_text("reinitializing");
    } else if (msg.find("recovered_via_xshut") != std::string::npos) {
      instance_->update_status_text("ok");
    }
    if (msg == "dual_core_success" || msg == "fallback_single_core" || msg == "force_single_core" ||
        msg == "interrupt_fallback_polling" || msg == "interrupt_recovered") {
      instance_->publish_feature_list();
    }
  }
}

Roode::~Roode() {
  delete entry;
  delete exit;
}
void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Sample size: %d", samples);
  LOG_UPDATE_INTERVAL(this);
  entry->dump_config();
  exit->dump_config();
}

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
#endif

#ifdef USE_WEBSERVER
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
    DynamicJsonDocument doc(1024);
    doc["roi_width"] = entry->roi->width;
    doc["roi_height"] = entry->roi->height;
    JsonObject roi_center = doc.createNestedObject("roi_center");
    roi_center["x"] = entry->roi->center % 16;
    roi_center["y"] = entry->roi->center / 16;
    JsonObject entry_center = doc.createNestedObject("entry_center");
    entry_center["x"] = entry->roi->center % 16;
    entry_center["y"] = entry->roi->center / 16;
    JsonObject exit_center = doc.createNestedObject("exit_center");
    exit_center["x"] = exit->roi->center % 16;
    exit_center["y"] = exit->roi->center / 16;
    doc["ranging_mode"] = distanceSensor->is_failed() ? "offline" : "auto";
    doc["min_threshold"] = entry->threshold->min_percentage.value_or(15);
    doc["max_threshold"] = entry->threshold->max_percentage.value_or(80);
    doc["sampling"] = samples;
    doc["firmware"] = VERSION;
    doc["last_calibration"] = last_calibration_ts_;
    doc["distance_entry_mm"] = entry->getDistance();
    doc["distance_exit_mm"] = exit->getDistance();
    doc["entry_active"] = LeftPreviousStatus == SOMEONE;
    doc["exit_active"] = RightPreviousStatus == SOMEONE;
    doc["people_counter"] = people_counter != nullptr ? people_counter->state : 0.0f;
    std::string output;
    serializeJson(doc, output);
    request->send(request->beginResponse(200, "application/json", output));
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/scan/status", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    DynamicJsonDocument doc(256);
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
    DynamicJsonDocument doc(128);
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
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    // This endpoint typically receives recommended ROI from the portal
    // For now we just return OK
    request->send(200, "application/json", "{\"ok\":true}");
  }));

  base->add_handler_without_auth(new LambdaRequestHandler(HTTP_GET, "/api/roi/preview", [this](web_server_idf::AsyncWebServerRequest *request) {
    if (!this->require_portal_auth_(request, "application/json"))
      return;
    // Return a dummy preview for now
    request->send(200, "application/json", "null");
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
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.to<JsonArray>();
    for (const auto &s : sessions_) {
      JsonObject obj = arr.createNestedObject();
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
    std::string uri = request->url().c_str();
    std::string id = uri.substr(uri.find_last_of('/') + 1);
    if (this->scan_results_.count(id)) {
      DynamicJsonDocument doc(4096);
      doc["type"] = "passive_scan";
      doc["grid"] = 8;
      doc["ranging"] = "auto";
      JsonArray data = doc.createNestedObject("data").createNestedArray("mcps");
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
      if (max_threshold_entry_sensor != nullptr) max_threshold_entry_sensor->publish_state(pending_max_th_[0]);
      if (max_threshold_exit_sensor != nullptr) max_threshold_exit_sensor->publish_state(pending_max_th_[1]);
      if (min_threshold_entry_sensor != nullptr) min_threshold_entry_sensor->publish_state(pending_min_th_[0]);
      if (min_threshold_exit_sensor != nullptr) min_threshold_exit_sensor->publish_state(pending_min_th_[1]);
      if (entry_roi_height_sensor != nullptr) entry_roi_height_sensor->publish_state(pending_roi_h_[0]);
      if (entry_roi_width_sensor != nullptr) entry_roi_width_sensor->publish_state(pending_roi_w_[0]);
      if (exit_roi_height_sensor != nullptr) exit_roi_height_sensor->publish_state(pending_roi_h_[1]);
      if (exit_roi_width_sensor != nullptr) exit_roi_width_sensor->publish_state(pending_roi_w_[1]);
      config_update_pending_ = false;
    }
    for (int i = 0; i < 2; i++) {
      if (calibration_save_pending_[i]) {
        calibration_prefs_[i].save(&calibration_data_[i]);
        calibration_save_pending_[i] = false;
      }
    }
    return;
  }
  uint32_t now = millis();
  if (last_loop_update_ts_ != 0 && (now - last_loop_update_ts_ > restart_timeout_ms_) &&
      (now - last_sensor_restart_ts_ > restart_timeout_ms_)) {
    ESP_LOGW(TAG, "Sensor unresponsive >%ds, restarting...", restart_timeout_ms_ / 1000);
    restart_sensor();
  }
  unsigned long start = micros();
  AnEventHasOccured = 0;
  AllZonesCurrentStatus = 0;
  this->read_and_track_zone_(this->entry, true);
  this->read_and_track_zone_(this->exit, false);
  // Attempt to recover the sensor when repeated invalid distance values are observed
  if (invalid_read_count_ > invalid_distance_limit_ && (now - last_sensor_restart_ts_ > restart_timeout_ms_)) {
    ESP_LOGW(TAG, "Consecutive invalid distances, restarting...");
    restart_sensor();
  }
  unsigned long end = micros();
  unsigned long delta = end - start;
  loop_time_sum_ += delta;
  loop_count_++;
  update_metrics();
  delay(polling_interval_ms_);
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
    PathTrackFillingSize = 1;
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
      if (PathTrackFillingSize == 4) {
        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
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
        } else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
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
      }

      PathTrackFillingSize = 1;
      state_ = STATE_IDLE;
    } else {
      // update PathTrack if the status is different from the last recorded one
      if (AllZonesCurrentStatus != PathTrack[PathTrackFillingSize - 1]) {
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
    }
  }
}
void Roode::updateCounter(int delta) {
  if (this->people_counter == nullptr) {
    return;
  }
  auto next = this->people_counter->state + (float) delta;
  ESP_LOGI(TAG, "Updating people count: %d", (int) next);
  expected_counter_ = next;
  if (use_sensor_task_) {
    pending_people_counter_value_ = next;
    people_counter_update_pending_ = true;
  } else {
    auto call = this->people_counter->make_call();
    call.set_value(next);
    call.perform();
  }
}
void Roode::recalibration() { calibrate_zones(); }

bool Roode::pause_sensor_task_if_needed_() {
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!use_sensor_task_ || sensor_task_handle_ == nullptr)
    return false;
  if (xTaskGetCurrentTaskHandle() == sensor_task_handle_)
    return false;
  i2c_lock();
  vTaskSuspend(sensor_task_handle_);
  return true;
#else
  return false;
#endif
}

void Roode::resume_sensor_task_if_needed_(bool paused) {
#ifdef CONFIG_IDF_TARGET_ESP32
  if (paused && sensor_task_handle_ != nullptr) {
    vTaskResume(sensor_task_handle_);
    i2c_unlock();
  }
#else
  (void) paused;
#endif
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
    loop_time = (float) loop_time_sum_ / loop_count_ / 1000.0f;
    cpu = ((float) loop_time_sum_ / ((now - loop_window_start_) * 1000.0f)) * 100.0f;
  }
  float ram_used = 0;
  float flash_used = 0;
#ifdef CONFIG_IDF_TARGET_ESP32
  uint32_t total_heap = ESP.getHeapSize();
  if (total_heap > 0) {
    uint32_t used = total_heap - ESP.getFreeHeap();
    ram_used = ((float) used / (float) total_heap) * 100.0f;
  }
  uint32_t total_flash = ESP.getFlashChipSize();
  if (total_flash > 0) {
    uint32_t used = total_flash - ESP.getFreeSketchSpace();
    flash_used = ((float) used / (float) total_flash) * 100.0f;
  }
#else
  ram_used = (1.0f - (float)ESP.getFreeHeap() / 81920.0f) * 100.0f;
  flash_used = (1.0f - (float)ESP.getFreeSketchSpace() / 1048576.0f) * 100.0f;
#endif

  if (use_sensor_task_) {
    pending_loop_time_ = loop_time;
    pending_cpu_usage_ = cpu;
    pending_ram_free_ = ram_used;
    pending_flash_free_ = flash_used;
    metrics_update_pending_ = true;
  } else {
    if (loop_time_sensor != nullptr)
      loop_time_sensor->publish_state(loop_time);
    if (cpu_usage_sensor != nullptr)
      cpu_usage_sensor->publish_state(cpu);
    if (ram_free_sensor != nullptr)
      ram_free_sensor->publish_state(ram_used);
    if (flash_free_sensor != nullptr)
      flash_free_sensor->publish_state(flash_used);
  }

  apply_cpu_optimizations(cpu);
  reset_cpu_optimizations(cpu);
  loop_time_sum_ = 0;
  loop_count_ = 0;
  loop_window_start_ = now;
}

const RangingMode *Roode::determine_ranging_mode(uint16_t average_entry_zone_distance,
                                                 uint16_t average_exit_zone_distance) {
  uint16_t min = average_entry_zone_distance < average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  uint16_t max = average_entry_zone_distance > average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  if (min <= short_distance_threshold) {
    return Ranging::Short;
  }
  if (max > short_distance_threshold && min <= medium_distance_threshold) {
    return Ranging::Medium;
  }
  if (max > medium_distance_threshold && min <= medium_long_distance_threshold) {
    return Ranging::Long;
  }
  if (max > medium_long_distance_threshold && min <= long_distance_threshold) {
    return Ranging::Longer;
  }
  return Ranging::Longest;
}

void Roode::calibrate_zones() {
  if (this->distanceSensor->is_failed()) {
    ESP_LOGW(TAG, "Skipping zone calibration: sensor is offline");
    return;
  }
  bool paused = pause_sensor_task_if_needed_();
  ESP_LOGI(SETUP, "Calibrating sensor zones");

  entry->reset_roi(orientation_ == Parallel ? 167 : 195);
  exit->reset_roi(orientation_ == Parallel ? 231 : 60);

  calibrateDistance();

  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  entry->calibrateThreshold(distanceSensor, 50);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->calibrateThreshold(distanceSensor, 50);

  publish_sensor_configuration(entry, exit, true);
  App.feed_wdt();
  publish_sensor_configuration(entry, exit, false);

  calibration_data_[0] = {entry->threshold->idle, entry->threshold->min, entry->threshold->max,
                          static_cast<uint32_t>(time(nullptr))};
  calibration_data_[1] = {exit->threshold->idle, exit->threshold->min, exit->threshold->max,
                          static_cast<uint32_t>(time(nullptr))};
  last_calibration_millis_ = millis();

  if (calibration_persistence_) {
    if (use_sensor_task_) {
      calibration_save_pending_[0] = true;
      calibration_save_pending_[1] = true;
    } else {
      calibration_prefs_[0].save(&calibration_data_[0]);
      calibration_prefs_[1].save(&calibration_data_[1]);
    }
  }
  ESP_LOGI(SETUP, "Finished calibrating sensor zones");
  last_calibration_ts_ =
      std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
  publish_feature_list();
  resume_sensor_task_if_needed_(paused);
}

void Roode::calibrateDistance() {
  if (this->distanceSensor->is_failed()) {
    return;
  }
  auto *const initial = distanceSensor->get_ranging_mode_override().value_or(Ranging::Longest);
  distanceSensor->set_ranging_mode(initial);

  entry->calibrateThreshold(distanceSensor, 50);
  exit->calibrateThreshold(distanceSensor, 50);

  if (distanceSensor->get_ranging_mode_override().has_value()) {
    return;
  }
  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  if (mode != initial) {
    distanceSensor->set_ranging_mode(mode);
  }
}

void Roode::publish_sensor_configuration(Zone *entry, Zone *exit, bool isMax) {
  if (use_sensor_task_) {
    pending_max_th_[0] = entry->threshold->max;
    pending_max_th_[1] = exit->threshold->max;
    pending_min_th_[0] = entry->threshold->min;
    pending_min_th_[1] = exit->threshold->min;
    pending_roi_h_[0] = entry->roi->height;
    pending_roi_h_[1] = exit->roi->height;
    pending_roi_w_[0] = entry->roi->width;
    pending_roi_w_[1] = exit->roi->width;
    config_update_pending_ = true;
    return;
  }
  if (isMax) {
    if (max_threshold_entry_sensor != nullptr) {
      max_threshold_entry_sensor->publish_state(entry->threshold->max);
    }

    if (max_threshold_exit_sensor != nullptr) {
      max_threshold_exit_sensor->publish_state(exit->threshold->max);
    }
  } else {
    if (min_threshold_entry_sensor != nullptr) {
      min_threshold_entry_sensor->publish_state(entry->threshold->min);
    }
    if (min_threshold_exit_sensor != nullptr) {
      min_threshold_exit_sensor->publish_state(exit->threshold->min);
    }
  }

  if (entry_roi_height_sensor != nullptr) {
    entry_roi_height_sensor->publish_state(entry->roi->height);
  }
  if (entry_roi_width_sensor != nullptr) {
    entry_roi_width_sensor->publish_state(entry->roi->width);
  }

  if (exit_roi_height_sensor != nullptr) {
    exit_roi_height_sensor->publish_state(exit->roi->height);
  }
  if (exit_roi_width_sensor != nullptr) {
    exit_roi_width_sensor->publish_state(exit->roi->width);
  }
}

void Roode::publish_feature_list() {
  auto fmt_bytes = [](uint32_t bytes) {
    char buf[16];
    if (bytes >= 1024UL * 1024UL * 1024UL)
      snprintf(buf, sizeof(buf), "%uGB", bytes / 1024 / 1024 / 1024);
    else if (bytes >= 1024 * 1024)
      snprintf(buf, sizeof(buf), "%uMB", bytes / 1024 / 1024);
    else
      snprintf(buf, sizeof(buf), "%uKB", bytes / 1024);
    return std::string(buf);
  };

  auto fmt_time = [](uint32_t epoch) {
    if (epoch == 0)
      return std::string("unknown");
    time_t t = epoch;
    struct tm tm_time;
    if (!localtime_r(&t, &tm_time))
      return std::string("unknown");
    char buf[8];
    int hour = tm_time.tm_hour % 12;
    if (hour == 0)
      hour = 12;
    snprintf(buf, sizeof(buf), "%d:%02d%cM", hour, tm_time.tm_min, tm_time.tm_hour >= 12 ? 'P' : 'A');
    return std::string(buf);
  };

  std::vector<std::pair<std::string, std::string>> features;
#ifdef CONFIG_IDF_TARGET_ESP32
  features.push_back({"cpu_mode", use_sensor_task_ ? "dual" : "single"});
  features.push_back({"cpu", ESP.getChipModel()});
  features.push_back({"cpu_cores", std::to_string(ESP.getChipCores())});
#else
  features.push_back({"cpu_mode", "single"});
  features.push_back({"cpu", "ESP8266"});
  features.push_back({"cpu_cores", "1"});
#endif
  features.push_back({"xshut", distanceSensor->get_xshut_state().has_value() ? "enabled" : "disabled"});
  features.push_back({"refresh", distanceSensor->is_interrupt_enabled() ? "interrupt" : "polling"});
  features.push_back({"ram", fmt_bytes(ESP.getHeapSize())});
  features.push_back({"flash", fmt_bytes(ESP.getFlashChipSize())});
  features.push_back({"calibration_value", std::to_string(entry->threshold->idle)});
  uint32_t last_cal_epoch = std::max(calibration_data_[0].last_calibrated_ts, calibration_data_[1].last_calibrated_ts);
  features.push_back({"calibration", fmt_time(last_cal_epoch)});

  std::string feature_list;
  for (size_t i = 0; i < features.size(); ++i) {
    feature_list += features[i].first + ":" + features[i].second;
    if (i + 1 < features.size())
      feature_list += "\n";
  }
  if (enabled_features_sensor != nullptr) {
    if (use_sensor_task_) {
      feature_list_update_pending_ = true;
    } else {
      enabled_features_sensor->publish_state(feature_list);
    }
  }
  log_event(std::string("features_enabled: ") + feature_list);
}

void Roode::update_status_text(const std::string &status) {
  if (status_text_sensor != nullptr && status != last_status_text_) {
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

    if (self->scan_state_ == SCANNING) {
      self->scan_step_ = "8x8 Grid Scan";
      std::vector<uint16_t> scan_data;
      scan_data.reserve(64);
      for (uint8_t y = 0; y < 8; y++) {
        for (uint8_t x = 0; x < 8; x++) {
          if (self->scan_state_ != SCANNING) break;
          ROI roi;
          roi.width = 4;
          roi.height = 4;
          roi.center = (y * 2 + 1) * 16 + (x * 2 + 1); // Approx center of 2x2 SPAD block in 16x16 grid
#ifdef CONFIG_IDF_TARGET_ESP32
          i2c_lock();
#endif
          VL53L1_Error status;
          self->distanceSensor->read_distance(&roi, status);
          auto rate = self->distanceSensor->get_signal_rate();
#ifdef CONFIG_IDF_TARGET_ESP32
          i2c_unlock();
#endif
          scan_data.push_back(rate.value_or(0));
          self->scan_progress_ = (scan_data.size() * 100) / 64;
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (self->scan_state_ != SCANNING) break;
      }
      if (self->scan_state_ == SCANNING) {
        self->scan_state_ = SCAN_IDLE;
        self->scan_step_ = "Complete";
        self->scan_progress_ = 100;
        // Store session (simplified)
        ScanSession s;
        s.id = self->active_scan_id_;
        s.ts = millis() / 1000;
        s.trials = 1;
        s.duration_sec = (millis() - self->scan_start_ts_) / 1000;
        s.size_bytes = scan_data.size() * 2;
        self->sessions_.push_back(s);
        self->scan_results_[self->active_scan_id_] = std::move(scan_data);
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
