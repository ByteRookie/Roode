#pragma once

static const char *const portal_html = R"PORTAL(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Roode Control Portal</title>
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
    .wrap{max-width:1100px;margin:24px auto;padding:0 16px}
    .hdr{display:flex;gap:16px;align-items:center;justify-content:space-between;margin-bottom:12px}
    .hdr h1{font-size:20px;margin:0;font-weight:600}
    .status{font-size:12px;color:var(--muted)}
    .banner{display:none;background:var(--err);color:#fff;padding:8px 12px;border-radius:8px;margin-bottom:12px}
    button[disabled],.disabled{opacity:0.5;pointer-events:none;cursor:not-allowed}
    .grid{display:grid;grid-template-columns:repeat(auto-fit, minmax(340px, 1fr));gap:16px}
    .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:20px;display:flex;flex-direction:column}
    .card.full{grid-column:1/-1}
    .card h2{margin:0 0 16px 0;font-size:15px;font-weight:600;color:var(--acc);text-transform:uppercase;letter-spacing:0.5px}
    .btns{display:flex;gap:10px;flex-wrap:wrap}
    button,.btn{appearance:none;border:1px solid var(--line);background:#1a212e;color:var(--text);padding:10px 16px;border-radius:8px;font-weight:600;cursor:pointer;transition:all 0.2s}
    @media (prefers-color-scheme: light){ button,.btn{background:#f6f8fa;color:var(--text)} }
    button:hover:not([disabled]),.btn:hover:not([disabled]){border-color:var(--acc);background:#232d3d}
    .btn.primary{background:var(--acc);color:#fff;border-color:transparent}
    .btn.primary:hover:not([disabled]){background:#4095ff}
    .btn.ghost{background:transparent;border-color:transparent;color:var(--muted)}
    .btn.ghost:hover{color:var(--text)}
    .kv{display:grid;grid-template-columns:160px 1fr;gap:8px 12px;align-items:center}
    .kv .k{color:var(--muted);font-weight:500}
    .kv .v{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:13px}
    .progress{height:8px;border-radius:4px;background:#0f1520;overflow:hidden;margin:12px 0}
    .progress>div{height:100%;background:var(--acc);width:0%;transition:width 0.3s}
    table{width:100%;border-collapse:separate;border-spacing:0}
    th,td{padding:12px 8px;border-bottom:1px solid var(--line);text-align:left}
    th{color:var(--muted);font-weight:600;font-size:12px;text-transform:uppercase}
    .hr{height:1px;background:var(--line);margin:16px 0}
    input[type="number"],select{background:#0f1520;color:var(--text);border:1px solid var(--line);padding:8px 10px;border-radius:6px;width:100%}
    .switch{position:relative;display:inline-block;width:36px;height:20px}
    .switch input{opacity:0;width:0;height:0}
    .slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#2d3748;transition:.3s;border-radius:20px}
    .slider:before{position:absolute;content:"";height:14px;width:14px;left:3px;bottom:3px;background-color:white;transition:.3s;border-radius:50%}
    input:checked + .slider{background-color:var(--acc)}
    input:checked + .slider:before{transform:translateX(16px)}
    .section-title{font-size:13px;font-weight:700;margin:20px 0 10px 0;color:var(--muted);border-bottom:1px solid var(--line);padding-bottom:4px}
    .sensor-grid{display:grid;grid-template-columns:repeat(auto-fill, minmax(140px, 1fr));gap:12px}
    .sensor-item{display:flex;align-items:center;gap:8px;background:#1a212e;padding:8px 12px;border-radius:8px;border:1px solid var(--line)}
    .tag{font-size:10px;padding:2px 6px;border-radius:4px;font-weight:700;background:var(--line);color:var(--muted)}
    .tag.ok{background:rgba(91,209,155,0.1);color:var(--ok)}
  </style>
</head>
<body>
  <div class="wrap">
    <div id="errorBanner" class="banner"></div>
    <div class="hdr">
      <h1>Roode Control Portal</h1>
      <div class="status"><span id="statusText">Sensor: <b>Idle</b></span> · <span id="syncText">HA Sync: —</span></div>
    </div>

    <div class="grid">
      <!-- Calibration Card -->
      <div class="card full">
        <h2>Calibration Workflow</h2>
        <p style="margin-top:0;color:var(--muted);font-size:13px">Follow the steps below to find the best ROI. First scan the empty frame, then stand in the frame and scan again.</p>
        <div class="btns">
          <button class="btn primary" id="btnBgScan">1. Scan Background (Empty)</button>
          <button class="btn primary" id="btnPersonScan">2. Scan Person (In Frame)</button>
          <button class="btn" id="btnCancel" style="display:none">Stop Scan</button>
          <button class="btn" id="btnApply" disabled>Apply Recommended ROI</button>
        </div>
        <div class="progress"><div id="progressBar"></div></div>
        <div style="display:flex;gap:24px;margin-top:10px">
          <div class="kv" style="flex:1"><div class="k">Step:</div><div id="stepText" class="v">Ready</div></div>
          <div class="kv" style="flex:1"><div class="k">Recommended ROI:</div><div id="pvRoi" class="v">—</div></div>
          <div class="kv" style="flex:1"><div class="k">Max Contrast:</div><div id="pvDelta" class="v">—</div></div>
        </div>
      </div>

      <!-- Current State Card -->
      <div class="card">
        <h2>Active ROI Configuration</h2>
        <div class="kv"><div class="k">ROI Size</div><div id="curRoiSize" class="v">—</div></div>
        <div class="kv"><div class="k">ROI Center</div><div id="curRoiCenter" class="v">—</div></div>
        <div class="kv"><div class="k">Entry Center</div><div id="curEntry" class="v">—</div></div>
        <div class="kv"><div class="k">Exit Center</div><div id="curExit" class="v">—</div></div>
        <div class="hr"></div>
        <div class="kv"><div class="k">Min Threshold</div><div id="curMin" class="v">—</div></div>
        <div class="kv"><div class="k">Max Threshold</div><div id="curMax" class="v">—</div></div>
        <div class="kv"><div class="k">Sampling</div><div id="curSampling" class="v">—</div></div>
        <div class="chips" style="margin-top:auto;padding-top:16px">
          <button class="btn ghost" style="padding:4px 8px;font-size:11px" id="btnCopyJSON">Copy Config JSON</button>
        </div>
      </div>

      <!-- Settings Card -->
      <div class="card">
        <h2>Global Configuration</h2>
        <div class="section-title">Performance</div>
        <div class="kv"><div class="k">Sampling Size</div><input type="number" id="setSampling"></div>
        <div class="kv"><div class="k">Interval (ms)</div><input type="number" id="setPolling"></div>
        
        <div class="section-title">Behavior</div>
        <div class="kv"><div class="k">Invert Direction</div><label class="switch"><input type="checkbox" id="setInvert"><span class="slider"></span></label></div>
        <div class="kv"><div class="k">Debug Logging</div><label class="switch"><input type="checkbox" id="setDebug"><span class="slider"></span></label></div>
        
        <div class="section-title">Filtering</div>
        <div class="kv">
          <div class="k">Algorithm</div>
          <select id="setFilterMode">
            <option value="0">Min (Fastest)</option>
            <option value="1">Median (Stable)</option>
            <option value="2">Percentile 10</option>
          </select>
        </div>
        <div class="kv"><div class="k">Window Size</div><input type="number" id="setFilterWindow"></div>
        
        <button class="btn primary" style="margin-top:20px" id="btnSaveSettings">Update All Settings</button>
      </div>

      <!-- Environmental Card -->
      <div class="card">
        <h2>Environment & Thresholds</h2>
        <div class="section-title">Light (Lux)</div>
        <div class="kv"><div class="k">Min Lux level</div><input type="number" id="setLuxThres" step="0.1"></div>
        
        <div class="section-title">Sun Elevation</div>
        <div class="kv"><div class="k">Enable Check</div><label class="switch"><input type="checkbox" id="setSunEnable"><span class="slider"></span></label></div>
        <div class="kv"><div class="k">Min Elevation</div><input type="number" id="setSunThres" step="0.1"></div>
        
        <div class="section-title">Safety</div>
        <div class="kv"><div class="k">Invalid Limit</div><input type="number" id="setInvalidLimit"></div>
        <div class="kv"><div class="k">Restart Timeout</div><input type="number" id="setRestartTimeout"></div>
        
        <div style="margin-top:auto;color:var(--muted);font-size:11px">Tracking pauses when thresholds are not met.</div>
      </div>

      <!-- History Card -->
      <div class="card full">
        <h2>Calibration Session History</h2>
        <div style="overflow:auto">
          <table id="tblSessions">
            <thead>
              <tr>
                <th>Date / Time</th><th>BG Lux</th><th>Person Lux</th><th>Status</th><th>Actions</th>
              </tr>
            </thead>
            <tbody>
              <tr><td colspan="5" class="muted">Loading history...</td></tr>
            </tbody>
          </table>
        </div>
      </div>

      <!-- Sensors Card -->
      <div class="card full">
        <h2>Active Sensor Entities</h2>
        <p style="margin-top:0;color:var(--muted);font-size:13px;margin-bottom:16px">Toggle which entities are published to Home Assistant / MQTT.</p>
        <div id="sensorList" class="sensor-grid"></div>
        <div class="hr"></div>
        <button class="btn primary" id="btnSaveSensors">Save Visibility Preferences</button>
      </div>
    </div>
  </div>

  <script>
  const TOKEN = new URLSearchParams(location.search).get('token') || '';
  const TOKEN_HEADER = TOKEN ? {'X-Portal-Token': TOKEN} : {};
  const $ = (s)=>document.querySelector(s);
  
  const SENSOR_MAP = [
    {id: 0x01, n: "Dist Entry"}, {id: 0x02, n: "Dist Exit"}, {id: 0x04, n: "Presence"}, {id: 0x08, n: "Counter"},
    {id: 0x10, n: "RAM Metrics"}, {id: 0x20, n: "Flash Metrics"}, {id: 0x40, n: "Loop Time"}, {id: 0x80, n: "CPU Load"},
    {id: 0x100, n: "Sensor Status"}, {id: 0x200, n: "FW Version"}, {id: 0x400, n: "Events"}, {id: 0x800, n: "Interrupts"},
    {id: 0x1000, n: "Manual Adj"}
  ];

  async function api(url, method='GET', body=null){
    try {
      const opts = {method, headers: {...TOKEN_HEADER}};
      if(body) { opts.headers['Content-Type']='application/json'; opts.body = JSON.stringify(body); }
      const r = await fetch(url, opts);
      return r.ok ? await r.json() : null;
    } catch(e) { return null; }
  }

  async function poll(){
    const [cur, stat, prev, history] = await Promise.all([
      api('/api/settings/current'), api('/api/scan/status'), api('/api/roi/preview'), api('/api/scan/sessions')
    ]);
    if(cur) {
      $('#syncText').innerHTML = `HA Sync: <span class="tag ${cur.ts?'ok':''}">${cur.ts?'ACTIVE':'NONE'}</span>`;
      $('#curRoiSize').textContent = `${cur.rw} × ${cur.rh}`;
      $('#curRoiCenter').textContent = cur.rc;
      $('#curEntry').textContent = cur.ec;
      $('#curExit').textContent = cur.xc;
      $('#curMin').textContent = cur.min + 'mm';
      $('#curMax').textContent = cur.max + 'mm';
      $('#curSampling').textContent = cur.sa;
      if(!window._loaded) {
        $('#setSampling').value = cur.sa; $('#setPolling').value = cur.pi; $('#setInvert').checked = cur.inv;
        $('#setDebug').checked = cur.debug; $('#setFilterMode').value = cur.fm; $('#setFilterWindow').value = cur.fw;
        $('#setLuxThres').value = cur.lt; $('#setSunEnable').checked = cur.se; $('#setSunThres').value = cur.st;
        $('#setInvalidLimit').value = cur.il; $('#setRestartTimeout').value = cur.rt;
        const sl = $('#sensorList'); sl.innerHTML = '';
        SENSOR_MAP.forEach(s => {
          const div = document.createElement('div'); div.className='sensor-item';
          div.innerHTML = `<label class="switch"><input type="checkbox" class="sensor-opt" data-id="${s.id}" ${cur.m & s.id ? 'checked' : ''}><span class="slider"></span></label> <span style="font-size:12px">${s.n}</span>`;
          sl.append(div);
        });
        window._loaded = true;
      }
    }
    if(stat) {
      $('#statusText').innerHTML = `Sensor: <b>${stat.s}</b>`;
      $('#progressBar').style.width = stat.p + '%';
      const busy = stat.s === 'Scanning';
      $('#btnBgScan').disabled = busy; $('#btnPersonScan').disabled = busy;
      $('#btnCancel').style.display = busy ? 'inline-block' : 'none';
    }
    if(prev) {
      $('#pvRoi').textContent = prev.roi; $('#pvDelta').textContent = prev.con + 'mm';
      $('#btnApply').disabled = false;
    }
    if(history) {
      const tbody = $('#tblSessions tbody'); tbody.innerHTML = '';
      if(!history.length) tbody.innerHTML = '<tr><td colspan="5" class="muted">No history sessions found.</td></tr>';
      history.forEach(s => {
        const tr = document.createElement('tr');
        const status = s.complete ? '<span class="tag ok">Complete</span>' : '<span class="tag">Incomplete</span>';
        tr.innerHTML = `<td>${s.date}</td><td>${s.bg_lux} lx</td><td>${s.p_lux || '—'} lx</td><td>${status}</td><td><button class="btn ghost" style="padding:2px 8px" onclick="delSession('${s.id}')">Delete</button></td>`;
        tbody.append(tr);
      });
    }
  }

  window.delSession = (id) => api('/api/scan/delete?id='+id, 'POST').then(poll);
  $('#btnBgScan').onclick = () => api('/api/scan/start?phase=1', 'POST');
  $('#btnPersonScan').onclick = () => api('/api/scan/start?phase=2', 'POST');
  $('#btnCancel').onclick = () => api('/api/scan/cancel', 'POST');
  $('#btnApply').onclick = () => api('/api/roi/apply', 'POST').then(()=>alert('ROI Applied'));
  $('#btnCopyJSON').onclick = () => { api('/api/settings/current').then(c => navigator.clipboard.writeText(JSON.stringify(c, null, 2))); alert('JSON Copied'); };
  
  $('#btnSaveSettings').onclick = () => {
    const body = {
      sa: parseInt($('#setSampling').value), pi: parseInt($('#setPolling').value), inv: $('#setInvert').checked,
      fm: parseInt($('#setFilterMode').value), fw: parseInt($('#setFilterWindow').value), debug: $('#setDebug').checked,
      lt: parseFloat($('#setLuxThres').value), se: $('#setSunEnable').checked, st: parseFloat($('#setSunThres').value),
      il: parseInt($('#setInvalidLimit').value), rt: parseInt($('#setRestartTimeout').value)
    };
    api('/api/settings/update', 'POST', body).then(r => r && r.ok && alert('Global settings saved and applied.'));
  };

  $('#btnSaveSensors').onclick = () => {
    let mask = 0; document.querySelectorAll('.sensor-opt').forEach(el => { if(el.checked) mask |= parseInt(el.dataset.id); });
    api('/api/settings/update', 'POST', {m: mask}).then(r => r && r.ok && alert('Sensor entity visibility updated.'));
  };

  setInterval(poll, 2000); poll();
  </script>
</body>
</html>
)PORTAL";
