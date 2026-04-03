#pragma once

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
    .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:16px;flex:1 1 300px;min-width:300px;margin-bottom:16px}
    .card h2{margin:0 0 10px 0;font-size:14px;font-weight:600;color:var(--acc)}
    .btns{display:flex;gap:8px;flex-wrap:wrap}
    button,.btn{appearance:none;border:1px solid var(--line);background:#0f1520;color:var(--text);padding:8px 12px;border-radius:10px;font-weight:600;cursor:pointer}
    @media (prefers-color-scheme: light){ button,.btn{background:#f6f8fa;color:var(--text)} }
    button:hover,.btn:hover{border-color:#39475e}
    .btn.primary{background:var(--acc);color:#fff;border-color:#3e7cc8}
    .btn.ghost{background:transparent}
    .kv{display:grid;grid-template-columns:180px 1fr;gap:6px 12px;align-items:center}
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
    .tabs{display:flex;gap:4px;border-bottom:1px solid var(--line);margin-bottom:16px}
    .tab{padding:8px 16px;cursor:pointer;font-weight:600;color:var(--muted)}
    .tab.active{color:var(--acc);border-bottom:2px solid var(--acc)}
    .tab-content{display:none}
    .tab-content.active{display:block}
    input[type="number"],input[type="text"],select{background:#0f1520;color:var(--text);border:1px solid var(--line);padding:6px 8px;border-radius:6px}
    .switch{position:relative;display:inline-block;width:40px;height:20px}
    .switch input{opacity:0;width:0;height:0}
    .slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#ccc;transition:.4s;border-radius:20px}
    .slider:before{position:absolute;content:"";height:16px;width:16px;left:2px;bottom:2px;background-color:white;transition:.4s;border-radius:50%}
    input:checked + .slider{background-color:var(--acc)}
    input:checked + .slider:before{transform:translateX(20px)}
  </style>
</head>
<body>
  <div class="wrap">
    <div id="errorBanner" class="banner"></div>
    <div class="hdr">
      <h1>Roode Control Portal</h1>
      <div class="status"><span id="statusText">Status: <b>Idle</b></span> · <span id="lastCal">Last calibration: —</span></div>
    </div>

    <div class="tabs">
      <div class="tab active" data-tab="calibration">Calibration</div>
      <div class="tab" data-tab="settings">Settings</div>
      <div class="tab" data-tab="sensors">Sensors</div>
    </div>

    <!-- Calibration Tab -->
    <div id="calibration" class="tab-content active">
      <div class="row">
        <div class="card" style="flex:1 1 100%">
          <h2>Calibration Controls</h2>
          <div class="btns">
            <button class="btn primary" id="btnStart">Start Calibration Scan</button>
            <button class="btn" id="btnCancel" style="display:none">Cancel Scan</button>
            <div style="display:flex;align-items:center;gap:8px;margin-left:auto">
              <span>Person in Frame:</span>
              <label class="switch">
                <input type="checkbox" id="presenceToggle">
                <span class="slider"></span>
              </label>
            </div>
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
          <h2>Current ROI & Thresholds</h2>
          <div class="kv" id="currentSettings">
            <div class="k">ROI Size</div><div id="curRoiSize">—</div>
            <div class="k">ROI Center</div><div id="curRoiCenter">—</div>
            <div class="k">Entry Center</div><div id="curEntry">—</div>
            <div class="k">Exit Center</div><div id="curExit">—</div>
            <div class="k">Min Threshold</div><div id="curMin">—</div>
            <div class="k">Max Threshold</div><div id="curMax">—</div>
          </div>
        </div>

        <div class="card">
          <h2>Recommended ROI</h2>
          <div class="kv" id="preview">
            <div class="k">ROI Center</div><div id="pvRoi">—</div>
            <div class="k">Delta (Contrast)</div><div id="pvDelta">—</div>
          </div>
          <div class="chips">
            <button class="btn" id="btnApply" disabled>Apply ROI</button>
          </div>
        </div>

        <div class="card" style="flex:1 1 100%">
          <h2>Past Sessions</h2>
          <div style="overflow:auto">
            <table id="tblSessions">
              <thead>
                <tr>
                  <th>#</th><th>Date/Time</th><th>Session ID</th><th>Lux</th><th>Actions</th>
                </tr>
              </thead>
              <tbody>
                <tr><td colspan="5" class="muted">No sessions yet.</td></tr>
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </div>

    <!-- Settings Tab -->
    <div id="settings" class="tab-content">
      <div class="card">
        <h2>General Settings</h2>
        <div class="kv">
          <div class="k">Sampling Size</div>
          <input type="number" id="setSampling" min="1" max="100">
          
          <div class="k">Polling Interval (ms)</div>
          <input type="number" id="setPolling" min="1" max="1000">
          
          <div class="k">Invert Direction</div>
          <label class="switch"><input type="checkbox" id="setInvert"><span class="slider"></span></label>
          
          <div class="k">Filter Mode</div>
          <select id="setFilterMode">
            <option value="0">Min</option>
            <option value="1">Median</option>
            <option value="2">Percentile 10</option>
          </select>
          
          <div class="k">Filter Window</div>
          <input type="number" id="setFilterWindow" min="1" max="255">

          <div class="k">Debug Mode</div>
          <label class="switch"><input type="checkbox" id="setDebug"><span class="slider"></span></label>
        </div>
      </div>

      <div class="card">
        <h2>Thresholds & Calibration</h2>
        <div class="kv">
          <div class="k">Min Threshold (%)</div>
          <input type="number" id="setMinThres" min="0" max="100">
          
          <div class="k">Max Threshold (%)</div>
          <input type="number" id="setMaxThres" min="0" max="100">
          
          <div class="k">Auto-Calibration Interval (s)</div>
          <input type="number" id="setAutoCal" min="0">
          
          <div class="k">Invalid Distance Limit</div>
          <input type="number" id="setInvalidLimit" min="1" max="255">
          
          <div class="k">Restart Timeout (ms)</div>
          <input type="number" id="setRestartTimeout" min="0">
        </div>
      </div>

      <div class="card">
        <h2>Lux & Sun Settings</h2>
        <div class="kv">
          <div class="k">Lux Threshold</div>
          <input type="number" id="setLuxThres" step="0.1">
          
          <div class="k">Sun Elevation Enable</div>
          <label class="switch"><input type="checkbox" id="setSunEnable"><span class="slider"></span></label>
          
          <div class="k">Sun Elevation Threshold</div>
          <input type="number" id="setSunThres" step="0.1">
        </div>
      </div>

      <div class="btns">
        <button class="btn primary" id="btnSaveSettings">Save All Settings</button>
      </div>
    </div>

    <!-- Sensors Tab -->
    <div id="sensors" class="tab-content">
      <div class="card">
        <h2>Enable/Disable Sensors</h2>
        <div id="sensorList" class="kv">
          <!-- Populated by JS -->
        </div>
        <div class="hr"></div>
        <button class="btn primary" id="btnSaveSensors">Update Sensor Visibility</button>
      </div>
    </div>

  </div>

  <script>
  const TOKEN = new URLSearchParams(location.search).get('token') || '';
  const TOKEN_HEADER = TOKEN ? {'X-Portal-Token': TOKEN} : {};
  const tokenParam = TOKEN ? `?token=${encodeURIComponent(TOKEN)}` : '';
  const $ = (s)=>document.querySelector(s);
  
  const SENSOR_MAP = [
    {id: 0x01, n: "Distance Entry"},
    {id: 0x02, n: "Distance Exit"},
    {id: 0x04, n: "Presence Binary"},
    {id: 0x08, n: "People Counter"},
    {id: 0x10, n: "RAM Free"},
    {id: 0x20, n: "Flash Free"},
    {id: 0x40, n: "Loop Time"},
    {id: 0x80, n: "CPU Usage"},
    {id: 0x100, n: "Status Sensor"},
    {id: 0x200, n: "Version Text"},
    {id: 0x400, n: "Event Text"},
    {id: 0x800, n: "Interrupt Status"},
    {id: 0x1000, n: "Manual Adjustment"}
  ];

  // Tab Switching
  document.querySelectorAll('.tab').forEach(t => {
    t.onclick = () => {
      document.querySelectorAll('.tab, .tab-content').forEach(el => el.classList.remove('active'));
      t.classList.add('active');
      $('#' + t.dataset.tab).classList.add('active');
    };
  });

  async function getJSON(url){
    try {
      const r = await fetch(url, {cache:'no-store', headers: TOKEN_HEADER});
      return r.ok ? await r.json() : null;
    } catch(e) { return null; }
  }
  async function postJSON(url, body){
    try {
      const headers = {'Content-Type':'application/json', ...TOKEN_HEADER};
      const r = await fetch(url, {method:'POST', headers, body: JSON.stringify(body || {})});
      return r.ok ? await r.json() : {ok:false};
    } catch(e) { return {ok:false}; }
  }

  let config = {};
  async function poll(){
    const [cur, stat, prev] = await Promise.all([
      getJSON('/api/settings/current'),
      getJSON('/api/scan/status'),
      getJSON('/api/roi/preview')
    ]);
    if(cur) {
      config = cur;
      $('#curRoiSize').textContent = `${cur.rw} × ${cur.rh}`;
      $('#curRoiCenter').textContent = cur.rc;
      $('#curEntry').textContent = cur.ec;
      $('#curExit').textContent = cur.xc;
      $('#curMin').textContent = cur.min + '%';
      $('#curMax').textContent = cur.max + '%';
      $('#presenceToggle').checked = cur.pres;
      
      // Update Settings Tab if it's the first load
      if(!window._loaded) {
        $('#setSampling').value = cur.sa;
        $('#setPolling').value = cur.pi;
        $('#setInvert').checked = cur.inv;
        $('#setFilterMode').value = cur.fm;
        $('#setFilterWindow').value = cur.fw;
        $('#setDebug').checked = cur.debug;
        $('#setMinThres').value = cur.min;
        $('#setMaxThres').value = cur.max;
        $('#setAutoCal').value = cur.ac;
        $('#setInvalidLimit').value = cur.il;
        $('#setRestartTimeout').value = cur.rt;
        $('#setLuxThres').value = cur.lt;
        $('#setSunEnable').checked = cur.se;
        $('#setSunThres').value = cur.st;
        
        const sl = $('#sensorList');
        sl.innerHTML = '';
        SENSOR_MAP.forEach(s => {
          const k = document.createElement('div'); k.className='k'; k.textContent = s.n;
          const v = document.createElement('div');
          v.innerHTML = `<label class="switch"><input type="checkbox" class="sensor-opt" data-id="${s.id}" ${cur.m & s.id ? 'checked' : ''}><span class="slider"></span></label>`;
          sl.append(k, v);
        });
        window._loaded = true;
      }
    }
    if(stat) {
      $('#statusText').innerHTML = `Status: <b>${stat.s}</b>`;
      $('#progressBar').style.width = stat.p + '%';
      $('#btnCancel').style.display = stat.s === 'Scanning' ? 'inline-block' : 'none';
    }
    if(prev) {
      $('#pvRoi').textContent = prev.roi;
      $('#pvDelta').textContent = prev.con;
      $('#btnApply').disabled = false;
    }
  }

  $('#presenceToggle').onchange = () => postJSON('/api/presence/toggle', {state: $('#presenceToggle').checked});
  $('#btnStart').onclick = () => postJSON('/api/scan/start');
  $('#btnCancel').onclick = () => postJSON('/api/scan/cancel');
  $('#btnApply').onclick = () => postJSON('/api/roi/apply');
  
  $('#btnSaveSettings').onclick = () => {
    const body = {
      sa: parseInt($('#setSampling').value),
      pi: parseInt($('#setPolling').value),
      inv: $('#setInvert').checked,
      fm: parseInt($('#setFilterMode').value),
      fw: parseInt($('#setFilterWindow').value),
      debug: $('#setDebug').checked,
      min: parseInt($('#setMinThres').value),
      max: parseInt($('#setMaxThres').value),
      ac: parseInt($('#setAutoCal').value),
      il: parseInt($('#setInvalidLimit').value),
      rt: parseInt($('#setRestartTimeout').value),
      lt: parseFloat($('#setLuxThres').value),
      se: $('#setSunEnable').checked,
      st: parseFloat($('#setSunThres').value)
    };
    postJSON('/api/settings/update', body).then(r => r.ok && alert('Settings Saved'));
  };

  $('#btnSaveSensors').onclick = () => {
    let mask = 0;
    document.querySelectorAll('.sensor-opt').forEach(el => {
      if(el.checked) mask |= parseInt(el.dataset.id);
    });
    postJSON('/api/settings/update', {m: mask}).then(r => r.ok && alert('Sensors Updated'));
  };

  setInterval(poll, 2000);
  poll();
  </script>
</body>
</html>
)PORTAL";
