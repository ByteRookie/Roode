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
    body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 system-ui,-apple-system,sans-serif}
    .wrap{max-width:1000px;margin:0 auto;padding:20px}
    .hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:20px}
    .hdr h1{font-size:20px;margin:0}
    .tabs{display:flex;gap:4px;border-bottom:1px solid var(--line);margin-bottom:20px}
    .tab{padding:10px 20px;cursor:pointer;font-weight:600;color:var(--muted);border-bottom:2px solid transparent}
    .tab.active{color:var(--acc);border-bottom-color:var(--acc)}
    .tab-content{display:none}
    .tab-content.active{display:block}
    .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:20px;margin-bottom:16px}
    .card h2{margin:0 0 16px 0;font-size:14px;text-transform:uppercase;color:var(--acc);letter-spacing:0.5px}
    .grid{display:grid;grid-template-columns:repeat(auto-fit, minmax(300px, 1fr));gap:16px}
    .kv{display:grid;grid-template-columns:180px 1fr;gap:8px 12px;align-items:center;margin-bottom:8px}
    .kv .k{color:var(--muted)}
    .v{font-family:monospace;font-weight:600}
    .btns{display:flex;gap:10px;margin-top:16px}
    button,.btn{background:#1a212e;color:#fff;border:1px solid var(--line);padding:8px 16px;border-radius:8px;font-weight:600;cursor:pointer}
    .btn.primary{background:var(--acc);border-color:transparent}
    .progress{height:8px;background:#0f1520;border-radius:4px;overflow:hidden;margin:12px 0}
    #progressBar{height:100%;background:var(--acc);width:0%;transition:width 0.2s}
    table{width:100%;border-collapse:collapse}
    th,td{text-align:left;padding:10px 8px;border-bottom:1px solid var(--line)}
    th{font-size:12px;color:var(--muted);text-transform:uppercase}
    input,select{background:#0f1520;color:#fff;border:1px solid var(--line);padding:6px 10px;border-radius:6px;width:100%}
    .switch{position:relative;display:inline-block;width:36px;height:20px}
    .switch input{opacity:0;width:0;height:0}
    .slider{position:absolute;top:0;left:0;right:0;bottom:0;background-color:#2d3748;transition:.3s;border-radius:20px;cursor:pointer}
    .slider:before{position:absolute;content:"";height:14px;width:14px;left:3px;bottom:3px;background-color:white;transition:.3s;border-radius:50%}
    input:checked + .slider{background-color:var(--acc)}
    input:checked + .slider:before{transform:translateX(16px)}
    .tag{font-size:10px;padding:2px 6px;border-radius:4px;background:var(--line);color:var(--muted);font-weight:700}
    .tag.ok{color:var(--ok);background:rgba(91,209,155,0.1)}
    .sensor-item{display:flex;align-items:center;justify-content:space-between;padding:8px 0;border-bottom:1px solid var(--line)}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hdr">
      <h1>Roode Control</h1>
      <div class="status" style="display:flex;align-items:center;gap:16px">
        <span id="statusText">Status: <b>Idle</b></span>
        <span id="luxDisplay" style="display:none"> · Lux: <b id="luxValue">—</b></span>
        <span style="display:flex;align-items:center;gap:8px;font-size:13px">
          <span style="color:var(--muted)">Counting</span>
          <label class="switch"><input type="checkbox" id="masterEnable" checked><span class="slider"></span></label>
        </span>
      </div>
    </div>

    <div class="tabs">
      <div class="tab active" data-tab="calibration">Calibration</div>
      <div class="tab" data-tab="settings">Config</div>
      <div class="tab" data-tab="sensors">Sensors</div>
      <div class="tab" data-tab="system">System</div>
    </div>

    <!-- Calibration Tab -->
    <div id="calibration" class="tab-content active">
      <div class="card">
        <h2>Manual Calibration</h2>
        <p style="color:var(--muted);font-size:13px">Perform a two-step scan to optimize the Region of Interest (ROI).</p>
        <div class="btns">
          <button id="btnBgScan" class="btn primary">1. Scan Background</button>
          <button id="btnPersonScan" class="btn primary">2. Scan Person</button>
          <button id="btnCancel" class="btn" style="display:none">Cancel</button>
        </div>
        <div class="progress"><div id="progressBar"></div></div>
        <div class="grid" style="margin-top:16px">
          <div class="kv"><div class="k">Progress</div><div id="progressText" class="v">0%</div></div>
          <div class="kv"><div class="k">Recommendation</div><div id="pvRoi" class="v">—</div></div>
        </div>
        <div class="btns">
          <button id="btnApply" class="btn primary" disabled>Apply Recommended ROI</button>
        </div>
      </div>

      <div class="card">
        <h2>Session History</h2>
        <div style="overflow:auto">
          <table id="tblSessions">
            <thead>
              <tr><th>Date/Time</th><th>BG Lux</th><th>P Lux</th><th>Status</th><th>Action</th></tr>
            </thead>
            <tbody><tr><td colspan="5" class="muted">No sessions yet.</td></tr></tbody>
          </table>
        </div>
      </div>
    </div>

    <!-- Config Tab -->
    <div id="settings" class="tab-content">
      <div class="grid">
        <div class="card">
          <h2>Roode Core</h2>
          <div class="kv"><div class="k">Sampling</div><input type="number" id="setSampling" min="1" max="10"></div>
          <div class="kv"><div class="k">Orientation</div>
            <select id="setOrientation"><option value="0">Parallel</option><option value="1">Perpendicular</option></select>
          </div>
          <div class="kv"><div class="k">Invert Zones</div><label class="switch"><input type="checkbox" id="setInvert"><span class="slider"></span></label></div>
          <div class="kv"><div class="k">Persistence</div><label class="switch"><input type="checkbox" id="setPersist"><span class="slider"></span></label></div>
        </div>

        <div class="card">
          <h2>Filtering</h2>
          <div class="kv"><div class="k">Filter Mode</div>
            <select id="setFilterMode">
              <option value="0">Min (Fastest)</option>
              <option value="1">Median</option>
              <option value="2">Percentile 10</option>
            </select>
          </div>
          <div class="kv"><div class="k">Window Size</div><input type="number" id="setFilterWindow" min="1" max="10"></div>
        </div>

        <div class="card">
          <h2>Entry Zone ROI</h2>
          <p style="color:var(--muted);font-size:12px">Region of Interest the sensor reads for the entry zone. Width/height 4–16, center 0–255.</p>
          <div class="kv"><div class="k">Width (cols)</div><input type="number" id="setErw" min="4" max="16"></div>
          <div class="kv"><div class="k">Height (rows)</div><input type="number" id="setErh" min="4" max="16"></div>
          <div class="kv"><div class="k">Center (SPAD)</div><input type="number" id="setErc" min="0" max="255"></div>
          <div class="kv"><div class="k">Min threshold (mm)</div><input type="number" id="setEmin" min="0"></div>
          <div class="kv"><div class="k">Max threshold (mm)</div><input type="number" id="setEmax" min="0"></div>
        </div>

        <div class="card">
          <h2>Exit Zone ROI</h2>
          <p style="color:var(--muted);font-size:12px">Region of Interest the sensor reads for the exit zone.</p>
          <div class="kv"><div class="k">Width (cols)</div><input type="number" id="setXrw" min="4" max="16"></div>
          <div class="kv"><div class="k">Height (rows)</div><input type="number" id="setXrh" min="4" max="16"></div>
          <div class="kv"><div class="k">Center (SPAD)</div><input type="number" id="setXrc" min="0" max="255"></div>
          <div class="kv"><div class="k">Min threshold (mm)</div><input type="number" id="setXmin" min="0"></div>
          <div class="kv"><div class="k">Max threshold (mm)</div><input type="number" id="setXmax" min="0"></div>
          <div class="btns" style="margin-top:12px">
            <button id="btnRecal" class="btn">Re-run Threshold Calibration</button>
          </div>
        </div>

        <div class="card">
          <h2>Environment</h2>
          <div class="kv" id="luxRow" style="display:none"><div class="k">Use Lux Automation</div><label class="switch"><input type="checkbox" id="setUseLux"><span class="slider"></span></label></div>
          <div class="kv"><div class="k">Use Sun (Rise/Set)</div><label class="switch"><input type="checkbox" id="setUseSun"><span class="slider"></span></label></div>
        </div>

        <div class="card">
          <h2>VL53L1X Hardware</h2>
          <div class="kv"><div class="k">Sensor ID</div><input type="number" id="setSid"></div>
          <div class="kv"><div class="k">I2C Address</div><input type="text" id="setAddr"></div>
          <div class="kv"><div class="k">Timeout (ms)</div><input type="number" id="setTO"></div>
          <div class="kv"><div class="k">Ranging Mode</div>
            <select id="setRM">
              <option value="0">Auto</option><option value="1">Shortest</option><option value="2">Short</option>
              <option value="3">Medium</option><option value="4">Long</option><option value="5">Longer</option>
              <option value="6">Longest</option>
            </select>
          </div>
        </div>
      </div>
      <div class="btns"><button id="btnSaveSettings" class="btn primary">Save & Apply All Settings</button></div>
    </div>

    <!-- Sensors Tab -->
    <div id="sensors" class="tab-content">
      <div class="card">
        <h2>Sensor Visibility</h2>
        <p style="color:var(--muted);font-size:13px">Toggle which entities are enabled and published to Home Assistant.</p>
        <div style="display:flex;align-items:center;gap:12px;margin-bottom:16px;padding-bottom:16px;border-bottom:1px solid var(--line)">
          <span style="font-weight:600">All Sensors</span>
          <div style="display:flex;gap:8px;margin-left:auto">
            <button id="btnEnableAll" class="btn">Enable All</button>
            <button id="btnDisableAll" class="btn">Disable All</button>
          </div>
        </div>
        <div id="sensorList"></div>
        <div class="btns"><button id="btnSaveSensors" class="btn primary">Save Visibility</button></div>
      </div>
    </div>

    <!-- System Tab -->
    <div id="system" class="tab-content">
      <div class="grid">
        <div class="card">
          <h2>Active Configuration</h2>
          <div id="activeState"></div>
        </div>
        <div class="card">
          <h2>Diagnostics</h2>
          <div class="kv"><div class="k">Debug Logging</div><label class="switch"><input type="checkbox" id="setDebug"><span class="slider"></span></label></div>
          <div class="kv"><div class="k">Single Core Only</div><label class="switch"><input type="checkbox" id="setSingleCore"><span class="slider"></span></label></div>
          <div class="kv"><div class="k">Invalid Limit</div><input type="number" id="setInvalidLimit"></div>
          <div class="kv"><div class="k">Restart TO (ms)</div><input type="number" id="setRestartTO"></div>
          <div class="btns" style="margin-top:12px">
            <button id="btnSaveDiag" class="btn primary">Save Diagnostics</button>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
  const TOKEN = new URLSearchParams(location.search).get('token') || '';
  const TOKEN_HEADER = TOKEN ? {'X-Portal-Token': TOKEN} : {};
  const $ = (s)=>document.querySelector(s);
  
  const SENSOR_MAP = [
    {id: 0x01, n: "Distance Entry"}, {id: 0x02, n: "Distance Exit"}, {id: 0x04, n: "Presence Binary"},
    {id: 0x08, n: "People Counter"}, {id: 0x10, n: "RAM Free"}, {id: 0x20, n: "Flash Free"},
    {id: 0x40, n: "Loop Time"}, {id: 0x80, n: "CPU Usage"}, {id: 0x100, n: "Sensor Status (Num)"},
    {id: 0x200, n: "FW Version"}, {id: 0x400, n: "Direction Event"}, {id: 0x800, n: "Interrupt Status"},
    {id: 0x1000, n: "Manual Adjust"}, {id: 0x2000, n: "Sensor Status (Text)"}, {id: 0x4000, n: "Enabled Features"},
    {id: 0x8000, n: "Entry Thresholds"}, {id: 0x10000, n: "Exit Thresholds"}, {id: 0x20000, n: "ROI Sensors"}
  ];

  document.querySelectorAll('.tab').forEach(t => {
    t.onclick = () => {
      document.querySelectorAll('.tab, .tab-content').forEach(el => el.classList.remove('active'));
      t.classList.add('active');
      $('#' + t.dataset.tab).classList.add('active');
    };
  });

  async function api(url, method='GET', body=null){
    try {
      const opts = {method, headers: {...TOKEN_HEADER}};
      if(body) { opts.headers['Content-Type']='application/json'; opts.body = JSON.stringify(body); }
      const r = await fetch(url, opts);
      return r.ok ? await r.json() : null;
    } catch(e) { return null; }
  }

  async function poll(){
    const [cur, stat, prev, hist] = await Promise.all([
      api('/api/settings/current'), api('/api/scan/status'), api('/api/roi/preview'), api('/api/scan/sessions')
    ]);
    if(cur) {
      if(cur.lux_val !== undefined) {
        $('#luxDisplay').style.display = 'inline';
        $('#luxValue').textContent = cur.lux_val + ' lx';
        $('#luxRow').style.display = 'grid';
      }
      // Always refresh values (not just once) so portal reflects live state
      $('#setSampling').value = cur.sa; $('#setOrientation').value = cur.or;
      $('#setInvert').checked = cur.inv; $('#setPersist').checked = cur.cp;
      $('#setFilterMode').value = cur.fm; $('#setFilterWindow').value = cur.fw;
      $('#setUseLux').checked = cur.ul; $('#setUseSun').checked = cur.us;
      $('#setSid').value = cur.sid; $('#setAddr').value = '0x'+cur.addr.toString(16);
      $('#setTO').value = cur.to; $('#setRM').value = cur.rm;
      $('#setDebug').checked = cur.debug; $('#setSingleCore').checked = cur.sc;
      $('#setInvalidLimit').value = cur.il; $('#setRestartTO').value = cur.rt;
      // Zone ROI & thresholds
      $('#setErw').value = cur.erw; $('#setErh').value = cur.erh; $('#setErc').value = cur.erc;
      $('#setEmin').value = cur.emin; $('#setEmax').value = cur.emax;
      $('#setXrw').value = cur.xrw; $('#setXrh').value = cur.xrh; $('#setXrc').value = cur.xrc;
      $('#setXmin').value = cur.xmin; $('#setXmax').value = cur.xmax;

      if(cur.ce !== undefined) $('#masterEnable').checked = cur.ce;
      if(!window._loaded) {
        const sl = $('#sensorList'); sl.innerHTML = '';
        SENSOR_MAP.forEach(s => {
          const div = document.createElement('div'); div.className='sensor-item';
          div.innerHTML = `<span>${s.n}</span><label class="switch"><input type="checkbox" class="sensor-opt" data-id="${s.id}" ${cur.m & s.id ? 'checked' : ''}><span class="slider"></span></label>`;
          sl.append(div);
        });
        window._loaded = true;
      }

      const as = $('#activeState'); as.innerHTML = '';
      const fields = [
        ['Entry ROI', `${cur.erh}h × ${cur.erw}w @ center ${cur.erc}`],
        ['Entry Threshold', `${cur.emin} – ${cur.emax} mm`],
        ['Exit ROI', `${cur.xrh}h × ${cur.xrw}w @ center ${cur.xrc}`],
        ['Exit Threshold', `${cur.xmin} – ${cur.xmax} mm`],
      ];
      fields.forEach(f => {
        const d = document.createElement('div'); d.className='kv';
        d.innerHTML = `<div class="k">${f[0]}</div><div class="v">${f[1]}</div>`;
        as.append(d);
      });
    }
    if(stat) {
      $('#statusText').innerHTML = `Sensor: <b>${stat.s}</b>`;
      $('#progressBar').style.width = stat.p + '%';
      $('#progressText').textContent = stat.p + '%';
      const busy = stat.s === 'Scanning';
      $('#btnBgScan').disabled = busy; $('#btnPersonScan').disabled = busy;
      $('#btnCancel').style.display = busy ? 'inline-block' : 'none';
    }
    if(prev) {
      $('#pvRoi').textContent = 'Center ' + prev.roi + ' (Contrast ' + prev.con + 'mm)';
      $('#btnApply').disabled = false;
    }
    if(hist) {
      const tbody = $('#tblSessions tbody'); tbody.innerHTML = '';
      if(!hist.length) tbody.innerHTML = '<tr><td colspan="5" class="muted">No history sessions.</td></tr>';
      hist.forEach(s => {
        const tr = document.createElement('tr');
        const st = s.complete ? '<span class="tag ok">Complete</span>' : '<span class="tag">Partial</span>';
        tr.innerHTML = `<td>${s.date}</td><td>${s.bg_lux} lx</td><td>${s.p_lux||'—'} lx</td><td>${st}</td><td><button class="btn" style="padding:2px 8px" onclick="delSession('${s.id}')">Del</button></td>`;
        tbody.append(tr);
      });
    }
  }

  window.delSession = (id) => api('/api/scan/delete?id='+id, 'POST').then(poll);
  $('#btnBgScan').onclick = () => api('/api/scan/start?phase=1', 'POST');
  $('#btnPersonScan').onclick = () => api('/api/scan/start?phase=2', 'POST');
  $('#btnCancel').onclick = () => api('/api/scan/cancel', 'POST');
  $('#btnApply').onclick = () => api('/api/roi/apply', 'POST').then(()=>alert('Applied'));
  
  function boolStr(v){ return v ? 'true' : 'false'; }
  function intVal(id){ return parseInt(document.getElementById(id).value) || 0; }

  $('#btnSaveSettings').onclick = () => {
    const params = new URLSearchParams({
      sa: intVal('setSampling'), or: intVal('setOrientation'),
      inv: boolStr($('#setInvert').checked), cp: boolStr($('#setPersist').checked),
      fm: intVal('setFilterMode'), fw: intVal('setFilterWindow'),
      ul: boolStr($('#setUseLux').checked), us: boolStr($('#setUseSun').checked),
      sid: intVal('setSid'), to: intVal('setTO'), rm: intVal('setRM'),
      // Zone ROI
      erh: intVal('setErh'), erw: intVal('setErw'), erc: intVal('setErc'),
      emin: intVal('setEmin'), emax: intVal('setEmax'),
      xrh: intVal('setXrh'), xrw: intVal('setXrw'), xrc: intVal('setXrc'),
      xmin: intVal('setXmin'), xmax: intVal('setXmax'),
      // Diagnostics
      debug: boolStr($('#setDebug').checked), sc: boolStr($('#setSingleCore').checked),
      il: intVal('setInvalidLimit'), rt: intVal('setRestartTO')
    }).toString();
    api('/api/settings/update?' + params, 'POST').then(r => { if(r && r.ok) alert('Settings saved'); });
  };

  $('#btnSaveDiag').onclick = () => {
    const params = new URLSearchParams({
      debug: boolStr($('#setDebug').checked), sc: boolStr($('#setSingleCore').checked),
      il: intVal('setInvalidLimit'), rt: intVal('setRestartTO')
    }).toString();
    api('/api/settings/update?' + params, 'POST').then(r => { if(r && r.ok) alert('Diagnostics saved'); });
  };

  $('#btnRecal').onclick = () => {
    if(!confirm('Run threshold calibration now? Keep the sensor view clear (no people).')) return;
    $('#btnRecal').disabled = true;
    api('/api/recalibrate', 'POST').then(r => {
      $('#btnRecal').disabled = false;
      if(r && r.ok) {
        alert(`Calibration done!\nEntry: ${r.emin}–${r.emax} mm (idle ${r.idle_e} mm)\nExit:  ${r.xmin}–${r.xmax} mm (idle ${r.idle_x} mm)`);
      } else {
        alert('Calibration failed — check logs.');
      }
    });
  };

  $('#masterEnable').onchange = function() {
    api('/api/settings/update?ce=' + boolStr(this.checked), 'POST');
  };
  $('#btnEnableAll').onclick = () => {
    document.querySelectorAll('.sensor-opt').forEach(el => { el.checked = true; });
  };
  $('#btnDisableAll').onclick = () => {
    document.querySelectorAll('.sensor-opt').forEach(el => { el.checked = false; });
  };
  $('#btnSaveSensors').onclick = () => {
    let m = 0; document.querySelectorAll('.sensor-opt').forEach(el => { if(el.checked) m |= parseInt(el.dataset.id); });
    api('/api/settings/update?m=' + m, 'POST').then(r => { if(r && r.ok) alert('Visibility updated'); });
  };

  setInterval(poll, 2000); poll();
  </script>
</body>
</html>
)PORTAL";
