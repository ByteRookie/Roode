#pragma once
#include <pgmspace.h>

inline const char portal_html[] PROGMEM = R"PORTAL(
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
          <div class="k">Time Remaining</div><div id="timeRemaining">—</div>
          <div class="k">Step</div><div id="stepText">—</div>
          <div class="k">Progress</div>
          <div>
            <div class="progress"><div id="progressBar" style="width:0%"></div></div>
            <div id="progressPercent" style="margin-top:4px">0%</div>
          </div>
        </div>
        <div class="hr"></div>
        <button class="btn" id="btnLog">Show Log</button>
        <pre id="logBox" class="mono" style="display:none;max-height:200px;overflow:auto;background:#0f1520;border:1px solid var(--line);padding:8px;border-radius:8px;margin-top:8px"></pre>
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
    if(s < 60) return s.toFixed(1) + ' s';
    const m = Math.floor(s/60);
    const sec = Math.round(s%60);
    return `${m}m ${sec}s`;
  };

  // State
  let current = null, preview = null, status = null, sessions = [], logVisible = false;

  // Fetch JSON with graceful fallback
  async function getJSON(url){
    try {
      const r = await fetch(url, {cache:'no-store', headers: TOKEN_HEADER});
      if(!r.ok){
        if(r.status===404) return null;
        throw new Error(r.status + ' ' + r.statusText);
      }
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
    const st = status || {state:'Idle', id:'—', step:'—', progress:0, remaining:0};
    $('#statusText').innerHTML = 'Status: <b>'+st.state+'</b>';
    $('#timeRemaining').textContent = Number.isFinite(st.remaining) ? fmtDur(st.remaining) : '—';
    $('#stepText').textContent = st.step || '—';
    const prog = st.progress || 0;
    $('#progressBar').style.width = prog + '%';
    $('#progressPercent').textContent = prog.toFixed(0) + '%';
    $('#btnCancel').style.display = (st.state==='Scanning') ? 'inline-block' : 'none';
    $('#btnStart').style.display = (st.state==='Scanning') ? 'none' : 'inline-block';
    if (st.last_calibration !== undefined) {
      const txt = st.last_calibration
        ? new Date(st.last_calibration).toLocaleString()
        : 'Not run yet';
      $('#lastCal').textContent = 'Last calibration: ' + txt;
    }
  }
  function renderCurrent(){
    const c = current || {};
    const entry = c.entry || {};
    const exit = c.exit || {};
    const roi = entry.roi || {};
    $('#curRoiSize').textContent = (roi.width && roi.height) ? `${roi.width} × ${roi.height}` : '—';
    $('#curRoiCenter').textContent = roi.center ? `(${roi.center.x}, ${roi.center.y})` : '—';
    $('#curEntry').textContent = entry.roi && entry.roi.center ? `(${entry.roi.center.x}, ${entry.roi.center.y})` : '—';
    $('#curExit').textContent = exit.roi && exit.roi.center ? `(${exit.roi.center.x}, ${exit.roi.center.y})` : '—';
    $('#curMode').textContent = c.ranging_mode || '—';
    const thr = entry.threshold || {};
    $('#curMin').textContent = thr.min!=null ? thr.min+'%' : '—';
    $('#curMax').textContent = thr.max!=null ? thr.max+'%' : '—';
    $('#curSampling').textContent = c.samples != null ? c.samples : '—';
    $('#curFw').textContent = c.firmware || '—';
    const last = c.last_calibration;
    $('#lastCal').textContent = 'Last calibration: ' + (last ? new Date(last).toLocaleString() : 'Not run yet');
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
    const entry = r.entry || {};
    const exit = r.exit || {};
    const roi = entry.roi || {};
    $('#pvRoiSize').textContent = (roi.width && roi.height) ? `${roi.width} × ${roi.height}` : '—';
    $('#pvRoi').textContent = roi.center ? `(${roi.center.x}, ${roi.center.y})` : '—';
    $('#pvEntry').textContent = entry.roi && entry.roi.center ? `(${entry.roi.center.x}, ${entry.roi.center.y})` : '—';
    $('#pvExit').textContent = exit.roi && exit.roi.center ? `(${exit.roi.center.x}, ${exit.roi.center.y})` : '—';
    $('#pvMode').textContent = r.ranging_mode || '—';
    const thr = entry.threshold || {};
    $('#pvMin').textContent = thr.min!=null ? thr.min+'%' : '—';
    $('#pvMax').textContent = thr.max!=null ? thr.max+'%' : '—';
    $('#pvSampling').textContent = r.samples != null ? r.samples : '—';
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

  // Translate numeric scan step into description
  function describeStep(step){
    const grids = ['4×4 Scan','8×8 Scan','16×16 Scan'];
    if(step == null) return '—';
    const idx = Math.floor(step / 15);
    return grids[idx] || 'Analyzing';
  }

  async function fetchLog(){
    if(!status || !status.id || status.id === '—') return;
    const sess = await getJSON(`/api/scan/session/${encodeURIComponent(status.id)}${tokenParam}`).catch(()=>null);
    const txt = (sess && sess.data) ? sess.data : '';
    $('#logBox').textContent = txt;
  }

  // Polling
  async function poll(){
    try {
      const wasScanning = status && status.state === 'Scanning';
      const [cur, stat, list, prev] = await Promise.all([
        getJSON('/api/settings/current').catch(()=>null),
        getJSON('/api/scan/status').catch(()=>null),
        getJSON('/api/scan/sessions').catch(()=>null),
        getJSON('/api/roi/preview').catch(()=>null)
      ]);
      if(cur) current = cur;
      if(stat){
        status = {
          state: stat.running ? 'Scanning' : 'Idle',
          id: stat.session_id || '—',
          step: stat.running ? describeStep(stat.step) : '—',
          progress: stat.running ? (stat.progress || 0) * 100 : 0,
          remaining: stat.time_remaining,
          last_calibration: stat.last_calibration
        };
      } else {
        status = {state:'Idle', id:status?.id || '—', step:'—', progress:0, remaining:0};
      }
      if(wasScanning && status.state !== 'Scanning') showSuccess('Calibration complete');
      if(list) sessions = list.sessions || list;
      preview = prev || null;

      renderStatus(); renderCurrent(); renderPreview(); renderSessions();
      if(logVisible) fetchLog();
    } catch(e) {
      showError(e.message);
    }
  }

  // Actions
  $('#btnStart').addEventListener('click', async (e)=>{
    const btn = e.target;
    btn.disabled = true;
    try {
      const r = await postJSON('/api/scan/start');
      const id = r && (r.id || r.session_id);
      if(id){ status = {state:'Scanning', id, step:'4×4 Scan', progress:0, remaining:0}; renderStatus(); }
    } finally {
      btn.disabled = false;
    }
  });
  $('#btnCancel').addEventListener('click', async (e)=>{
    const btn = e.target;
    btn.disabled = true;
    try {
      await postJSON('/api/scan/cancel');
      status = {state:'Idle', id:status?.id || '—', step:'—', progress:0, remaining:0};
      renderStatus();
    } finally {
      btn.disabled = false;
    }
  });
  $('#btnLog').addEventListener('click', async ()=>{
    logVisible = !logVisible;
    $('#logBox').style.display = logVisible ? 'block' : 'none';
    $('#btnLog').textContent = logVisible ? 'Hide Log' : 'Show Log';
    if(logVisible) await fetchLog();
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
