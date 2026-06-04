/*
 * Note: Claude (Anthropic) was used under human supervision to implement this
 * frontend UI. The optimizer algorithms themselves (HC, SA, GA) and the DANCE
 * scoring function are the author's own work in the C++ backend.
 */

// optimizer.js — multi-scenario, multi-algorithm road optimizer (embedded in app.html #optimizer-view)

const API = '';

// ── Palette ───────────────────────────────────────────────────────────────────
const PHASE_COLORS    = ['#ff4466','#00c8f0','#aa66ff','#44ee88','#ffaa22'];
const COLOR_CANDIDATE = '#f5a623';
const COLOR_CLOSED    = '#ff3355';   // pedestrianization: closed candidates
const COLOR_OPEN      = '#00e87a';   // pedestrianization: open candidates

// ── State ─────────────────────────────────────────────────────────────────────
let laneData         = [];       // [{wayId, laneIdx, points, type}]
let wayMeta          = {};       // wayId → {name, type, lanes}
let buildingData     = [];       // [{outer:[[{x,y},...]], inner:[[{x,y},...]]}]
let candidates       = [];       // [{k:'r'|'l', w:wayId, i?:laneIdx, label:str}]
let scenario         = 'phased_construction';
let activeProblemId  = null;
let activeSessionId  = null;
let pollTimer        = null;
let lastStatus       = null;
let sessionStartTime    = null; // Date.now() when the current session started
let sessionMaxParallel  = 1;   // maxParallelSims for the current session (for avg sim estimate)

// Map interaction — simple inline pan/zoom
let canvas, ctx;
let _ready = false;
const cam = { cx: 0, cy: 0, scale: 1 };

// Export viewport tool
let exportViewMode = false;
let exportViewport = null;   // { minX, minY, maxX, maxY } world coords, or null
let _vpDrag = null;          // { mode:'draw'|'move'|'resize', corner?, sx, sy, wx0?, wy0?, vp0? }

// Lane popup state
let lanePopupWayId   = null;
let lanePopupChecked = {};

// ── Road type labels (matches RoadType enum: 0=Motorway … 7=Unknown) ─────────
const ROAD_TYPE_LABELS = ['Motorway', 'Trunk', 'Primary', 'Secondary', 'Tertiary', 'Residential', 'Service', 'Other'];

// ── Helpers ───────────────────────────────────────────────────────────────────
function distToSegSq(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return (px - ax) ** 2 + (py - ay) ** 2;
  let t = ((px - ax) * dx + (py - ay) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const cx = ax + t * dx, cy = ay + t * dy;
  return (px - cx) ** 2 + (py - cy) ** 2;
}

function fmtRatio(v) {
  if (v == null || !isFinite(v)) return '—';
  const c = v < 1.0 ? 'good' : v > 1.2 ? 'bad' : '';
  return `<span class="ov-fitness-val ${c}">${v.toFixed(4)}</span>`;
}

function fmtTime(s) {
  if (s < 0 || !isFinite(s)) return '—';
  if (s < 60) return `${Math.round(s)}s`;
  if (s < 3600) return `${Math.round(s / 60)}m ${Math.round(s % 60)}s`;
  return `${Math.floor(s / 3600)}h ${Math.round((s % 3600) / 60)}m`;
}

function algoLabel(a) {
  if (a === 'hill_climbing')       return 'HC';
  if (a === 'simulated_annealing') return 'SA';
  if (a === 'genetic_algorithm')   return 'GA';
  return a || '?';
}

function algoClass(a) {
  if (a === 'hill_climbing')       return 'hc';
  if (a === 'simulated_annealing') return 'sa';
  if (a === 'genetic_algorithm')   return 'ga';
  return 'hc';
}

// ── Camera helpers ────────────────────────────────────────────────────────────
function W() { return canvas.clientWidth; }
function H() { return canvas.clientHeight; }
function worldToScreen(x, y) {
  return [W() / 2 + (x - cam.cx) * cam.scale,
          H() / 2 - (y - cam.cy) * cam.scale];
}
function screenToWorld(sx, sy) {
  return [(sx - W() / 2) / cam.scale + cam.cx,
          -(sy - H() / 2) / cam.scale + cam.cy];
}

// ── Export viewport helpers ───────────────────────────────────────────────────
function vpScreenRect() {
  if (!exportViewport) return null;
  const { minX, maxX, minY, maxY } = exportViewport;
  const [l, t] = worldToScreen(minX, maxY);
  const [r, b] = worldToScreen(maxX, minY);
  return { l, t, r, b };
}

function vpHitTest(clientX, clientY) {
  if (!exportViewport) return null;
  const rect = canvas.getBoundingClientRect();
  const sx = clientX - rect.left, sy = clientY - rect.top;
  const sr = vpScreenRect();
  const HANDLE = 10;
  if (Math.hypot(sx - sr.l, sy - sr.t) < HANDLE) return 'tl';
  if (Math.hypot(sx - sr.r, sy - sr.t) < HANDLE) return 'tr';
  if (Math.hypot(sx - sr.l, sy - sr.b) < HANDLE) return 'bl';
  if (Math.hypot(sx - sr.r, sy - sr.b) < HANDLE) return 'br';
  if (sx >= sr.l && sx <= sr.r && sy >= sr.t && sy <= sr.b) return 'inside';
  return null;
}

function vpUpdateCursor(clientX, clientY) {
  const hit = vpHitTest(clientX, clientY);
  if (hit === 'tl' || hit === 'br') canvas.style.cursor = 'nwse-resize';
  else if (hit === 'tr' || hit === 'bl') canvas.style.cursor = 'nesw-resize';
  else if (hit === 'inside') canvas.style.cursor = 'move';
  else canvas.style.cursor = 'crosshair';
}

function toggleExportViewMode() {
  exportViewMode = !exportViewMode;
  const btns = [
    document.getElementById('ov-exportAreaBtn'),
    document.getElementById('ov-histExportAreaBtn'),
  ];
  if (exportViewMode) {
    btns.forEach(b => b?.classList.add('active'));
    canvas.style.cursor = 'crosshair';
    if (!exportViewport) {
      const candWayIds = new Set(candidates.map(c => c.w));
      let cMinX = Infinity, cMaxX = -Infinity, cMinY = Infinity, cMaxY = -Infinity;
      for (const { wayId, points } of laneData) {
        if (!candWayIds.has(wayId)) continue;
        for (const [x, y] of points) {
          if (x < cMinX) cMinX = x; if (x > cMaxX) cMaxX = x;
          if (y < cMinY) cMinY = y; if (y > cMaxY) cMaxY = y;
        }
      }
      if (isFinite(cMinX)) {
        const px = (cMaxX - cMinX) * 0.08, py = (cMaxY - cMinY) * 0.08;
        exportViewport = { minX: cMinX - px, maxX: cMaxX + px, minY: cMinY - py, maxY: cMaxY + py };
      }
    }
  } else {
    btns.forEach(b => b?.classList.remove('active'));
    canvas.style.cursor = '';
  }
}

function renderExportViewport() {
  if (!exportViewMode) return;
  const sr = vpScreenRect();
  ctx.save();
  if (sr) {
    const w = sr.r - sr.l, h = sr.b - sr.t;
    ctx.fillStyle = 'rgba(0,0,0,0.45)';
    ctx.beginPath();
    ctx.rect(0, 0, W(), H());
    ctx.rect(sr.l, sr.t, w, h);
    ctx.fill('evenodd');
    ctx.strokeStyle = '#f5a623';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.strokeRect(sr.l, sr.t, w, h);
    ctx.setLineDash([]);
    ctx.fillStyle = '#f5a623';
    const HANDLE = 8;
    for (const [cx, cy] of [[sr.l, sr.t], [sr.r, sr.t], [sr.l, sr.b], [sr.r, sr.b]]) {
      ctx.fillRect(cx - HANDLE / 2, cy - HANDLE / 2, HANDLE, HANDLE);
    }
  } else {
    ctx.fillStyle = 'rgba(0,0,0,0.25)';
    ctx.fillRect(0, 0, W(), H());
  }
  ctx.restore();
}

// ── Map rendering ─────────────────────────────────────────────────────────────
function resizeCanvas() {
  if (!canvas) return;
  const pane = canvas.parentElement;
  const dpr  = window.devicePixelRatio || 1;
  const w = pane.clientWidth, h = pane.clientHeight;
  canvas.style.width = w + 'px'; canvas.style.height = h + 'px';
  canvas.width = Math.floor(w * dpr); canvas.height = Math.floor(h * dpr);
}

function isWayCandidated(wayId) {
  return candidates.some(c => c.w === wayId);
}

function render() {
  if (!canvas) { requestAnimationFrame(render); return; }
  const dpr = window.devicePixelRatio || 1;
  ctx.save();
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, canvas.width / dpr, canvas.height / dpr);

  if (!laneData.length) { ctx.restore(); requestAnimationFrame(render); return; }

  // Build coloring maps for result overlay
  const wayColorMap  = new Map();   // wayId → color
  const laneColorMap = new Map();   // `${wayId}:${laneIdx}` → color

  if (lastStatus) {
    const curScenario = lastStatus.scenario || scenario;
    if (curScenario === 'phased_construction' && lastStatus.bestAssignment && lastStatus.bestAssignment.length) {
      for (let i = 0; i < candidates.length; i++) {
        const c   = candidates[i];
        const ph  = lastStatus.bestAssignment[i] ?? 0;
        const col = PHASE_COLORS[ph % PHASE_COLORS.length];
        if (c.k === 'r') wayColorMap.set(c.w, col);
        else              laneColorMap.set(`${c.w}:${c.i}`, col);
      }
    } else if ((lastStatus.scenario || scenario) === 'pedestrianization' && lastStatus.bestAssignment) {
      for (let i = 0; i < candidates.length; i++) {
        const c   = candidates[i];
        const col = (lastStatus.bestAssignment[i] === 1) ? COLOR_CLOSED : COLOR_OPEN;
        if (c.k === 'r') wayColorMap.set(c.w, col);
        else              laneColorMap.set(`${c.w}:${c.i}`, col);
      }
    }
  }

  const baseW = (rt) => rt <= 1 ? 3.5 : rt <= 2 ? 2.5 : rt <= 3 ? 2.0 : rt <= 4 ? 1.5 : 1.0;
  const baseC = (rt) => {
    if (rt <= 1) return 'rgba(210,190,120,0.85)';
    if (rt <= 2) return 'rgba(170,155,110,0.75)';
    if (rt <= 3) return 'rgba(130,120,100,0.65)';
    if (rt <= 4) return 'rgba(100,110,120,0.55)';
    return 'rgba(80,90,100,0.45)';
  };

  for (const lane of laneData) {
    const { wayId, laneIdx, points, type } = lane;
    const laneKey    = `${wayId}:${laneIdx}`;
    const laneResult = laneColorMap.get(laneKey);
    const wayResult  = wayColorMap.get(wayId);

    let color, width;
    if (laneResult) {
      color = laneResult; width = Math.max(1.4, cam.scale * 2.5);
    } else if (wayResult) {
      color = wayResult; width = Math.max(1.4, cam.scale * baseW(type));
    } else if (isWayCandidated(wayId) && !candidates.some(c => c.k === 'l' && c.w === wayId)) {
      color = COLOR_CANDIDATE; width = Math.max(1.4, cam.scale * baseW(type));
    } else {
      color = baseC(type); width = Math.max(0.7, cam.scale * baseW(type));
    }

    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    let first = true;
    for (const [wx, wy] of points) {
      const [sx, sy] = worldToScreen(wx, wy);
      if (first) { ctx.moveTo(sx, sy); first = false; }
      else          ctx.lineTo(sx, sy);
    }
    ctx.stroke();
  }

  renderExportViewport();

  ctx.restore();
  requestAnimationFrame(render);
}

// ── Init map ──────────────────────────────────────────────────────────────────
function initMap() {
  if (_ready) return;
  _ready = true;

  canvas = document.getElementById('ov-mapCanvas');
  ctx    = canvas.getContext('2d');
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  // Pan / viewport tool
  let drag = null;
  canvas.addEventListener('mousedown', e => {
    if (e.button !== 0) return;
    if (exportViewMode) {
      const rect = canvas.getBoundingClientRect();
      const hit  = vpHitTest(e.clientX, e.clientY);
      if (hit === 'tl' || hit === 'tr' || hit === 'bl' || hit === 'br') {
        _vpDrag = { mode: 'resize', corner: hit, sx: e.clientX, sy: e.clientY, vp0: { ...exportViewport } };
      } else if (hit === 'inside') {
        _vpDrag = { mode: 'move', sx: e.clientX, sy: e.clientY, vp0: { ...exportViewport } };
      } else {
        const [wx, wy] = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
        _vpDrag = { mode: 'draw', wx0: wx, wy0: wy };
        exportViewport = { minX: wx, maxX: wx, minY: wy, maxY: wy };
      }
      return;
    }
    drag = { sx: e.clientX, sy: e.clientY, cx0: cam.cx, cy0: cam.cy };
  });
  window.addEventListener('mousemove', e => {
    if (exportViewMode) {
      if (_vpDrag) {
        const rect = canvas.getBoundingClientRect();
        const [wx, wy] = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
        const dxW = (e.clientX - _vpDrag.sx) / cam.scale;
        const dyW = (e.clientY - _vpDrag.sy) / cam.scale;
        if (_vpDrag.mode === 'draw') {
          exportViewport = {
            minX: Math.min(_vpDrag.wx0, wx), maxX: Math.max(_vpDrag.wx0, wx),
            minY: Math.min(_vpDrag.wy0, wy), maxY: Math.max(_vpDrag.wy0, wy),
          };
        } else if (_vpDrag.mode === 'move') {
          const v = _vpDrag.vp0;
          exportViewport = { minX: v.minX + dxW, maxX: v.maxX + dxW, minY: v.minY - dyW, maxY: v.maxY - dyW };
        } else if (_vpDrag.mode === 'resize') {
          const v = _vpDrag.vp0, c = _vpDrag.corner;
          let { minX, maxX, minY, maxY } = v;
          if (c.includes('l')) minX = v.minX + dxW;
          if (c.includes('r')) maxX = v.maxX + dxW;
          if (c.includes('t')) maxY = v.maxY - dyW;
          if (c.includes('b')) minY = v.minY - dyW;
          exportViewport = {
            minX: Math.min(minX, maxX), maxX: Math.max(minX, maxX),
            minY: Math.min(minY, maxY), maxY: Math.max(minY, maxY),
          };
        }
      }
      vpUpdateCursor(e.clientX, e.clientY);
      return;
    }
    if (!drag) return;
    cam.cx = drag.cx0 - (e.clientX - drag.sx) / cam.scale;
    cam.cy = drag.cy0 + (e.clientY - drag.sy) / cam.scale;
  });
  window.addEventListener('mouseup', () => { drag = null; _vpDrag = null; });

  // Zoom
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
    const [wx, wy] = screenToWorld(sx, sy);
    const factor = Math.pow(1.001, -e.deltaY);
    cam.scale = Math.max(0.1, Math.min(500, cam.scale * factor));
    cam.cx = wx - (sx - W() / 2) / cam.scale;
    cam.cy = wy + (sy - H() / 2) / cam.scale;
  }, { passive: false });

  canvas.addEventListener('click', e => { if (!exportViewMode) onMapClick(e); });
  canvas.addEventListener('contextmenu', e => { e.preventDefault(); if (!exportViewMode) onMapRightClick(e); });

  requestAnimationFrame(render);
  loadMap();
}

async function loadMap() {
  try {
    const r = await fetch(`${API}/map`);
    const data = await r.json();
    processMapData(data);
  } catch (e) {
    console.error('Optimizer: failed to load map:', e);
  }
}

function processMapData(data) {
  laneData     = [];
  wayMeta      = {};
  buildingData = (data.buildings || []).map(b => ({ outer: b.outer || [], inner: b.inner || [] }));

  const wayInfo = {};
  for (const w of (data.ways || [])) {
    wayInfo[w.id] = { name: w.name || `Way ${w.id}`, type: w.type ?? 5 };
  }

  if (!data.lanes) return;
  for (const l of data.lanes) {
    const pts = (l.points || []).map(p => [p.x, p.y]);
    if (pts.length < 2) continue;

    // laneId = wayId * 1000 + 100/200 + laneIndex  (backend encoding)
    const wayId   = Math.floor(l.id / 1000);
    const laneIdx = l.id % 1000 >= 200 ? (l.id % 1000) - 200 : (l.id % 1000) - 100;
    const info    = wayInfo[wayId] || { name: `Way ${wayId}`, type: 5 };

    laneData.push({ wayId, laneIdx, points: pts, type: info.type });

    if (!wayMeta[wayId]) {
      wayMeta[wayId] = { name: info.name, type: info.type, lanes: 1 };
    } else {
      wayMeta[wayId].lanes = Math.max(wayMeta[wayId].lanes, laneIdx + 1);
    }
  }

  if (laneData.length) {
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const lane of laneData) {
      for (const [x, y] of lane.points) {
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
      }
    }
    const pw = W(), ph = H();
    const mw = maxX - minX, mh = maxY - minY;
    cam.scale = Math.min(pw / Math.max(mw, 1), ph / Math.max(mh, 1)) * 0.9;
    cam.cx    = (minX + maxX) / 2;
    cam.cy    = (minY + maxY) / 2;
  }
}

// ── Map click handlers ────────────────────────────────────────────────────────
function hitTestWay(event) {
  const rect = canvas.getBoundingClientRect();
  const mx = event.clientX - rect.left;
  const my = event.clientY - rect.top;
  const THR = 12;

  let bestDist = Infinity, bestWayId = null;
  for (const lane of laneData) {
    const pts = lane.points;
    for (let i = 0; i < pts.length - 1; i++) {
      const [sx1, sy1] = worldToScreen(pts[i][0], pts[i][1]);
      const [sx2, sy2] = worldToScreen(pts[i+1][0], pts[i+1][1]);
      const d = distToSegSq(mx, my, sx1, sy1, sx2, sy2);
      if (d < bestDist) { bestDist = d; bestWayId = lane.wayId; }
    }
  }
  return bestDist <= THR * THR ? bestWayId : null;
}

function onMapClick(e) {
  if (e.button !== 0) return;
  const wayId = hitTestWay(e);
  if (wayId == null) return;
  toggleWholeRoad(wayId);
}

function onMapRightClick(e) {
  e.preventDefault();
  const wayId = hitTestWay(e);
  if (wayId == null) return;
  showLanePopup(wayId, e.clientX, e.clientY);
}

function toggleWholeRoad(wayId) {
  const existing = candidates.filter(c => c.w === wayId);
  if (existing.length > 0) {
    candidates = candidates.filter(c => c.w !== wayId);
  } else {
    const meta = wayMeta[wayId];
    candidates.push({ k: 'r', w: wayId, label: meta?.name || `Way ${wayId}` });
  }
  renderCandidateList();
}

// ── Lane popup ────────────────────────────────────────────────────────────────
function showLanePopup(wayId, cx, cy) {
  lanePopupWayId   = wayId;
  lanePopupChecked = {};

  const meta      = wayMeta[wayId];
  const laneCount = meta?.lanes ?? 1;
  const popup     = document.getElementById('ov-lane-popup');

  document.getElementById('ov-lp-road-name').textContent = meta?.name || `Way ${wayId}`;

  const rows = document.getElementById('ov-lp-rows');
  rows.innerHTML = '';

  // "Whole road" option
  const wholeChecked = candidates.some(c => c.k === 'r' && c.w === wayId);
  {
    const row = document.createElement('div');
    row.className = 'lp-row';
    const cb  = document.createElement('input');
    cb.type   = 'checkbox'; cb.id = 'lp-whole'; cb.checked = wholeChecked;
    cb.addEventListener('change', () => {
      if (cb.checked) {
        for (const k in lanePopupChecked) lanePopupChecked[k] = false;
        rows.querySelectorAll('.lp-lane-cb').forEach(c => c.checked = false);
      }
    });
    const lbl = document.createElement('label');
    lbl.htmlFor = 'lp-whole'; lbl.textContent = 'Whole road';
    row.append(cb, lbl);
    rows.appendChild(row);
    rows._wholeCb = cb;
  }

  // Per-lane options
  for (let li = 0; li < laneCount; li++) {
    const alreadyClosed = candidates.some(c => c.k === 'l' && c.w === wayId && c.i === li);
    lanePopupChecked[li] = alreadyClosed;

    const row = document.createElement('div');
    row.className = 'lp-row';
    const cb  = document.createElement('input');
    cb.type   = 'checkbox'; cb.id = `lp-lane-${li}`;
    cb.className = 'lp-lane-cb';
    cb.checked = alreadyClosed;
    cb.addEventListener('change', () => {
      lanePopupChecked[li] = cb.checked;
      if (cb.checked && rows._wholeCb) rows._wholeCb.checked = false;
    });
    const lbl = document.createElement('label');
    lbl.htmlFor = `lp-lane-${li}`; lbl.textContent = `Lane ${li}`;
    row.append(cb, lbl);
    rows.appendChild(row);
  }

  // Position popup near click
  popup.style.display = 'block';
  const pr = canvas.parentElement.getBoundingClientRect();
  popup.style.left = Math.min(cx - pr.left + 8, pr.width - 200) + 'px';
  popup.style.top  = Math.min(cy - pr.top  + 8, pr.height - 200) + 'px';
}

document.getElementById('ov-lp-close').addEventListener('click', () => {
  document.getElementById('ov-lane-popup').style.display = 'none';
});

document.getElementById('ov-lp-apply').addEventListener('click', () => {
  const wayId = lanePopupWayId;
  if (wayId == null) return;

  candidates = candidates.filter(c => c.w !== wayId);

  const rows  = document.getElementById('ov-lp-rows');
  const whole = rows._wholeCb?.checked ?? false;
  const meta  = wayMeta[wayId];

  if (whole) {
    candidates.push({ k: 'r', w: wayId, label: meta?.name || `Way ${wayId}` });
  } else {
    for (const [li, checked] of Object.entries(lanePopupChecked)) {
      if (checked) {
        candidates.push({
          k: 'l', w: wayId, i: parseInt(li),
          label: `${meta?.name || `Way ${wayId}`} · Lane ${li}`,
        });
      }
    }
  }

  document.getElementById('ov-lane-popup').style.display = 'none';
  renderCandidateList();
});

// ── Candidate list UI ─────────────────────────────────────────────────────────
function candType(c) {
  const t = wayMeta[c.w]?.type;
  return (t != null && t >= 0 && t < ROAD_TYPE_LABELS.length) ? t : ROAD_TYPE_LABELS.length - 1;
}

function renderCandidateList() {
  const list = document.getElementById('ov-cands-list');
  const none = document.getElementById('ov-no-cands');

  if (candidates.length === 0) {
    list.innerHTML = '';
    if (none) list.appendChild(none);
    renderTypeFilterBtns();
    return;
  }

  list.innerHTML = '';
  for (let i = 0; i < candidates.length; i++) {
    const c = candidates[i];
    const div = document.createElement('div');
    div.className = 'ov-cand-item';
    const icon = document.createElement('span');
    icon.className = 'cand-icon';
    icon.textContent = c.k === 'l' ? '⇐' : '▬';
    const lbl = document.createElement('span');
    lbl.className = 'cand-label';
    lbl.textContent = c.label;
    const rm = document.createElement('button');
    rm.className = 'cand-remove';
    rm.textContent = '✕';
    rm.addEventListener('click', () => {
      candidates.splice(i, 1);
      renderCandidateList();
    });
    div.append(icon, lbl, rm);
    list.appendChild(div);
  }

  renderTypeFilterBtns();
}

function renderTypeFilterBtns() {
  const container = document.getElementById('ov-typeFilterBtns');
  if (!container) return;
  container.innerHTML = '';

  // Count candidates per type — only show buttons for types that are present
  const typeCounts = new Map();
  for (const c of candidates) {
    const t = candType(c);
    typeCounts.set(t, (typeCounts.get(t) || 0) + 1);
  }

  for (const [t, count] of [...typeCounts].sort((a, b) => a[0] - b[0])) {
    const btn = document.createElement('button');
    btn.className = 'ov-btn';
    btn.style.cssText = 'font-size:10px;padding:2px 7px';
    btn.textContent = `${ROAD_TYPE_LABELS[t]} (${count})`;
    btn.title = `Remove all ${ROAD_TYPE_LABELS[t]} roads from candidates`;
    btn.addEventListener('click', () => {
      candidates = candidates.filter(c => candType(c) !== t);
      renderCandidateList();
    });
    container.appendChild(btn);
  }
}

// ── Scenario UI ───────────────────────────────────────────────────────────────
document.querySelectorAll('.ov-scenario-card').forEach(card => {
  card.addEventListener('click', () => {
    document.querySelectorAll('.ov-scenario-card').forEach(c => c.classList.remove('active'));
    card.classList.add('active');
    scenario = card.dataset.scenario;
    document.getElementById('ov-phased-opts').style.display     = scenario === 'phased_construction' ? '' : 'none';
    document.getElementById('ov-ped-opts').style.display        = scenario === 'pedestrianization'   ? '' : 'none';
  });
});

// ── Algorithm tabs ────────────────────────────────────────────────────────────
let selectedAlgo = 'hill_climbing';
document.querySelectorAll('.ov-algo-tab').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.ov-algo-tab').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    selectedAlgo = btn.dataset.algo;
    document.querySelectorAll('.ov-algo-params').forEach(p => p.style.display = 'none');
    document.getElementById(`ov-params-${selectedAlgo}`).style.display = '';
  });
});

// ── Tab switching ─────────────────────────────────────────────────────────────
document.querySelectorAll('.ov-tab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.ov-tab-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.ov-tab-pane').forEach(p => p.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById(`ov-tab-${btn.dataset.tab}`).classList.add('active');
    if (btn.dataset.tab === 'run' && !_ready)     initMap();
    if (btn.dataset.tab === 'problem' && !_ready) initMap();
    if (btn.dataset.tab === 'history')            loadSessionHistory();
  });
});

// ── Problem save / load ───────────────────────────────────────────────────────
document.getElementById('ov-clearCandsBtn').addEventListener('click', () => {
  candidates = [];
  renderCandidateList();
});

document.getElementById('ov-saveProblemBtn').addEventListener('click', async () => {
  if (candidates.length === 0) { alert('Add at least one candidate road.'); return; }

  const name       = document.getElementById('ov-probName').value.trim() || 'Problem';
  const phaseCount = parseInt(document.getElementById('ov-phaseCount').value) || 3;
  const simDur     = parseFloat(document.getElementById('ov-simDur').value) || 3600;
  const traffic    = document.getElementById('ov-trafficMode').value;
  const threshold  = parseFloat(document.getElementById('ov-threshold').value) || 1.15;

  const candJson = candidates.map(c => c.k === 'r' ? { k: 'r', w: c.w } : { k: 'l', w: c.w, i: c.i });

  const body = {
    name,
    phaseCount,
    simDurationS:        simDur,
    trafficMode:         traffic,
    candidateRoadIds:    candJson,
    scenarioType:        scenario,
    constraintThreshold: threshold,
  };

  try {
    const r   = await fetch(`${API}/optimizer/problems`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const data = await r.json();
    if (!data.ok) { alert('Failed: ' + (data.error || '?')); return; }
    await loadProblems();
    selectProblem(data.problemId);
  } catch (e) { alert('Error: ' + e.message); }
});

async function loadProblems() {
  const div = document.getElementById('ov-savedProblems');
  try {
    const r   = await fetch(`${API}/optimizer/problems`);
    const arr = await r.json();
    if (!arr.length) {
      div.innerHTML = '<div style="color:var(--c-text-dim);font-size:11px">No problems saved.</div>';
      return;
    }

    div.innerHTML = '';
    for (const p of arr) {
      const item = document.createElement('div');
      item.className = 'ov-prob-item' + (p.problemId === activeProblemId ? ' active' : '');
      item.dataset.id = p.problemId;

      const info = document.createElement('div');
      info.className = 'sp-info';
      const nm = document.createElement('div'); nm.className = 'sp-name'; nm.textContent = p.name;
      const mt = document.createElement('div'); mt.className = 'sp-meta';
      const scenLabel = p.scenarioType === 'pedestrianization' ? 'Pedestrianization' : `Phased (${p.phaseCount} phases)`;
      const cands = Array.isArray(p.candidateRoadIds) ? p.candidateRoadIds.length : '?';
      mt.textContent = `${scenLabel} · ${cands} candidates · ${p.trafficMode}`;
      info.append(nm, mt);

      const del = document.createElement('button');
      del.className = 'sp-del'; del.textContent = '✕';
      del.addEventListener('click', async (e) => {
        e.stopPropagation();
        if (!confirm(`Delete problem "${p.name}"?`)) return;
        await fetch(`${API}/optimizer/problems/${p.problemId}`, { method: 'DELETE' });
        if (activeProblemId === p.problemId) { activeProblemId = null; updateActiveProblemInfo(null); }
        loadProblems();
      });

      item.append(info, del);
      item.addEventListener('click', () => selectProblem(p.problemId, p));
      div.appendChild(item);
    }
  } catch { div.innerHTML = '<div style="color:var(--c-text-dim);font-size:11px">DB unavailable.</div>'; }
}

function selectProblem(id, data) {
  activeProblemId = id;
  document.querySelectorAll('.ov-prob-item').forEach(el => {
    el.classList.toggle('active', el.dataset.id === id);
  });
  if (data) updateActiveProblemInfo(data);
  else      loadProblemData(id);
  loadSessionHistory();
}

async function loadProblemData(id) {
  try {
    const r    = await fetch(`${API}/optimizer/problems`);
    const arr  = await r.json();
    const prob = arr.find(p => p.problemId === id);
    if (prob) updateActiveProblemInfo(prob);
  } catch {}
}

function updateActiveProblemInfo(p) {
  const div = document.getElementById('ov-activeProblemInfo');
  if (!p) {
    div.innerHTML = 'No problem selected. Go to <b>Problem</b> tab and click a saved problem.';
    div.style.color = 'var(--c-text-dim)';
    return;
  }
  div.style.color = 'var(--c-text)';
  const scenLabel = p.scenarioType === 'pedestrianization' ? 'Pedestrianization' : `Phased (${p.phaseCount} phases)`;
  const cands = Array.isArray(p.candidateRoadIds) ? p.candidateRoadIds.length : '?';
  div.innerHTML = `<b style="color:var(--c-text-hi)">${p.name}</b><br>
    <span style="color:var(--c-text-label)">${scenLabel} · ${cands} candidates · ${p.trafficMode} traffic · ${p.simDurationS}s sim</span>`;

  scenario = p.scenarioType || 'phased_construction';
  document.querySelectorAll('.ov-scenario-card').forEach(c => {
    c.classList.toggle('active', c.dataset.scenario === scenario);
  });

  if (Array.isArray(p.candidateRoadIds)) {
    candidates = p.candidateRoadIds.map(c => {
      if (typeof c === 'number') {
        return { k: 'r', w: c, label: wayMeta[c]?.name || `Way ${c}` };
      }
      if (c.k === 'r') return { k: 'r', w: c.w, label: wayMeta[c.w]?.name || `Way ${c.w}` };
      if (c.k === 'l') return { k: 'l', w: c.w, i: c.i, label: `${wayMeta[c.w]?.name || `Way ${c.w}`} · Lane ${c.i}` };
      return null;
    }).filter(Boolean);
    renderCandidateList();
  }
}

// ── Run ───────────────────────────────────────────────────────────────────────
document.getElementById('ov-startBtn').addEventListener('click', startSession);
document.getElementById('ov-pauseBtn').addEventListener('click', () => sessionAction('pause'));
document.getElementById('ov-resumeBtn').addEventListener('click', () => sessionAction('resume'));
document.getElementById('ov-cancelBtn').addEventListener('click', () => sessionAction('cancel'));

async function startSession() {
  if (!activeProblemId) { alert('Select a problem first.'); return; }

  const algo = selectedAlgo;
  let maxIter, maxParallel;

  if (algo === 'hill_climbing') {
    maxIter     = parseInt(document.getElementById('ov-hc-maxIter').value) || 30;
    maxParallel = parseInt(document.getElementById('ov-hc-parallel').value) || 4;
  } else if (algo === 'simulated_annealing') {
    maxIter     = parseInt(document.getElementById('ov-sa-maxIter').value) || 60;
    maxParallel = parseInt(document.getElementById('ov-sa-parallel').value) || 2;
  } else {
    maxIter     = parseInt(document.getElementById('ov-ga-maxIter').value) || 40;
    maxParallel = parseInt(document.getElementById('ov-ga-parallel').value) || 4;
  }

  const runsPerEval = Math.max(1, Math.min(5, parseInt(document.getElementById('ov-runsPerEval').value) || 1));
  const teMult      = Math.max(0, parseFloat(document.getElementById('ov-teMult').value) || 0);
  const seedRaw     = document.getElementById('ov-seed').value.trim();
  const seedVal     = seedRaw !== '' ? parseInt(seedRaw) : undefined;

  const body = {
    problemId:      activeProblemId,
    algorithm:      algo,
    maxIter,
    maxParallelSims: maxParallel,
    runsPerEval,
    teMult,
    ...(seedVal !== undefined && !isNaN(seedVal) ? { seed: seedVal } : {}),
    saT0:      algo === 'simulated_annealing' ? parseFloat(document.getElementById('ov-sa-T0').value)    || 0.1   : undefined,
    saTmin:    algo === 'simulated_annealing' ? parseFloat(document.getElementById('ov-sa-Tmin').value)  || 0.005 : undefined,
    gaPopSize: algo === 'genetic_algorithm'   ? parseInt(document.getElementById('ov-ga-popSize').value) || 20    : undefined,
  };

  try {
    const r = await fetch(`${API}/optimizer/sessions`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
    });
    const d = await r.json();
    if (!d.ok) { alert('Failed to start: ' + (d.error || '?')); return; }
    activeSessionId     = d.sessionId;
    sessionStartTime    = Date.now();
    sessionMaxParallel  = maxParallel;
    startPolling();
    setRunButtons('running');
  } catch (e) { alert('Error: ' + e.message); }
}

async function sessionAction(action) {
  if (!activeSessionId) return;
  try {
    await fetch(`${API}/optimizer/sessions/${activeSessionId}/${action}`, { method: 'POST' });
    if (action === 'pause')  setRunButtons('paused');
    if (action === 'resume') setRunButtons('running');
    if (action === 'cancel') { stopPolling(); setRunButtons('idle'); }
  } catch (e) { console.error(e); }
}

function setRunButtons(state) {
  document.getElementById('ov-startBtn').style.display  = state === 'idle' ? '' : 'none';
  document.getElementById('ov-pauseBtn').style.display  = state === 'running' ? '' : 'none';
  document.getElementById('ov-resumeBtn').style.display = state === 'paused'  ? '' : 'none';
  document.getElementById('ov-cancelBtn').style.display = (state === 'running' || state === 'paused') ? '' : 'none';
}

// ── Polling ───────────────────────────────────────────────────────────────────
function startPolling() {
  stopPolling();
  pollTimer = setInterval(pollStatus, 2000);
  pollStatus();
}

function stopPolling() {
  if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
}

async function pollStatus() {
  if (!activeSessionId) return;
  try {
    const r = await fetch(`${API}/optimizer/sessions/${activeSessionId}/status`);
    if (!r.ok) return;
    const s = await r.json();
    lastStatus = s;
    updateRunUI(s);
    if (s.status === 'completed' || s.status === 'failed') {
      stopPolling();
      setRunButtons('idle');
      loadSessionHistory();
    }
  } catch {}
}

function updateRunUI(s) {
  const statusEl = document.getElementById('ov-runStatus');
  const pct      = s.totalEvals > 0 ? Math.min(1, s.evalsDone / s.totalEvals) : 0;
  const pctStr   = s.totalEvals > 0 ? ` (${(pct * 100).toFixed(0)}%)` : '';

  statusEl.innerHTML = `<span class="ov-badge ${s.status}">${s.status}</span>
    &nbsp; iter ${s.itersDone ?? 0} · eval ${s.evalsDone ?? 0}/${s.totalEvals ?? '?'}${pctStr}`;

  document.getElementById('ov-progressWrap').style.display = '';
  document.getElementById('ov-progressBarFill').style.width = (pct * 100).toFixed(1) + '%';

  // ETA line: remaining + elapsed + avg sim speed
  const etaParts = [];
  if (s.etaSeconds > 0 && s.status === 'running') etaParts.push(`ETA ${fmtTime(s.etaSeconds)}`);
  if (sessionStartTime) {
    const elapsedS = (Date.now() - sessionStartTime) / 1000;
    if (elapsedS > 2) etaParts.push(`elapsed ${fmtTime(elapsedS)}`);
    // derive avg sim time from elapsed + eval count (accounting for parallel)
    if (s.evalsDone > 0 && sessionMaxParallel > 0) {
      const avgS = elapsedS * sessionMaxParallel / s.evalsDone;
      etaParts.push(`~${avgS.toFixed(1)}s/sim`);
    }
  }
  document.getElementById('ov-etaLabel').textContent = etaParts.join(' · ');

  if (s.baselineDANC != null) {
    document.getElementById('ov-resultSummary').innerHTML =
      `Baseline DANC: <b>${s.baselineDANC.toFixed(4)}</b> &nbsp; Best fitness: ${fmtRatio(s.bestFitness)}`;
  }

  document.getElementById('ov-secResults').style.display = '';

  const currentScenario = s.scenario || scenario;
  if (currentScenario === 'pedestrianization') {
    document.getElementById('ov-resultPhased').style.display     = 'none';
    document.getElementById('ov-resultPedestrian').style.display = '';
    renderPedestrianResult(s);
  } else {
    document.getElementById('ov-resultPhased').style.display     = '';
    document.getElementById('ov-resultPedestrian').style.display = 'none';
    renderPhasedResult(s);
  }
  scenario = currentScenario;

  if (s.convergenceHistory && s.convergenceHistory.length > 1) {
    drawConvergence(s.convergenceHistory);
  }
}

// ── Phased result ─────────────────────────────────────────────────────────────
function renderPhasedResult(s) {
  const legend = document.getElementById('ov-phaseLegend');
  const tbody  = document.getElementById('ov-phaseResultTbody');
  const K      = s.phaseCount ?? (s.phaseFitnesses ? s.phaseFitnesses.length : 2);

  legend.innerHTML = '';
  for (let k = 0; k < K; k++) {
    const col  = PHASE_COLORS[k % PHASE_COLORS.length];
    const chip = document.createElement('div');
    chip.className = 'ov-phase-chip';
    chip.style.borderColor = col + '88';
    chip.style.color       = col;
    chip.innerHTML = `<span class="ov-phase-dot" style="background:${col}"></span>Phase ${k+1}`;
    legend.appendChild(chip);
  }

  tbody.innerHTML = '';
  for (let k = 0; k < K; k++) {
    const col   = PHASE_COLORS[k % PHASE_COLORS.length];
    const fit   = s.phaseFitnesses ? s.phaseFitnesses[k] : null;
    const roads = [];
    if (s.bestAssignment && candidates.length) {
      for (let i = 0; i < candidates.length; i++) {
        if (s.bestAssignment[i] === k) roads.push(candidates[i].label);
      }
    }
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td><span class="ov-phase-dot" style="background:${col}"></span></td>
      <td>Phase ${k+1}</td>
      <td>${fmtRatio(fit)}</td>
      <td style="max-width:160px;word-break:break-word;line-height:1.5;color:var(--c-text-label)">${roads.join(', ') || '—'}</td>`;
    tbody.appendChild(tr);
  }
}

// ── Pedestrianization result ──────────────────────────────────────────────────
function renderPedestrianResult(s) {
  const tbody = document.getElementById('ov-pedestrianResultTbody');
  tbody.innerHTML = '';

  for (let i = 0; i < candidates.length; i++) {
    const c      = candidates[i];
    const closed = s.bestAssignment ? (s.bestAssignment[i] === 1) : false;
    const tr     = document.createElement('tr');
    const badge  = closed
      ? `<span class="ov-badge failed">Closed</span>`
      : `<span class="ov-badge completed">Open</span>`;
    tr.innerHTML = `
      <td style="color:var(--c-text-hi)">${c.label}</td>
      <td style="color:var(--c-text-label)">${c.k === 'l' ? 'lane' : 'road'}</td>
      <td>${badge}</td>`;
    tbody.appendChild(tr);
  }

  if (s.bestClosedCount != null) {
    const dancRatio = s.phaseFitnesses && s.phaseFitnesses.length > 0 ? s.phaseFitnesses[0] : null;
    const ratioHtml = dancRatio != null
      ? ` &nbsp; DANCE ratio: ${fmtRatio(dancRatio)}`
      : '';
    document.getElementById('ov-resultSummary').innerHTML +=
      ` &nbsp; Closed: <b>${s.bestClosedCount}</b>/${candidates.length}${ratioHtml}`;
  }
}

// ── Convergence chart ─────────────────────────────────────────────────────────
function drawConvergence(history) {
  const cvs = document.getElementById('ov-convCanvas');
  if (!cvs) return;
  const ct  = cvs.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const w   = cvs.clientWidth, h = cvs.clientHeight;
  cvs.width = Math.floor(w * dpr); cvs.height = Math.floor(h * dpr);
  ct.scale(dpr, dpr);

  const pts  = history;
  const minF = Math.min(...pts.map(p => p.fit));
  const maxF = Math.max(...pts.map(p => p.fit));
  const rng  = Math.max(maxF - minF, 0.001);
  const pad  = 12;

  ct.clearRect(0, 0, w, h);
  ct.strokeStyle = 'rgba(255,255,255,0.07)';
  ct.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = pad + ((h - 2 * pad) * i / 4);
    ct.beginPath(); ct.moveTo(pad, y); ct.lineTo(w - pad, y); ct.stroke();
  }

  ct.strokeStyle = '#00c8f0';
  ct.lineWidth   = 1.5;
  ct.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const x = pad + (i / Math.max(pts.length - 1, 1)) * (w - 2 * pad);
    const y = pad + (1 - (pts[i].fit - minF) / rng) * (h - 2 * pad);
    if (i === 0) ct.moveTo(x, y); else ct.lineTo(x, y);
  }
  ct.stroke();

  ct.fillStyle = 'rgba(148,163,184,0.7)';
  ct.font      = '9px system-ui';
  ct.textAlign = 'left';
  ct.fillText(maxF.toFixed(3), pad + 2, pad + 8);
  ct.fillText(minF.toFixed(3), pad + 2, h - pad - 2);
}

// ── History tab ───────────────────────────────────────────────────────────────
async function loadSessionHistory() {
  const div = document.getElementById('ov-sessionsList');
  const msg = document.getElementById('ov-sessionsMsg');

  if (!activeProblemId) {
    msg.textContent = 'Select a problem to view sessions.';
    div.innerHTML   = '';
    return;
  }

  msg.textContent = '';
  try {
    const r   = await fetch(`${API}/optimizer/problems/${activeProblemId}/sessions`);
    const arr = await r.json();
    div.innerHTML = '';

    if (!arr.length) {
      msg.textContent = 'No sessions yet for this problem.';
      return;
    }

    for (const s of arr) {
      const item = document.createElement('div');
      item.className = 'ov-sess-item' + (s.sessionId === activeSessionId ? ' active' : '');

      const cls     = algoClass(s.algorithm || 'hill_climbing');
      const algoStr = algoLabel(s.algorithm || 'hill_climbing');
      const fit     = s.bestFitness != null ? s.bestFitness.toFixed(4) : '—';
      const created = s.createdAt ? new Date(s.createdAt).toLocaleString() : '—';

      item.innerHTML = `
        <div class="si-header">
          <span class="ov-algo-badge ${cls}">${algoStr}</span>
          <span style="font-size:10px;color:var(--c-text-label)">${created}</span>
          <span class="ov-badge ${s.status}">${s.status}</span>
        </div>
        <div class="si-fitness">Fitness: ${fit}</div>
        <div class="si-iters">Iters: ${s.itersDone ?? '?'} · Evals: ${s.evalsDone ?? '?'}</div>`;

      item.addEventListener('click', () => {
        document.querySelectorAll('.ov-sess-item').forEach(el => el.classList.remove('active'));
        item.classList.add('active');
        showSessionDetail(s);
      });

      div.appendChild(item);
    }
  } catch (e) {
    msg.textContent = 'Failed to load sessions.';
    console.error(e);
  }
}

function showSessionDetail(s) {
  lastStatus = s;

  const sec  = document.getElementById('ov-sessDetailSection');
  const body = document.getElementById('ov-sessDetailBody');
  sec.style.display = '';

  let html = `
    <div style="margin-bottom:8px">
      <span class="ov-badge ${s.status}">${s.status}</span>
      &nbsp;<span style="font-size:11px;color:var(--c-text-label)">${algoLabel(s.algorithm || '?')} algorithm</span>
    </div>
    <table class="ov-result-table" style="margin-bottom:8px">
      <tr><td style="color:var(--c-text-label)">Best fitness</td><td>${fmtRatio(s.bestFitness)}</td></tr>
      <tr><td style="color:var(--c-text-label)">Baseline DANC</td><td class="ov-fitness-val">${s.baselineDANC?.toFixed(4) ?? '—'}</td></tr>
      <tr><td style="color:var(--c-text-label)">Iterations</td><td>${s.itersDone ?? '—'}</td></tr>
      <tr><td style="color:var(--c-text-label)">Evaluations</td><td>${s.evalsDone ?? '—'}</td></tr>
    </table>`;

  if (s.convergenceHistory && s.convergenceHistory.length > 1) {
    html += `<div style="font-size:10px;color:var(--c-text-label);margin-bottom:4px;text-transform:uppercase;letter-spacing:0.5px">Convergence</div>`;
    body.innerHTML = html;
    body.innerHTML += '<canvas id="ov-detailConvCanvas" style="display:block;width:100%;height:80px"></canvas>';
    drawConvergenceOn('ov-detailConvCanvas', s.convergenceHistory);
    return;
  }

  body.innerHTML = html;
}

function drawConvergenceOn(id, history) {
  const cvs = document.getElementById(id);
  if (!cvs) return;
  const ct  = cvs.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const w   = cvs.clientWidth, h = cvs.clientHeight;
  cvs.width = Math.floor(w * dpr); cvs.height = Math.floor(h * dpr);
  ct.scale(dpr, dpr);

  const minF = Math.min(...history.map(p => p.fit));
  const maxF = Math.max(...history.map(p => p.fit));
  const rng  = Math.max(maxF - minF, 0.001);
  const pad  = 8;

  ct.clearRect(0, 0, w, h);
  ct.strokeStyle = '#00c8f0'; ct.lineWidth = 1.5;
  ct.beginPath();
  for (let i = 0; i < history.length; i++) {
    const x = pad + (i / Math.max(history.length - 1, 1)) * (w - 2 * pad);
    const y = pad + (1 - (history[i].fit - minF) / rng) * (h - 2 * pad);
    if (i === 0) ct.moveTo(x, y); else ct.lineTo(x, y);
  }
  ct.stroke();
}

// ── Sensitivity import ────────────────────────────────────────────────────────
function ovImportSensitivity() {
  document.getElementById('ov-sensImportFile')?.click();
}

function ovOnSensFileSelected(e) {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = evt => {
    try {
      const data    = JSON.parse(evt.target.result);
      const results = data.results;
      if (!Array.isArray(results) || results.length === 0) {
        alert('No results found in sensitivity JSON. Run and export a sensitivity scan first.');
        return;
      }
      const N      = Math.max(1, parseInt(document.getElementById('ov-sensTopN')?.value) || 20);
      const sorted = [...results].sort((a, b) => (b.fitness ?? 0) - (a.fitness ?? 0));
      const top    = sorted.slice(0, N);
      candidates   = top.map(r => ({
        k: 'r', w: r.wayId,
        label: r.name || wayMeta[r.wayId]?.name || `Way ${r.wayId}`,
      }));
      renderCandidateList();
      const msg = document.getElementById('ov-sensImportMsg');
      if (msg) msg.textContent = `Loaded ${top.length} of ${results.length} roads (sorted by impact).`;
    } catch (err) {
      alert('Error parsing sensitivity JSON: ' + err.message);
    }
  };
  reader.readAsText(file);
  e.target.value = '';
}

// ── Export for thesis ─────────────────────────────────────────────────────────
function dlBlob(content, filename, type = 'application/json') {
  const blob = new Blob([content], { type });
  const url  = URL.createObjectURL(blob);
  const a    = document.createElement('a');
  a.href = url; a.download = filename; a.click();
  URL.revokeObjectURL(url);
}

async function ovExportSessionsJson() {
  if (!activeProblemId) { alert('Select a problem first.'); return; }
  try {
    const r   = await fetch(`${API}/optimizer/problems/${activeProblemId}/sessions`);
    const arr = await r.json();
    const pid  = activeProblemId.slice(0, 8);
    const date = new Date().toISOString().slice(0, 10);
    dlBlob(JSON.stringify(arr, null, 2), `optimizer_${pid}_${date}.json`);
  } catch (e) { alert('Export failed: ' + e.message); }
}

async function ovExportSessionsCsv() {
  if (!activeProblemId) { alert('Select a problem first.'); return; }
  try {
    const r   = await fetch(`${API}/optimizer/problems/${activeProblemId}/sessions`);
    const arr = await r.json();
    const pid  = activeProblemId.slice(0, 8);
    const date = new Date().toISOString().slice(0, 10);

    // Summary CSV — one row per session
    const maxPhases = Math.max(...arr.map(s => (s.phaseFitnesses || []).length), 0);
    const phaseHdrs = Array.from({length: maxPhases}, (_, i) => `phase${i+1}Fitness`).join(',');
    let sumCsv = `sessionId,algorithm,status,bestFitness,baselineDANC,phaseCount,roadCount,${phaseHdrs},itersDone,evalsDone,totalEvals,maxIter,parallel,runsPerEval,createdAt\n`;
    for (const s of arr) {
      const hp  = s.hyperparams || {};
      const pfs = s.phaseFitnesses || [];
      const pfCols = Array.from({length: maxPhases}, (_, i) => pfs[i] ?? '').join(',');
      const n   = s.bestAssignment?.length ?? (s.candidateRoadIds?.length ?? '');
      sumCsv += [
        s.sessionId, s.algorithm, s.status,
        s.bestFitness ?? '', s.baselineDANC ?? '',
        s.phaseCount ?? '', n, pfCols,
        s.itersDone ?? '', s.evalsDone ?? '', s.totalEvals ?? '',
        hp.maxIter ?? '', hp.maxParallel ?? '', hp.runsPerEval ?? 1,
        s.createdAt ?? '',
      ].join(',') + '\n';
    }

    // Convergence CSV — one row per (session, eval checkpoint)
    // evalNum is in sim-run units; divide by runsPerEval for logical-eval count
    let convCsv = 'sessionId,algorithm,phaseCount,roadCount,runsPerEval,evalNum,bestFitness\n';
    for (const s of arr) {
      const n = s.bestAssignment?.length ?? (s.candidateRoadIds?.length ?? '');
      const rpe = (s.hyperparams?.runsPerEval) ?? 1;
      for (const pt of (s.convergenceHistory || [])) {
        convCsv += `${s.sessionId},${s.algorithm},${s.phaseCount ?? ''},${n},${rpe},${pt.n},${pt.fit}\n`;
      }
    }

    dlBlob(sumCsv,  `optimizer_summary_${pid}_${date}.csv`,     'text/csv');
    dlBlob(convCsv, `optimizer_convergence_${pid}_${date}.csv`, 'text/csv');
  } catch (e) { alert('Export failed: ' + e.message); }
}

// ── Map PNG export ────────────────────────────────────────────────────────────

const PRINT_BG          = '#ffffff';
const PRINT_BORDER      = '#cdc8c1';
const PRINT_BUILDING    = '#e2ddd8';
const PRINT_BLDG_STROKE = '#cac5bf';
const PRINT_ROAD_BASE   = '#bfbbb4';
const PRINT_ROAD_DIM    = '#c9c5be';
const PRINT_NAME_FILL   = '#383838';
const PRINT_NAME_HALO   = 'rgba(255,255,255,0.93)';
const PRINT_TITLE       = '#181818';
const PRINT_SUBTITLE    = '#555';
const PRINT_FONT        = 'Arial, Helvetica, sans-serif';
// Saturated colors that read well on light/paper background
const PRINT_PHASE_COLS     = ['#c41832', '#0077b3', '#6b1ec0', '#117a38', '#c86400'];
// Pedestrianization print colors
const PRINT_PED_CANDIDATE  = '#c86400';  // all-candidates panel: amber
const PRINT_PED_CLOSED     = '#c41832';  // closed-roads panel: red
// Phase fitness quality — phased construction only cares about baseline (f < 1.0 = good)
const PRINT_Q_GOOD = '#1a7a35';
const PRINT_Q_WARN = '#9a5200';
function phaseQualityColor(f) {
  if (f == null || !isFinite(f)) return PRINT_SUBTITLE;
  return f < 1.0 ? PRINT_Q_GOOD : PRINT_Q_WARN;
}
function phaseQualityLabel(f) {
  if (f == null || !isFinite(f)) return '';
  return f < 1.0 ? '✓ Below baseline' : '↑ Above baseline';
}
// Algo badge colours
const PRINT_ALGO_COL = { hill_climbing: '#c86400', simulated_annealing: '#0077b3', genetic_algorithm: '#6b1ec0' };

// Convert world → panel-local pixel  (y-flip baked in)
function w2s(wx, wy, pc) {
  return [
    pc.pad + (wx - pc.minX) * pc.scale,
    pc.pad + pc.mapPxH - (wy - pc.minY) * pc.scale,
  ];
}

function exportDrawBuildings(ctx, pc) {
  if (!buildingData.length) return;
  ctx.fillStyle   = PRINT_BUILDING;
  ctx.strokeStyle = PRINT_BLDG_STROKE;
  ctx.lineWidth   = 0.6;
  for (const b of buildingData) {
    for (const ring of b.outer) {
      if (ring.length < 3) continue;
      ctx.beginPath();
      for (let i = 0; i < ring.length; i++) {
        const [sx, sy] = w2s(ring[i].x, ring[i].y, pc);
        const cx = pc.ox + sx, cy = pc.oy + pc.headerH + sy;
        i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy);
      }
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }
  }
}

function exportDrawRoads(ctx, pc, highlightPhase) {
  // Build candidate phase lookups
  const wayPh  = new Map();
  const lanePh = new Map();
  if (lastStatus?.bestAssignment) {
    for (let i = 0; i < candidates.length; i++) {
      const c = candidates[i], ph = lastStatus.bestAssignment[i] ?? 0;
      c.k === 'r' ? wayPh.set(c.w, ph) : lanePh.set(`${c.w}:${c.i}`, ph);
    }
  }

  // Two-pass: base roads first, candidates on top
  for (const pass of [0, 1]) {
    for (const { wayId, laneIdx, points, type } of laneData) {
      const lp = lanePh.get(`${wayId}:${laneIdx}`);
      const wp = wayPh.get(wayId);
      const ph = lp !== undefined ? lp : wp;
      const isCandidate = ph !== undefined;
      if (pass === 0 && isCandidate)  continue;
      if (pass === 1 && !isCandidate) continue;

      let color, lw;
      if (isCandidate) {
        if (highlightPhase === -1 || ph === highlightPhase) {
          color = PRINT_PHASE_COLS[ph % PRINT_PHASE_COLS.length];
          lw    = type <= 1 ? 4.5 : type <= 2 ? 3.8 : type <= 3 ? 3.2 : 2.6;
        } else {
          color = PRINT_ROAD_DIM;
          lw    = type <= 1 ? 2.2 : 1.6;
        }
      } else {
        color = PRINT_ROAD_BASE;
        lw    = type <= 1 ? 2.4 : type <= 2 ? 2.0 : type <= 3 ? 1.4 : type <= 4 ? 1.0 : 0.6;
      }

      ctx.beginPath();
      ctx.strokeStyle = color;
      ctx.lineWidth   = lw;
      ctx.lineCap = 'round'; ctx.lineJoin = 'round';
      let first = true;
      for (const [wx, wy] of points) {
        const [sx, sy] = w2s(wx, wy, pc);
        const cx = pc.ox + sx, cy = pc.oy + pc.headerH + sy;
        first ? (ctx.moveTo(cx, cy), first = false) : ctx.lineTo(cx, cy);
      }
      ctx.stroke();
    }
  }
}

// mode: 'all' = show every candidate as amber; 'closed' = closed=red, open=dimmed
function exportDrawRoadsPed(ctx, pc, mode) {
  const wayAssign  = new Map();
  const laneAssign = new Map();
  if (lastStatus?.bestAssignment) {
    for (let i = 0; i < candidates.length; i++) {
      const c = candidates[i], v = lastStatus.bestAssignment[i] ?? 0;
      c.k === 'r' ? wayAssign.set(c.w, v) : laneAssign.set(`${c.w}:${c.i}`, v);
    }
  }

  for (const pass of [0, 1]) {
    for (const { wayId, laneIdx, points, type } of laneData) {
      const la = laneAssign.get(`${wayId}:${laneIdx}`);
      const wa = wayAssign.get(wayId);
      const assign     = la !== undefined ? la : wa;
      const isCandidate = assign !== undefined;
      if (pass === 0 && isCandidate)  continue;
      if (pass === 1 && !isCandidate) continue;

      let color, lw;
      if (isCandidate) {
        if (mode === 'all') {
          color = PRINT_PED_CANDIDATE;
          lw    = type <= 1 ? 4.5 : type <= 2 ? 3.8 : type <= 3 ? 3.2 : 2.6;
        } else {
          // 'closed' mode: highlight closed, dim open
          if (assign === 1) {
            color = PRINT_PED_CLOSED;
            lw    = type <= 1 ? 4.5 : type <= 2 ? 3.8 : type <= 3 ? 3.2 : 2.6;
          } else {
            color = PRINT_ROAD_DIM;
            lw    = type <= 1 ? 2.2 : 1.6;
          }
        }
      } else {
        color = PRINT_ROAD_BASE;
        lw    = type <= 1 ? 2.4 : type <= 2 ? 2.0 : type <= 3 ? 1.4 : type <= 4 ? 1.0 : 0.6;
      }

      ctx.beginPath();
      ctx.strokeStyle = color;
      ctx.lineWidth   = lw;
      ctx.lineCap = 'round'; ctx.lineJoin = 'round';
      let first = true;
      for (const [wx, wy] of points) {
        const [sx, sy] = w2s(wx, wy, pc);
        const cx = pc.ox + sx, cy = pc.oy + pc.headerH + sy;
        first ? (ctx.moveTo(cx, cy), first = false) : ctx.lineTo(cx, cy);
      }
      ctx.stroke();
    }
  }
}

function exportDrawNames(ctx, pc, maxType, fontSize) {
  const drawn = new Set();
  if (!fontSize) fontSize = Math.max(12, Math.min(20, pc.w * 0.008));
  ctx.font = `${fontSize}px ${PRINT_FONT}`;
  for (const { wayId, laneIdx, points, type } of laneData) {
    if (type > maxType || laneIdx !== 0 || drawn.has(wayId)) continue;
    const name = wayMeta[wayId]?.name;
    if (!name || name.startsWith('Way ')) continue;
    drawn.add(wayId);
    const screenPts = points.map(([wx, wy]) => {
      const [sx, sy] = w2s(wx, wy, pc);
      return [pc.ox + sx, pc.oy + pc.headerH + sy];
    });
    exportPlaceName(ctx, screenPts, name, fontSize);
  }
}

function exportPlaceName(ctx, pts, name, fontSize) {
  if (pts.length < 2) return;
  let total = 0;
  const segs = [];
  for (let i = 1; i < pts.length; i++) {
    const dx = pts[i][0] - pts[i-1][0], dy = pts[i][1] - pts[i-1][1];
    const len = Math.hypot(dx, dy);
    segs.push({ x1: pts[i-1][0], y1: pts[i-1][1], x2: pts[i][0], y2: pts[i][1], len });
    total += len;
  }
  if (total < ctx.measureText(name).width + 16) return;

  let cum = 0, half = total / 2;
  for (const seg of segs) {
    if (cum + seg.len >= half) {
      const t  = (half - cum) / seg.len;
      const mx = seg.x1 + t * (seg.x2 - seg.x1);
      const my = seg.y1 + t * (seg.y2 - seg.y1);
      let angle = Math.atan2(seg.y2 - seg.y1, seg.x2 - seg.x1);
      if (angle > Math.PI / 2 || angle < -Math.PI / 2) angle += Math.PI;
      ctx.save();
      ctx.translate(mx, my);
      ctx.rotate(angle);
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.strokeStyle = PRINT_NAME_HALO;
      ctx.lineWidth   = Math.ceil(fontSize / 3.2);
      ctx.strokeText(name, 0, 0);
      ctx.fillStyle = PRINT_NAME_FILL;
      ctx.fillText(name, 0, 0);
      ctx.restore();
      return;
    }
    cum += seg.len;
  }
}

function exportDrawFrame(ctx, pc, opts) {
  const K = opts.K || lastStatus?.phaseFitnesses?.length || lastStatus?.phaseCount || 2;

  // ── Panel outer border ────────────────────────────────────────────────────
  ctx.strokeStyle = PRINT_BORDER;
  ctx.lineWidth   = 2;
  ctx.strokeRect(pc.ox + 1, pc.oy + 1, pc.w - 2, pc.totalH - 2);

  // ── Header ────────────────────────────────────────────────────────────────
  if (opts.isOverview || opts.isPedCandidates || opts.isPedClosed) {
    const titleSz = Math.min(58, Math.max(36, pc.w * 0.024));
    const subSz   = Math.min(30, Math.max(20, pc.w * 0.012));
    const titleY  = pc.oy + pc.headerH * 0.36;
    const subY    = pc.oy + pc.headerH * 0.73;

    ctx.fillStyle    = PRINT_TITLE;
    ctx.font         = `bold ${titleSz}px ${PRINT_FONT}`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(opts.title, pc.ox + pc.w / 2, titleY);

    const algo     = lastStatus?.algorithm || 'hill_climbing';
    const bestF    = lastStatus?.bestFitness;
    const blDANC   = lastStatus?.baselineDANC;
    const maxIter_ = lastStatus?.maxIter ?? lastStatus?.hyperparams?.maxIter;
    const runsR    = lastStatus?.runsPerEval ?? lastStatus?.hyperparams?.runsPerEval ?? 1;
    const shortId  = lastStatus?.sessionId?.slice(0, 8) || '';
    const K_       = lastStatus?.phaseCount ?? K;

    let subParts;
    if (opts.isPedCandidates || opts.isPedClosed) {
      subParts = [
        algoLabel(algo) + ' algorithm',
        maxIter_ != null ? `${maxIter_} iter` : null,
        runsR > 1 ? `${runsR}× avg` : null,
        bestF != null ? `Best f = ${bestF.toFixed(4)}` : null,
        blDANC != null ? `Baseline: ${blDANC.toFixed(4)}` : null,
        shortId ? `[${shortId}]` : null,
      ].filter(Boolean).join('  ·  ');
    } else {
      subParts = [
        algoLabel(algo) + ' algorithm',
        K_ ? `K=${K_} phases` : null,
        maxIter_ != null ? `${maxIter_} iter` : null,
        runsR > 1 ? `${runsR}× avg` : null,
        bestF != null ? `Best f = ${bestF.toFixed(4)}` : null,
        blDANC != null ? `Baseline: ${blDANC.toFixed(4)}` : null,
        shortId ? `[${shortId}]` : null,
      ].filter(Boolean).join('  ·  ');
    }

    ctx.fillStyle    = PRINT_SUBTITLE;
    ctx.font         = `${subSz}px ${PRINT_FONT}`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(subParts, pc.ox + pc.w / 2, subY);

    // Accent bar for pedestrianization panels
    if (opts.isPedCandidates) {
      ctx.fillStyle = PRINT_PED_CANDIDATE;
      ctx.fillRect(pc.ox, pc.oy + pc.headerH - 6, pc.w, 6);
    } else if (opts.isPedClosed) {
      ctx.fillStyle = PRINT_PED_CLOSED;
      ctx.fillRect(pc.ox, pc.oy + pc.headerH - 6, pc.w, 6);
    }
  } else {
    const titleSz  = Math.min(52, Math.max(30, pc.w * 0.075));
    const phaseIdx = opts.phaseIdx ?? 0;
    const barCol   = PRINT_PHASE_COLS[phaseIdx % PRINT_PHASE_COLS.length];

    ctx.fillStyle    = PRINT_TITLE;
    ctx.font         = `bold ${titleSz}px ${PRINT_FONT}`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(opts.title, pc.ox + pc.w / 2, pc.oy + pc.headerH / 2);

    // Phase-colour accent bar across bottom of header
    ctx.fillStyle = barCol;
    ctx.fillRect(pc.ox, pc.oy + pc.headerH - 6, pc.w, 6);
  }

  // Separator line
  ctx.strokeStyle = PRINT_BORDER;
  ctx.lineWidth   = 1;
  ctx.beginPath();
  ctx.moveTo(pc.ox, pc.oy + pc.headerH);
  ctx.lineTo(pc.ox + pc.w, pc.oy + pc.headerH);
  ctx.stroke();

  // ── Footer ────────────────────────────────────────────────────────────────
  const footerTop  = pc.oy + pc.headerH + pc.pad + pc.mapPxH + pc.pad;
  const footerMidY = footerTop + pc.footerH / 2;

  if (opts.isPedCandidates) {
    // Single chip: "N candidate roads"
    const chipH   = Math.min(52, Math.max(36, pc.footerH * 0.42));
    const chipW   = Math.min(520, Math.max(200, pc.w * 0.22));
    const ly      = footerMidY - chipH / 2;
    const lx      = pc.ox + (pc.w - chipW) / 2;
    const cFontSz = Math.min(26, Math.max(16, chipW * 0.16));
    ctx.fillStyle = PRINT_PED_CANDIDATE;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(lx, ly, chipW, chipH, 6); else ctx.rect(lx, ly, chipW, chipH);
    ctx.fill();
    ctx.fillStyle    = '#fff';
    ctx.font         = `bold ${cFontSz}px ${PRINT_FONT}`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${opts.candCount} candidate road${opts.candCount !== 1 ? 's' : ''}`, lx + chipW / 2, footerMidY);
  } else if (opts.isPedClosed) {
    // Two chips: "X closed / N total" and fitness
    const chipH   = Math.min(52, Math.max(36, pc.footerH * 0.42));
    const chipW   = Math.min(480, Math.max(180, pc.w * 0.20));
    const gap     = 16;
    const totalCW = 2 * chipW + gap;
    const ly      = footerMidY - chipH / 2;
    let lx        = pc.ox + (pc.w - totalCW) / 2;
    const cFontSz = Math.min(24, Math.max(14, chipW * 0.15));

    ctx.fillStyle = PRINT_PED_CLOSED;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(lx, ly, chipW, chipH, 6); else ctx.rect(lx, ly, chipW, chipH);
    ctx.fill();
    ctx.fillStyle    = '#fff';
    ctx.font         = `bold ${cFontSz}px ${PRINT_FONT}`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${opts.closedCount} / ${opts.candCount} closed`, lx + chipW / 2, footerMidY);

    lx += chipW + gap;
    const f = opts.fitness;
    const qCol = phaseQualityColor(f);
    ctx.fillStyle = qCol;
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(lx, ly, chipW, chipH, 6); else ctx.rect(lx, ly, chipW, chipH);
    ctx.fill();
    ctx.fillStyle    = '#fff';
    ctx.font         = `bold ${cFontSz}px ${PRINT_FONT}`;
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(f != null ? `f = ${f.toFixed(4)}` : '—', lx + chipW / 2, footerMidY);
  } else if (opts.isOverview) {
    // K phase legend chips — centered, with per-phase fitness
    const chipH   = Math.min(52, Math.max(36, pc.footerH * 0.42));
    const chipW   = Math.min(380, Math.max(120, (pc.w * 0.90) / K - 12));
    const gap     = 12;
    const totalCW = K * (chipW + gap) - gap;
    let lx        = pc.ox + (pc.w - totalCW) / 2;
    const ly      = footerMidY - chipH / 2;
    const cFontSz = Math.min(26, Math.max(16, chipW * 0.19));

    for (let k = 0; k < K; k++) {
      const col = PRINT_PHASE_COLS[k % PRINT_PHASE_COLS.length];
      ctx.fillStyle = col;
      ctx.beginPath();
      if (ctx.roundRect) ctx.roundRect(lx, ly, chipW, chipH, 6); else ctx.rect(lx, ly, chipW, chipH);
      ctx.fill();
      const fit = lastStatus?.phaseFitnesses?.[k];
      ctx.fillStyle    = '#fff';
      ctx.font         = `bold ${cFontSz}px ${PRINT_FONT}`;
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(`Phase ${k + 1}${fit != null ? '  f=' + fit.toFixed(3) : ''}`, lx + chipW / 2, footerMidY);
      lx += chipW + gap;
    }
  } else {
    // Large bold fitness number + quality label
    const f = opts.fitness;
    if (f != null) {
      const qCol    = phaseQualityColor(f);
      const qLabel  = phaseQualityLabel(f);
      const fSz     = Math.min(62, Math.max(40, pc.w * 0.088));
      const lSz     = Math.min(30, Math.max(20, pc.w * 0.042));
      const lineGap = 6;
      const blockH  = fSz + lineGap + lSz;
      const fY      = footerMidY - blockH / 2 + fSz / 2;
      const lY      = fY + fSz / 2 + lineGap + lSz / 2;

      ctx.fillStyle    = qCol;
      ctx.font         = `bold ${fSz}px ${PRINT_FONT}`;
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(`f = ${f.toFixed(4)}`, pc.ox + pc.w / 2, fY);

      ctx.fillStyle    = qCol;
      ctx.font         = `${lSz}px ${PRINT_FONT}`;
      ctx.textAlign    = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(qLabel, pc.ox + pc.w / 2, lY);
    }
  }
}

function ovExportMapPng() {
  if (!lastStatus?.bestAssignment?.length) {
    alert('Run the optimizer first — no result to export.');
    return;
  }

  // Full map bounds (used as fallback clamp)
  let fullMinX = Infinity, fullMaxX = -Infinity, fullMinY = Infinity, fullMaxY = -Infinity;
  for (const { points } of laneData) {
    for (const [x, y] of points) {
      if (x < fullMinX) fullMinX = x; if (x > fullMaxX) fullMaxX = x;
      if (y < fullMinY) fullMinY = y; if (y > fullMaxY) fullMaxY = y;
    }
  }

  let vMinX, vMaxX, vMinY, vMaxY;
  if (exportViewport) {
    // Use the interactively set viewport directly
    vMinX = exportViewport.minX; vMaxX = exportViewport.maxX;
    vMinY = exportViewport.minY; vMaxY = exportViewport.maxY;
  } else {
    // Fall back to candidate bounds + padding
    const candWayIds = new Set(candidates.map(c => c.w));
    let cMinX = Infinity, cMaxX = -Infinity, cMinY = Infinity, cMaxY = -Infinity;
    for (const { wayId, points } of laneData) {
      if (!candWayIds.has(wayId)) continue;
      for (const [x, y] of points) {
        if (x < cMinX) cMinX = x; if (x > cMaxX) cMaxX = x;
        if (y < cMinY) cMinY = y; if (y > cMaxY) cMaxY = y;
      }
    }
    if (!isFinite(cMinX)) { cMinX = fullMinX; cMaxX = fullMaxX; cMinY = fullMinY; cMaxY = fullMaxY; }
    const padPct = Math.max(0, Math.min(80, parseFloat(document.getElementById('ov-exportPad')?.value) || 20)) / 100;
    const padX   = (cMaxX - cMinX) * padPct;
    const padY   = (cMaxY - cMinY) * padPct;
    vMinX = Math.max(fullMinX, cMinX - padX);
    vMaxX = Math.min(fullMaxX, cMaxX + padX);
    vMinY = Math.max(fullMinY, cMinY - padY);
    vMaxY = Math.min(fullMaxY, cMaxY + padY);
  }
  const viewW = vMaxX - vMinX, viewH = vMaxY - vMinY;
  if (viewW < 1 || viewH < 1) { alert('Export area is too small — draw a larger rectangle.'); return; }

  const TOTAL_W   = 2400;
  const PAD       = 40;
  const HEADER_H  = 140;
  const FOOTER_H  = 110;
  const PANEL_GAP = 20;

  const currentScenario = lastStatus?.scenario || scenario;

  const ovScale  = (TOTAL_W - 2 * PAD) / viewW;
  const ovMapPxH = viewH * ovScale;
  const ovTotalH = HEADER_H + PAD + ovMapPxH + PAD + FOOTER_H;

  const mkPc = (ox, oy, w, scale, mapPxH) => ({
    ox, oy, w, scale, mapPxH,
    pad: PAD, headerH: HEADER_H, footerH: FOOTER_H,
    totalH: HEADER_H + PAD + mapPxH + PAD + FOOTER_H,
    minX: vMinX, minY: vMinY,
  });

  const cvs = document.createElement('canvas');
  const ctx = cvs.getContext('2d');

  if (currentScenario === 'pedestrianization') {
    // Two stacked full-width panels: candidates overview, then closed roads
    const CANVAS_H = Math.ceil(ovTotalH) * 2 + PANEL_GAP;
    cvs.width = TOTAL_W; cvs.height = CANVAS_H;
    ctx.fillStyle = PRINT_BG;
    ctx.fillRect(0, 0, TOTAL_W, CANVAS_H);

    const fontSz      = Math.max(36, Math.min(52, TOTAL_W * 0.018));
    const closedCount = lastStatus.bestAssignment.reduce((s, v) => s + (v === 1 ? 1 : 0), 0);
    const candCount   = candidates.length;

    const drawPanelPed = (pc, mode, frameOpts) => {
      ctx.save();
      ctx.beginPath();
      ctx.rect(pc.ox, pc.oy + pc.headerH, pc.w, pc.pad + pc.mapPxH + pc.pad);
      ctx.clip();
      exportDrawBuildings(ctx, pc);
      exportDrawRoadsPed(ctx, pc, mode);
      exportDrawNames(ctx, pc, 2, fontSz);
      ctx.restore();
      exportDrawFrame(ctx, pc, frameOpts);
    };

    // Top: all candidate roads
    const candPc = mkPc(0, 0, TOTAL_W, ovScale, ovMapPxH);
    drawPanelPed(candPc, 'all', {
      title: 'Pedestrianization — Candidate Roads',
      isPedCandidates: true,
      candCount,
    });

    // Bottom: only closed roads
    const closedPc = mkPc(0, Math.ceil(ovTotalH) + PANEL_GAP, TOTAL_W, ovScale, ovMapPxH);
    drawPanelPed(closedPc, 'closed', {
      title: 'Pedestrianization — Closed Roads',
      isPedClosed: true,
      closedCount,
      candCount,
      fitness: lastStatus.bestFitness,
    });

    const a    = document.createElement('a');
    a.download = `ped_map_${(activeProblemId || 'export').slice(0, 8)}_${new Date().toISOString().slice(0, 10)}.png`;
    a.href     = cvs.toDataURL('image/png');
    a.click();
    return;
  }

  // ── Phased construction layout ────────────────────────────────────────────
  const K = lastStatus.phaseFitnesses?.length || lastStatus.phaseCount || 2;

  const phW      = Math.floor(TOTAL_W / K);
  const phScale  = (phW - 2 * PAD) / viewW;
  const phMapPxH = viewH * phScale;
  const phTotalH = HEADER_H + PAD + phMapPxH + PAD + FOOTER_H;

  const CANVAS_H = Math.ceil(ovTotalH) + PANEL_GAP + Math.ceil(phTotalH);
  cvs.width = TOTAL_W; cvs.height = CANVAS_H;
  ctx.fillStyle = PRINT_BG;
  ctx.fillRect(0, 0, TOTAL_W, CANVAS_H);

  // Draw one panel: clip map area, draw layers, then frame on top
  const drawPanel = (pc, highlightPhase, frameOpts, nameMaxType, nameFontSz) => {
    ctx.save();
    ctx.beginPath();
    ctx.rect(pc.ox, pc.oy + pc.headerH, pc.w, pc.pad + pc.mapPxH + pc.pad);
    ctx.clip();
    exportDrawBuildings(ctx, pc);
    exportDrawRoads(ctx, pc, highlightPhase);
    exportDrawNames(ctx, pc, nameMaxType, nameFontSz);
    ctx.restore();
    exportDrawFrame(ctx, pc, frameOpts);
  };

  // ── Overview (all phases colored, primary road labels) ────────────────────
  const ovPc     = mkPc(0, 0, TOTAL_W, ovScale, ovMapPxH);
  const ovFontSz = Math.max(36, Math.min(52, TOTAL_W * 0.018));
  drawPanel(ovPc, -1, { title: 'Phased Construction — Overview', isOverview: true, K }, 2, ovFontSz);

  // ── Phase panels (one phase highlighted per panel) ────────────────────────
  const phaseY   = Math.ceil(ovTotalH) + PANEL_GAP;
  const phFontSz = Math.max(20, Math.min(32, phW * 0.048));
  for (let k = 0; k < K; k++) {
    const phPc  = mkPc(k * phW, phaseY, phW, phScale, phMapPxH);
    const phFit = lastStatus?.phaseFitnesses?.[k];
    drawPanel(phPc, k, { title: `Phase ${k + 1}`, fitness: phFit, phaseIdx: k, K }, 2, phFontSz);
  }

  const a    = document.createElement('a');
  a.download = `phased_map_${(activeProblemId || 'export').slice(0, 8)}_${new Date().toISOString().slice(0, 10)}.png`;
  a.href     = cvs.toDataURL('image/png');
  a.click();
}

// ── Boot — called by app.js when user switches to optimizer tab ───────────────
window.__initOptimizer = async () => {
  initMap();
  await loadProblems();
  renderCandidateList();

  document.getElementById('ov-sensImportBtn')?.addEventListener('click', ovImportSensitivity);
  document.getElementById('ov-sensImportFile')?.addEventListener('change', ovOnSensFileSelected);
  document.getElementById('ov-exportJsonBtn')?.addEventListener('click', ovExportSessionsJson);
  document.getElementById('ov-exportCsvBtn')?.addEventListener('click', ovExportSessionsCsv);
  document.getElementById('ov-exportMapPngBtn')?.addEventListener('click', ovExportMapPng);
  document.getElementById('ov-histExportMapPngBtn')?.addEventListener('click', ovExportMapPng);
  document.getElementById('ov-exportAreaBtn')?.addEventListener('click', toggleExportViewMode);
  document.getElementById('ov-histExportAreaBtn')?.addEventListener('click', toggleExportViewMode);
};
