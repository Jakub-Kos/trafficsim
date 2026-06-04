/*
 * Note: The core simulation loop, canvas architecture, and render pipeline
 * integration are the author's own work. Claude (Anthropic) was used under
 * human supervision to integrate new view modes and UI features on top of
 * this foundation.
 */
// frontend/main.js
import { makeViewport } from './viewport.js';
import {
    WebGLRenderer,
    drawDebugLayers,
    drawVehicleIds,
    drawSelectedDebug,
    drawRawOSM,
    drawParkingSpots,
} from './renderer.js';
import { fetchMap, fetchParkingSpots, connectLive, setSimSpeed, setRoadClosed, fetchVehicleDetail, sendViewport, fetchHeatmap, fetchLaneStats, setStreamMode } from './net.js';
import { initPerf, perfBegin, perfMark, perfEnd } from './perf.js';
import { SceneView } from './scene.js';

// ── Canvas setup ──────────────────────────────────────────────────────────────
const bgCanvas  = document.getElementById('bgCanvas');
const glCanvas  = document.getElementById('glCanvas');
const dbgCanvas = document.getElementById('dbgCanvas');
const dbgCtx    = dbgCanvas.getContext('2d');
const hud       = document.getElementById('hud');

initPerf();

// WebGL renderer
const glRenderer = new WebGLRenderer(glCanvas);

// Viewport (dbgCanvas is the interaction surface)
const { vp, step: vpStep, getViewMatrix, worldToScreen, screenToWorld, setUserControlCallback } =
    makeViewport(dbgCanvas, glCanvas);

// Scene view (background tile renderer)
const sceneView = new SceneView(bgCanvas, vp, worldToScreen);

// ── Inspector DOM ─────────────────────────────────────────────────────────────
const inspector = document.getElementById('inspector');
document.getElementById('inspectorClose').addEventListener('click', () => {
    selectedId = null; detailedInfo = null; inspector.classList.remove('active');
});

const domId       = document.getElementById('ins-id');
const domState    = document.getElementById('ins-state');
const domSpeed    = document.getElementById('ins-speed');
const domAccel    = document.getElementById('ins-accel');
const domLeader   = document.getElementById('ins-leader');
const domGap      = document.getElementById('ins-gap');
const domHeadway  = document.getElementById('ins-headway');
const domLane     = document.getElementById('ins-lane');
const domRoute    = document.getElementById('ins-route');
const domProgress = document.getElementById('ins-progress');
const domLaneLen  = document.getElementById('ins-lanelen');
const domInInt    = document.getElementById('ins-inint');
const domBlocked  = document.getElementById('ins-blocked');

// ── Metrics DOM ───────────────────────────────────────────────────────────────
const domMetTotal      = document.getElementById('met-total');
const domMetQueue      = document.getElementById('met-queue');
const domMetQPct       = document.getElementById('met-qpct');
const domMetIntersect  = document.getElementById('met-intersect');
const domMetYielding   = document.getElementById('met-yielding');
const domMetSpeed      = document.getElementById('met-speed');
const domMetMaxSpeed   = document.getElementById('met-maxspeed');
const domMetGap        = document.getElementById('met-gap');
const domMetSpawned    = document.getElementById('met-spawned');
const domMetCompleted  = document.getElementById('met-completed');
const domMetThroughput = document.getElementById('met-throughput');
const domMetTravel     = document.getElementById('met-traveltime');
const domMetStepMs     = document.getElementById('met-stepms');
const domMetMaxScale   = document.getElementById('met-maxscale');

const graphCanvas = document.getElementById('metricsGraph');
const graphCtx    = graphCanvas.getContext('2d');
const metricsHistory = [];
const MAX_HISTORY = 150;

// Metrics panel collapse (metricsToggle may not exist in app shell — dock handles collapse)
const metricsBody   = document.getElementById('metricsBody');
const metricsToggle = document.getElementById('metricsToggle');
let metricsCollapsed = false;
if (metricsToggle) {
    metricsToggle.addEventListener('click', (e) => {
        e.stopPropagation();
        metricsCollapsed = !metricsCollapsed;
        if (metricsBody) metricsBody.style.display = metricsCollapsed ? 'none' : '';
        metricsToggle.textContent = metricsCollapsed ? '▶' : '◀';
    });
}

// ── State ─────────────────────────────────────────────────────────────────────
let mapData          = null;
let vehicleTargets   = new Map();
let vehicleRenderStates = new Map();
let selectedId       = null;
let followMode       = false;
let currentSimTime   = 0;
let detailedInfo     = null;
let lastIntersections = [];
let isPaused         = true;   // start paused; synced from backend on load
let lastSpeedBeforePause = 1.0;
let lastViewportSendTime = 0;

// Road editor state
let roadEditMode = false;
const closedWays = new Set();

// Parking spots (fetched once after map load)
let parkingSpots = [];

// Pipeline export
let simExportMode     = false;
let simExportViewport = null;
let _simVpDrag        = null;

// ── View mode ────────────────────────────────────────────────────────────────
// 'agents' = individual vehicles (default)
// 'density' = grid-based vehicle density heatmap
// 'congestion' = per-lane congestion overlay
let viewMode = 'agents';
let laneStatMap = new Map();        // laneId -> { count, stuck, speed }
let laneSpeedLimitMap = new Map();  // laneId -> speedLimitMps (built once from map data)
let densityCells = null;            // density object from WS: {minX,minY,maxX,maxY,resX,resY,cells[]}

// Scene mode — overlay background tiles behind vehicles (independent of viewMode)
let sceneMode = false;

// ── Debug options ─────────────────────────────────────────────────────────────
const debugOptions = {
    master: true, hitbox: false, conflict: false, yield: false,
    ids: false, lights: true, rawosm: false, spawnrates: false,
};
['master', 'hitbox', 'conflict', 'yield', 'ids', 'lights', 'rawosm', 'spawnrates'].forEach(key => {
    const cb = document.getElementById('dbg-' + key);
    if (!cb) return;
    cb.checked = debugOptions[key];
    cb.addEventListener('change', () => { debugOptions[key] = cb.checked; });
});

// ── Interaction mode (pan vs select) ─────────────────────────────────────────
// 'select' = pan + click to pick a vehicle
// 'pan'    = pan only, clicks do nothing
let interactionMode = 'select';

const toolSelectBtn = document.getElementById('toolSelect');
const toolPanBtn    = document.getElementById('toolPan');

function setInteractionMode(mode) {
    interactionMode = mode;
    toolSelectBtn.classList.toggle('active', mode === 'select');
    toolPanBtn.classList.toggle('active',    mode === 'pan');
    dbgCanvas.style.cursor = mode === 'pan' ? 'grab' : 'default';
}

toolSelectBtn.addEventListener('click', () => setInteractionMode('select'));
toolPanBtn.addEventListener('click',    () => setInteractionMode('pan'));

// Recenter button (same as pressing F)
document.getElementById('recenterBtn').addEventListener('click', () => {
    followMode = false;
    vp.autofit = false;
    if (mapData) centerViewOnMap(mapData);
});

// Road editor toggle
const roadEditBtn    = document.getElementById('roadEditBtn');
const closureCountEl = document.getElementById('closureCount');
roadEditBtn.addEventListener('click', () => {
    roadEditMode = !roadEditMode;
    roadEditBtn.classList.toggle('active', roadEditMode);
    dbgCanvas.style.cursor = roadEditMode ? 'crosshair' : (interactionMode === 'pan' ? 'grab' : 'default');
});

// ── View mode buttons ─────────────────────────────────────────────────────────
const viewBtns = {
    agents:     document.getElementById('viewAgents'),
    density:    document.getElementById('viewDensity'),
    congestion: document.getElementById('viewCongestion'),
};

function setViewMode(mode) {
    viewMode = mode;
    for (const [k, btn] of Object.entries(viewBtns)) {
        btn.classList.toggle('active', k === mode);
    }

    // Clear stale heatmap data
    laneStatMap = new Map();
    densityCells = null;

    setStreamMode(mode);
    if (mode !== 'agents') {
        // Clear vehicle render state so they disappear immediately
        vehicleTargets = new Map();
        vehicleRenderStates = new Map();
        if (selectedId != null) { selectedId = null; inspector.classList.remove('active'); }
    }
}

for (const [mode, btn] of Object.entries(viewBtns)) {
    btn.addEventListener('click', () => setViewMode(mode));
}

// ── Scene mode button ─────────────────────────────────────────────────────────
const sceneBtn         = document.getElementById('viewScene');
const sceneSourcePanel = document.getElementById('sceneSourcePanel');
const sceneSatBtn      = document.getElementById('sceneSatellite');
const sceneOsmBtn      = document.getElementById('sceneOsm');

function setSceneMode(active) {
    sceneMode = active;
    sceneBtn.classList.toggle('active', active);
    sceneSourcePanel.style.display = active ? '' : 'none';
    if (!active) {
        sceneView.ctx.clearRect(0, 0, bgCanvas.width, bgCanvas.height);
    }
}

sceneBtn.addEventListener('click', () => setSceneMode(!sceneMode));

sceneSatBtn.addEventListener('click', () => {
    sceneView.setSource('satellite');
    sceneSatBtn.classList.add('active');
    sceneOsmBtn.classList.remove('active');
});
sceneOsmBtn.addEventListener('click', () => {
    sceneView.setSource('osm');
    sceneOsmBtn.classList.add('active');
    sceneSatBtn.classList.remove('active');
});

const sceneOpacitySlider = document.getElementById('sceneOpacity');
const sceneOpacityVal    = document.getElementById('sceneOpacityVal');
sceneOpacitySlider.addEventListener('input', () => {
    const pct = parseInt(sceneOpacitySlider.value);
    sceneOpacityVal.textContent = pct + '%';
    bgCanvas.style.opacity = pct / 100;
});

// Heatmap data arrives via the WS broadcast — see connectLive callback below.

// ── Spawn panel ───────────────────────────────────────────────────────────────
const spawnEnabledCb    = document.getElementById('spawnEnabled');
const spawnRegionList   = document.getElementById('spawnRegionList');
const spawnBody         = document.getElementById('spawnBody');
const spawnToggleCollapse = document.getElementById('spawnToggleCollapse');
let spawnPanelCollapsed = false;
let lastSpawnData = null;  // cached response from /spawning, used by the spawn-rates debug layer

spawnToggleCollapse.addEventListener('click', (e) => {
    e.stopPropagation();
    spawnPanelCollapsed = !spawnPanelCollapsed;
    spawnBody.style.display = spawnPanelCollapsed ? 'none' : '';
    spawnToggleCollapse.textContent = spawnPanelCollapsed ? '▶' : '◀';
});

spawnEnabledCb.addEventListener('change', async () => {
    try {
        await fetch('/spawning', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ enabled: spawnEnabledCb.checked }),
        });
    } catch {}
});

async function refreshSpawnInfo() {
    try {
        const r = await fetch('/spawning');
        if (!r.ok) return;
        const data = await r.json();
        lastSpawnData = data;
        spawnEnabledCb.checked = data.enabled;

        if (!data.regions || data.regions.length === 0) {
            spawnRegionList.innerHTML = '<div style="color:var(--c-text-dim);font-size:10px">No regions defined</div>';
            return;
        }

        const totalLabel = `<div style="font-size:10px;color:var(--c-text-dim);margin-bottom:3px">
            Total: <span style="color:var(--c-accent)">${data.totalRate.toFixed(3)} veh/s</span>
        </div>`;
        const rows = data.regions.map(r =>
            `<div class="spawn-region-row">
                <span class="spawn-region-id" title="${r.id}">${r.id}</span>
                <span class="spawn-region-rate">${r.totalRate.toFixed(3)}/s</span>
            </div>`
        ).join('');
        spawnRegionList.innerHTML = totalLabel + rows;
    } catch {}
}

// Refresh spawn info: every 2s when spawn-rates debug layer is on, else every 5s
setInterval(() => refreshSpawnInfo(), 2000);
refreshSpawnInfo(); // initial fetch

// ── Inspector polling ─────────────────────────────────────────────────────────
setInterval(async () => {
    if (selectedId == null) { detailedInfo = null; return; }
    try {
        const d = await fetchVehicleDetail(selectedId);
        if (d) { detailedInfo = d; updateInspector(d); }
        else   { selectedId = null; detailedInfo = null; inspector.classList.remove('active'); }
    } catch {}
}, 100);

function updateInspector(info) {
    domId.textContent    = info.id;
    domState.textContent = info.state;
    domSpeed.textContent = info.speed.toFixed(1) + ' m/s';

    domAccel.textContent  = info.acceleration.toFixed(2) + ' m/s²';
    domAccel.className    = 'ins-val ' + (info.acceleration < -0.1 ? 'warn' : info.acceleration > 0.1 ? 'hi' : '');

    domLeader.textContent  = info.leaderId === -1 ? 'None' : info.leaderId;
    domGap.textContent     = (!info.gap || info.gap > 1000) ? '∞' : info.gap.toFixed(1) + ' m';
    domHeadway.textContent = (!info.headway || info.headway > 1000) ? '∞' : info.headway.toFixed(2) + ' s';

    domLane.textContent     = info.lane;
    domRoute.textContent    = info.routeIdx;
    domProgress.textContent = info.pathProgress != null ? info.pathProgress.toFixed(1) + ' m' : '--';
    domLaneLen.textContent  = info.laneLength   != null ? info.laneLength.toFixed(1)   + ' m' : '--';
    domInInt.textContent    = info.inIntersection != null ? (info.inIntersection ? 'Yes' : 'No') : '--';
    domInInt.className      = 'ins-val ' + (info.inIntersection ? 'hi' : '');
    domBlocked.textContent  = info.blockedReason || '—';
}

// ── Camera helpers ────────────────────────────────────────────────────────────
function lerpAngle(from, to, factor) {
    let delta = to - from;
    if (delta >  Math.PI) delta -= 2 * Math.PI;
    if (delta < -Math.PI) delta += 2 * Math.PI;
    return (from + delta * factor + 2 * Math.PI) % (2 * Math.PI);
}

setUserControlCallback(() => { if (followMode) followMode = false; });

// ── Click handling ────────────────────────────────────────────────────────────
dbgCanvas.addEventListener('click', (e) => {
    const rect = dbgCanvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const [wx, wy] = screenToWorld(sx, sy);

    // Road editor mode: click toggles nearest road closure
    if (roadEditMode && mapData) {
        const wayId = glRenderer.laneAtWorldPos(wx, wy, 8 / vp.scale);
        if (wayId != null) {
            const nowClosed = !closedWays.has(wayId);
            if (nowClosed) closedWays.add(wayId); else closedWays.delete(wayId);
            setRoadClosed(wayId, nowClosed);
            glRenderer.setClosedWays(new Set(closedWays));
            const n = closedWays.size;
            closureCountEl.textContent = n > 0
                ? `${n} road${n > 1 ? 's' : ''} closed`
                : '';
        }
        return;
    }

    // Vehicle selection — only in select mode
    if (interactionMode !== 'select') return;

    let best = null, bestD2 = 40 * 40;
    for (const v of vehicleRenderStates.values()) {
        const [px, py] = worldToScreen(v.x, v.y);
        const dx = px - sx, dy = py - sy;
        const d2 = dx * dx + dy * dy;
        if (d2 < bestD2) { bestD2 = d2; best = v; }
    }

    if (best) {
        selectedId = best.id;
        followMode = true;
        vp.autofit = false;
        vp.targetCx = best.x; vp.targetCy = best.y;
        fetchVehicleDetail(selectedId).then(d => {
            if (d) { detailedInfo = d; updateInspector(d); inspector.classList.add('active'); }
        }).catch(() => {});
    } else {
        selectedId = null; detailedInfo = null;
        inspector.classList.remove('active');
    }
});

// ── Keyboard shortcuts ────────────────────────────────────────────────────────
window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT') return;
    if (e.key.toLowerCase() === 'f') { followMode = false; vp.autofit = true; }
    if (e.key.toLowerCase() === 's') setInteractionMode('select');
    if (e.key.toLowerCase() === 'p') setInteractionMode('pan');
    if (e.key.toLowerCase() === 'e') simToggleExportMode();
    if (e.code === 'Space') { e.preventDefault(); togglePause(); }
    if (e.key === 'Escape') {
        if (simExportMode) simCancelExportMode();
        else if (roadEditMode) { roadEditMode = false; roadEditBtn.classList.remove('active'); }
        else if (selectedId != null) { selectedId = null; detailedInfo = null; inspector.classList.remove('active'); }
    }
});

// ── Speed / Pause controls ────────────────────────────────────────────────────
const pauseBtn   = document.getElementById('pauseBtn');
const speedBtns  = document.querySelectorAll('[data-speed]');
const speedSlider = document.getElementById('speedSlider');
const speedLabel  = document.getElementById('speedVal');

function togglePause() {
    isPaused = !isPaused;
    pauseBtn.classList.toggle('paused', isPaused);
    pauseBtn.textContent = isPaused ? '▶' : '⏸';
    if (isPaused) {
        const v = parseFloat(speedSlider.value); if (v > 0) lastSpeedBeforePause = v;
        setSimSpeed(0);
    } else {
        setSimSpeed(lastSpeedBeforePause);
    }
}

function updateSpeedUI(val) {
    speedSlider.value = val;
    speedLabel.textContent = val + '×';
    speedBtns.forEach(b => b.classList.toggle('active', parseFloat(b.dataset.speed) === val));
}

// Apply initial paused state visually (backend starts paused after project load)
pauseBtn.classList.toggle('paused', isPaused);
pauseBtn.textContent = isPaused ? '▶' : '⏸';

pauseBtn.addEventListener('click', togglePause);

speedBtns.forEach(btn => {
    btn.addEventListener('click', () => {
        const val = parseFloat(btn.dataset.speed);
        lastSpeedBeforePause = val;
        if (isPaused) togglePause();
        updateSpeedUI(val); setSimSpeed(val);
    });
});

speedSlider.addEventListener('input', () => {
    speedLabel.textContent = parseFloat(speedSlider.value).toFixed(1) + '×';
    speedBtns.forEach(b => b.classList.remove('active'));
});
speedSlider.addEventListener('change', () => {
    const val = parseFloat(speedSlider.value);
    lastSpeedBeforePause = val;
    if (isPaused) togglePause();
    setSimSpeed(val);
});

// ── Map loading ───────────────────────────────────────────────────────────────
function centerViewOnMap(data) {
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const lane of (data.lanes || [])) {
        for (const p of (lane.points || [])) {
            if (p.x < minX) minX = p.x; if (p.y < minY) minY = p.y;
            if (p.x > maxX) maxX = p.x; if (p.y > maxY) maxY = p.y;
        }
    }
    if (minX === Infinity) return;
    const dpr = window.devicePixelRatio || 1;
    const W = glCanvas.width / dpr, H = glCanvas.height / dpr;
    const s = Math.min(W * 0.85 / Math.max(maxX - minX, 1), H * 0.85 / Math.max(maxY - minY, 1));
    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    // Set both current and target so the view snaps immediately without animating from origin
    vp.cx = vp.targetCx = cx;
    vp.cy = vp.targetCy = cy;
    vp.scale = vp.targetScale = s;
}

fetchMap()
    .then(data => {
        mapData = data;
        glRenderer.loadMap(data);
        centerViewOnMap(data);
        if (data.mapOrigin) {
            console.log('[Scene] mapOrigin from map data:', data.mapOrigin);
            sceneView.setOrigin(data.mapOrigin.lat, data.mapOrigin.lon);
        } else {
            // Backend not yet recompiled — parse origin from raw OSM bounding box
            const proj = sessionStorage.getItem('currentProject');
            if (proj) {
                fetch(`/projects/${encodeURIComponent(proj)}/osm/raw`)
                    .then(r => r.text())
                    .then(xml => {
                        const doc = new DOMParser().parseFromString(xml, 'text/xml');
                        // Try <bounds> first
                        const b = doc.querySelector('bounds');
                        if (b) {
                            const refLat = (parseFloat(b.getAttribute('minlat')) + parseFloat(b.getAttribute('maxlat'))) / 2;
                            const refLon = (parseFloat(b.getAttribute('minlon')) + parseFloat(b.getAttribute('maxlon'))) / 2;
                            console.log('[Scene] mapOrigin from <bounds>:', refLat, refLon);
                            sceneView.setOrigin(refLat, refLon);
                        } else {
                            // Fall back to min/max of all node lat/lon
                            const nodes = doc.querySelectorAll('node[lat][lon]');
                            if (nodes.length > 0) {
                                let minLat = Infinity, maxLat = -Infinity;
                                let minLon = Infinity, maxLon = -Infinity;
                                for (const n of nodes) {
                                    const lat = parseFloat(n.getAttribute('lat'));
                                    const lon = parseFloat(n.getAttribute('lon'));
                                    if (lat < minLat) minLat = lat;
                                    if (lat > maxLat) maxLat = lat;
                                    if (lon < minLon) minLon = lon;
                                    if (lon > maxLon) maxLon = lon;
                                }
                                const refLat = (minLat + maxLat) / 2;
                                const refLon = (minLon + maxLon) / 2;
                                console.log('[Scene] mapOrigin from node bounds:', refLat, refLon, `(${nodes.length} nodes)`);
                                sceneView.setOrigin(refLat, refLon);
                            } else {
                                console.warn('[Scene] No <bounds> or nodes with lat/lon in OSM file');
                            }
                        }
                    })
                    .catch(e => console.warn('[Scene] OSM fallback failed:', e));
            } else {
                console.warn('[Scene] No currentProject in sessionStorage');
            }
        }
        // Build speed-limit lookup used by the congestion overlay
        laneSpeedLimitMap = new Map(
            (data.lanes || []).map(l => [l.id, l.speedLimitMps ?? 13.9])
        );
        hud.textContent = 'Sim Ready';
        console.log('Map loaded', data);
        // Fetch parking spots after map is loaded
        return fetchParkingSpots();
    })
    .then(data => {
        if (data && data.spots) {
            parkingSpots = data.spots;
            console.log(`Parking: ${data.total} spots, ${data.occupancy} occupied`);
        }
    })
    .catch(err => { console.error(err); hud.textContent = 'map load failed'; });

// Sync pause button state from backend (engine may start paused after project load)
fetch('/simulation/status')
    .then(r => r.ok ? r.json() : null)
    .then(data => {
        if (!data) return;
        const scale = data.timeScale ?? 0;
        const backendPaused = scale < 0.001;
        if (backendPaused !== isPaused) {
            // Correct frontend state without sending a WS message
            isPaused = backendPaused;
            pauseBtn.classList.toggle('paused', isPaused);
            pauseBtn.textContent = isPaused ? '▶' : '⏸';
            if (!isPaused && scale > 0) {
                lastSpeedBeforePause = scale;
                updateSpeedUI(scale);
            }
        }
    })
    .catch(() => {});

// ── WebSocket live feed ───────────────────────────────────────────────────────
connectLive((payload) => {
    currentSimTime = payload.simTime;
    const arr = payload.vehicles || [];

    const newTargets = new Map();
    for (const v of arr) newTargets.set(v.id, v);

    // Add new vehicles
    for (const [id, target] of newTargets.entries()) {
        if (!vehicleRenderStates.has(id)) vehicleRenderStates.set(id, { ...target });
    }

    // Remove gone vehicles
    for (const id of vehicleRenderStates.keys()) {
        if (!newTargets.has(id)) {
            vehicleRenderStates.delete(id);
            if (id === selectedId) { selectedId = null; inspector.classList.remove('active'); }
        }
    }

    vehicleTargets = newTargets;

    // Follow selected vehicle
    if (selectedId != null && followMode) {
        const sel = vehicleTargets.get(selectedId);
        if (sel) { vp.targetCx = sel.x; vp.targetCy = sel.y; }
    }

    // Autofit all vehicles on screen
    if (vp.autofit && arr.length > 0) {
        let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
        for (const v of arr) {
            if (v.x < minX) minX = v.x; if (v.y < minY) minY = v.y;
            if (v.x > maxX) maxX = v.x; if (v.y > maxY) maxY = v.y;
        }
        minX -= 10; minY -= 10; maxX += 10; maxY += 10;
        const dpr = window.devicePixelRatio || 1;
        const W = glCanvas.width / dpr, H = glCanvas.height / dpr;
        const sx = W * 0.9 / Math.max(maxX - minX, 50);
        const sy = H * 0.9 / Math.max(maxY - minY, 50);
        vp.targetScale = Math.max(vp.minScale, Math.min(vp.maxScale, Math.min(sx, sy)));
        vp.targetCx = (minX + maxX) * 0.5;
        vp.targetCy = (minY + maxY) * 0.5;
    }

    lastIntersections = payload.intersections || [];

    // Heatmap data pushed from server (only present in their respective modes)
    if (payload.density) {
        densityCells = payload.density; // full object: {minX,minY,maxX,maxY,resX,resY,cells}
    }
    if (payload.laneStats) {
        laneStatMap = new Map(payload.laneStats.map(s => [s.id, s]));
    }

    if (payload.metrics) updateMetrics(payload.metrics);
    if (payload.stepMs     != null) domMetStepMs.textContent   = payload.stepMs.toFixed(2) + ' ms';
    if (payload.maxTimeScale != null) {
        const ms = payload.maxTimeScale;
        domMetMaxScale.textContent = ms > 0 ? ms.toFixed(1) + '×' : '--';
    }
});

function updateMetrics(m) {
    domMetTotal.textContent = m.totalVehicles;

    const qPct = m.totalVehicles > 0 ? (m.vehiclesInQueue / m.totalVehicles) * 100 : 0;
    const qClass = qPct > 60 ? 'metric-val danger' : qPct > 30 ? 'metric-val warn' : 'metric-val ok';
    domMetQueue.textContent  = m.vehiclesInQueue;
    domMetQueue.className    = qClass;
    domMetQPct.textContent   = qPct.toFixed(0) + '%';
    domMetQPct.className     = qClass;
    domMetIntersect.textContent = m.vehiclesInIntersection ?? '--';
    domMetYielding.textContent  = m.yieldingVehicles ?? '--';

    const avgKmh = (m.avgSpeed * 3.6).toFixed(1);
    const maxKmh = (m.maxSpeed * 3.6).toFixed(1);
    const speedClass = parseFloat(avgKmh) < 5 ? 'metric-val danger'
                     : parseFloat(avgKmh) < 20 ? 'metric-val warn' : 'metric-val ok';
    domMetSpeed.textContent    = avgKmh + ' km/h';
    domMetSpeed.className      = speedClass;
    domMetMaxSpeed.textContent = maxKmh + ' km/h';
    domMetGap.textContent = (m.avgGapToLeader > 0 && m.avgGapToLeader < 1e6)
        ? m.avgGapToLeader.toFixed(1) + ' m' : '--';

    domMetSpawned.textContent   = m.totalSpawned ?? '--';
    domMetCompleted.textContent = m.totalCompleted ?? '--';
    const thr = m.throughputPerMinute ?? 0;
    domMetThroughput.textContent = thr > 0 ? thr.toFixed(1) + ' /min' : '-- /min';
    const travel = m.avgTravelTime ?? 0;
    domMetTravel.textContent = travel > 0
        ? (travel < 60 ? travel.toFixed(0) + ' s' : (travel / 60).toFixed(1) + ' min') : '--';

    metricsHistory.push({ speed: parseFloat(avgKmh), queuePct: qPct, throughput: thr });
    if (metricsHistory.length > MAX_HISTORY) metricsHistory.shift();
}

// ── Heatmap rendering helpers ─────────────────────────────────────────────────

// Maps t∈[0,1] → blue→cyan→green→yellow→red
function heatColor(t, alpha) {
    let r, g, b;
    if (t < 0.25)      { const s = t / 0.25;          r = 0;   g = Math.round(s*210);       b = 255; }
    else if (t < 0.5)  { const s = (t-0.25)/0.25;     r = Math.round(s*255); g = 210;        b = Math.round(255*(1-s)); }
    else if (t < 0.75) { const s = (t-0.5)/0.25;      r = 255; g = Math.round(210*(1-s));   b = 0; }
    else               {                                r = 255; g = 0;                        b = 0; }
    return `rgba(${r},${g},${b},${alpha})`;
}

// Sample the KDE density grid at an arbitrary world position, averaging a 3×3 neighbourhood.
function sampleDensityAt(density, wx, wy) {
    const { minX, minY, maxX, maxY, resX, resY, cells } = density;
    if (wx < minX || wx > maxX || wy < minY || wy > maxY) return 0;
    const cellW = (maxX - minX) / resX;
    const cellH = (maxY - minY) / resY;
    const ix = Math.floor((wx - minX) / cellW);
    const iy = Math.floor((wy - minY) / cellH);
    let sum = 0, n = 0;
    for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
            const gx = ix + dx, gy = iy + dy;
            if (gx >= 0 && gx < resX && gy >= 0 && gy < resY) {
                sum += cells[gy * resX + gx];
                n++;
            }
        }
    }
    return n > 0 ? sum / n : 0;
}

// Intersection density circles: one circle per intersection, sized by local agent count.
// The label shows agents-per-minute, estimated from the KDE snapshot count assuming
// an average intersection traversal time of ~5 seconds at city speed.
const INTER_TRAVERSAL_S = 5;  // seconds to cross an intersection (heuristic for per-min estimate)
const INTER_SATURATION  = 6;  // KDE count that maps to maximum circle size
const INTER_MIN_R       = 8;  // minimum circle radius in screen-pixels
const INTER_MAX_R       = 38; // maximum circle radius in screen-pixels

function drawDensityOverlay(ctx, worldToScreen, density, mapData) {
    if (!density?.cells || !mapData?.intersections) return;
    const { cells, resX, resY } = density;
    if (!cells || cells.length !== resX * resY) return;

    ctx.save();
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';

    for (const inter of mapData.intersections) {
        const count = sampleDensityAt(density, inter.x, inter.y);
        if (count < 0.2) continue;

        const t      = Math.min(count / INTER_SATURATION, 1);
        const r      = INTER_MIN_R + t * (INTER_MAX_R - INTER_MIN_R);
        const perMin = Math.round(count * 60 / INTER_TRAVERSAL_S);
        const [sx, sy] = worldToScreen(inter.x, inter.y);

        // Soft glow ring
        ctx.beginPath();
        ctx.arc(sx, sy, r + 4, 0, Math.PI * 2);
        ctx.fillStyle = heatColor(t, 0.15);
        ctx.fill();

        // Main filled circle
        ctx.beginPath();
        ctx.arc(sx, sy, r, 0, Math.PI * 2);
        ctx.fillStyle = heatColor(t, 0.80);
        ctx.fill();
        ctx.strokeStyle = 'rgba(255,255,255,0.55)';
        ctx.lineWidth   = 1.5;
        ctx.stroke();

        // Agent-per-minute label
        if (perMin > 0) {
            const fontSize = Math.max(9, Math.min(14, Math.round(r * 0.65)));
            ctx.font      = `bold ${fontSize}px system-ui`;
            ctx.fillStyle = 'rgba(255,255,255,0.95)';
            ctx.fillText(perMin, sx, sy);
        }
    }
    ctx.restore();
}

/**
 * Map a free-flow ratio [0, 1] to a green→yellow→red colour.
 *   ratio = 1  → pure green  (hue 120)
 *   ratio = 0  → pure red    (hue 0)
 * Uses HSL so the mid-point lands on yellow (hue 60).
 */
function congestionColor(ratio, alpha) {
    const hue = Math.round(ratio * 120);
    return `hsla(${hue},100%,50%,${alpha})`;
}

function drawCongestionOverlay(ctx, worldToScreen, mapData, statMap, vp) {
    if (!mapData?.lanes) return;
    const laneW = Math.max(2, vp.scale * 4.0);
    ctx.lineWidth = laneW;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    for (const lane of mapData.lanes) {
        const stat = statMap.get(lane.id);
        if (!stat || stat.count === 0) {
            // No agents on this lane — draw faint so the road network is still visible
            ctx.strokeStyle = 'rgba(80,110,130,0.18)';
        } else {
            const speedLimit = laneSpeedLimitMap.get(lane.id) ?? 13.9; // default ~50 km/h
            // ratio: 1 = travelling at speed limit (free flow), 0 = standstill
            const ratio = Math.min(1, stat.speed / speedLimit);
            ctx.strokeStyle = congestionColor(ratio, 0.82);
        }
        ctx.beginPath();
        let first = true;
        for (const pt of lane.points) {
            const [sx, sy] = worldToScreen(pt.x, pt.y);
            if (first) { ctx.moveTo(sx, sy); first = false; }
            else ctx.lineTo(sx, sy);
        }
        ctx.stroke();
    }
}

// Draw a compact legend in the bottom-right of the debug canvas.
function drawHeatmapLegend(ctx, mode, W, H) {
    if (mode === 'density') {
        const px = W - 118, py = H - 130;
        ctx.fillStyle = 'rgba(6,10,20,0.82)';
        ctx.fillRect(px - 8, py - 20, 110, 72);

        ctx.font = 'bold 9px system-ui';
        ctx.fillStyle = '#3e5a6c';
        ctx.textAlign = 'left';
        ctx.fillText('DENSITY', px, py - 7);

        const stops = [0, 0.33, 0.67, 1.0];
        stops.forEach((t, i) => {
            const cx = px + 10 + i * 23;
            const r  = INTER_MIN_R * 0.45 + t * (INTER_MAX_R * 0.45 - INTER_MIN_R * 0.45);
            ctx.beginPath();
            ctx.arc(cx, py + 12, Math.max(4, r), 0, Math.PI * 2);
            ctx.fillStyle = heatColor(t, 0.80);
            ctx.fill();
            ctx.strokeStyle = 'rgba(255,255,255,0.4)';
            ctx.lineWidth = 1;
            ctx.stroke();
        });
        ctx.font = '9px system-ui';
        ctx.fillStyle = '#6a8898';
        ctx.textAlign = 'left';  ctx.fillText('Low',  px,      py + 30);
        ctx.textAlign = 'right'; ctx.fillText('High', px + 90, py + 30);
        ctx.textAlign = 'left';
        ctx.fillStyle = '#4a7080';
        ctx.fillText('agents / min', px, py + 42);
    } else {
        // Congestion legend — gradient bar from red (0%) to green (100%)
        const barW = 96, barH = 10;
        const px = W - 118, py = H - 110;
        ctx.fillStyle = 'rgba(6,10,20,0.82)';
        ctx.fillRect(px - 8, py - 22, barW + 20, 74);

        ctx.font = 'bold 9px system-ui';
        ctx.fillStyle = '#3e5a6c';
        ctx.textAlign = 'left';
        ctx.fillText('CONGESTION', px, py - 9);

        // Gradient bar: left = red (0%), right = green (100%)
        const grad = ctx.createLinearGradient(px, 0, px + barW, 0);
        grad.addColorStop(0,    'hsl(0,100%,50%)');
        grad.addColorStop(0.5,  'hsl(60,100%,50%)');
        grad.addColorStop(1.0,  'hsl(120,100%,50%)');
        ctx.fillStyle = grad;
        ctx.fillRect(px, py, barW, barH);

        // Tick labels
        ctx.font = '9px system-ui';
        ctx.fillStyle = '#8ab0c0';
        ctx.textAlign = 'center';
        [[0,'0%'],[0.5,'50%'],[1,'100%']].forEach(([t, lbl]) => {
            ctx.fillText(lbl, px + t * barW, py + barH + 11);
        });

        // Sub-label
        ctx.font = '9px system-ui';
        ctx.fillStyle = '#4a7080';
        ctx.textAlign = 'left';
        ctx.fillText('% of speed limit', px, py + barH + 24);

        // No-traffic swatch
        ctx.fillStyle = 'rgba(80,110,130,0.55)';
        ctx.fillRect(px, py + barH + 34, 11, 8);
        ctx.fillStyle = '#6a8898';
        ctx.textAlign = 'left';
        ctx.fillText('No traffic', px + 15, py + barH + 42);
    }
}

// ── Render loop ───────────────────────────────────────────────────────────────
function frame() {
    perfBegin();

    // Advance viewport interpolation
    vpStep();
    perfMark('Viewport');

    // Throttle viewport subscription updates to ~4/s
    const now = performance.now();
    if (now - lastViewportSendTime > 250) {
        const dpr = window.devicePixelRatio || 1;
        const W = glCanvas.width / dpr, H = glCanvas.height / dpr;
        const halfW = W / (2 * vp.scale), halfH = H / (2 * vp.scale);
        const PAD = 100;
        sendViewport(vp.cx - halfW - PAD, vp.cy - halfH - PAD,
                     vp.cx + halfW + PAD, vp.cy + halfH + PAD);
        lastViewportSendTime = now;
    }

    // Smooth vehicle positions
    const SMOOTH = 0.2;
    const rendered = [];
    for (const [id, rs] of vehicleRenderStates.entries()) {
        const ts = vehicleTargets.get(id);
        if (ts) {
            rs.x  += (ts.x - rs.x) * SMOOTH;
            rs.y  += (ts.y - rs.y) * SMOOTH;
            rs.h   = lerpAngle(rs.h ?? ts.h, ts.h, SMOOTH);
            rs.speed = ts.speed;
            rs.a   = ts.acceleration;
        }
        rendered.push(rs);
    }
    perfMark('Smooth');

    // Scene background (tiles) — rendered before WebGL so it shows through the transparent clear
    if (sceneMode) sceneView.render();

    // WebGL render (roads + vehicles)
    const viewMatrix = getViewMatrix();
    glRenderer.render(viewMatrix, rendered, selectedId, sceneMode);
    perfMark('GL Render');

    // 2D debug overlay
    const dpr = window.devicePixelRatio || 1;
    const overlayW = dbgCanvas.width / dpr;
    const overlayH = dbgCanvas.height / dpr;
    dbgCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    dbgCtx.clearRect(0, 0, overlayW, overlayH);

    // Heatmap overlays (drawn before debug layers so debug info stays on top)
    if (viewMode === 'density' && densityCells && densityCells.cells) {
        drawDensityOverlay(dbgCtx, worldToScreen, densityCells, mapData);
    }
    if (viewMode === 'congestion' && mapData) {
        drawCongestionOverlay(dbgCtx, worldToScreen, mapData, laneStatMap, vp);
    }
    if (viewMode !== 'agents') {
        drawHeatmapLegend(dbgCtx, viewMode, overlayW, overlayH);
    }

    // Draw parking spots (always visible in agents mode when zoomed in)
    if (viewMode === 'agents' && parkingSpots.length > 0) {
        drawParkingSpots(dbgCtx, worldToScreen, parkingSpots, vp);
    }

    if (mapData && debugOptions.master) {
        drawDebugLayers(dbgCtx, worldToScreen, mapData, vp, lastIntersections, debugOptions, lastSpawnData, currentSimTime);
        if (debugOptions.ids) drawVehicleIds(dbgCtx, worldToScreen, rendered, vp);
    }

    if (selectedId != null) {
        const sel    = vehicleRenderStates.get(selectedId);
        const leader = detailedInfo?.leaderId !== -1
            ? vehicleRenderStates.get(detailedInfo?.leaderId)
            : null;
        drawSelectedDebug(dbgCtx, worldToScreen, vp, sel, leader, detailedInfo, debugOptions);
    }
    renderSimExportViewport(dbgCtx, overlayW, overlayH);
    perfMark('Debug Overlay');

    // Metrics graph
    drawMetricsGraph();

    // HUD
    hud.textContent =
        `Time: ${currentSimTime.toFixed(1)} s  |  Cars: ${rendered.length}  |  Scale: ${vp.scale.toFixed(1)}`;

    perfEnd();
    requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

// ── Metrics graph ─────────────────────────────────────────────────────────────
function drawMetricsGraph() {
    if (metricsHistory.length < 2) return;
    const w = graphCanvas.width, h = graphCanvas.height;
    graphCtx.fillStyle = '#040810'; graphCtx.fillRect(0, 0, w, h);

    // Grid
    graphCtx.strokeStyle = '#0e1828'; graphCtx.lineWidth = 1;
    for (const f of [0.25, 0.5, 0.75]) {
        const y = Math.round(h * f) + 0.5;
        graphCtx.beginPath(); graphCtx.moveTo(0, y); graphCtx.lineTo(w, y); graphCtx.stroke();
    }

    const stepX = w / (MAX_HISTORY - 1);
    let maxSpeed = 50, maxThru = 10;
    for (const d of metricsHistory) {
        if (d.speed      > maxSpeed) maxSpeed = d.speed;
        if (d.throughput > maxThru)  maxThru  = d.throughput;
    }

    const drawLine = (color, getValue) => {
        graphCtx.beginPath(); graphCtx.strokeStyle = color; graphCtx.lineWidth = 1.5;
        graphCtx.lineJoin = 'round';
        for (let i = 0; i < metricsHistory.length; i++) {
            const x = i * stepX;
            const y = h - getValue(metricsHistory[i]) * (h - 2) - 1;
            i === 0 ? graphCtx.moveTo(x, y) : graphCtx.lineTo(x, y);
        }
        graphCtx.stroke();
    };

    drawLine('rgba(102,136,255,0.85)', d => maxThru  > 0 ? Math.min(d.throughput / maxThru,  1) : 0);
    drawLine('rgba(255,170,34,0.90)',  d =>               Math.min(d.queuePct    / 100,       1));
    drawLine('#00e87a',                d => maxSpeed > 0 ? Math.min(d.speed      / maxSpeed,  1) : 0);
}

// ── Pipeline PNG export ───────────────────────────────────────────────────────

const SIM_TOTAL_W  = 2480;   // A4 portrait @ 300 dpi
const SIM_H_PAD    = 80;
const SIM_V_PAD    = 36;
const SIM_HDR_H    = 100;
const SIM_LEG_H    = 56;
const SIM_STEP_GAP = 30;
const SIM_FONT     = 'Arial, Helvetica, sans-serif';

const SIM_BG        = '#ffffff';
const SIM_TITLE_COL = '#111111';
const SIM_SUB_COL   = '#555555';

const SIM_ROAD_COLS = [
    '#c41832', '#c86400', '#0077b3', '#117a38',
    '#6b1ec0', '#444444', '#888888', '#aaaaaa',
];
const SIM_ROAD_LABELS = [
    'Motorway', 'Trunk', 'Primary', 'Secondary',
    'Tertiary', 'Residential', 'Service', 'Other',
];
const SIM_ROAD_LW = [3.0, 2.6, 2.2, 1.8, 1.4, 1.0, 0.8, 0.6];

const SIM_LANE_DIM    = '#c8c4bc';
const SIM_BUILDING    = '#ede9e4';
const SIM_BLDG_STROKE = '#cdc9c3';
const SIM_BEZIER_COL  = '#e06010';
const SIM_TL_COL      = '#cc1111';
const SIM_INT_COL     = '#0066aa';
const SIM_SPAWN_COL   = '#0077b3';
const SIM_PARK_COL    = '#117a38';
const SIM_AGENT_COL   = '#c41832';
const SIM_PARKED_COL  = '#666666';

const SIM_BADGE_COLS = [
    '#c41832', '#c86400', '#0077b3', '#117a38',
    '#6b1ec0', '#c86400', '#117a38', '#444444', '#c41832',
];

const SIM_STEPS = [
    { title: 'Raw OSM Graph',                   sub: 'Nodes and way segments as loaded from OpenStreetMap' },
    { title: 'Road Classification & Filtering', sub: 'Non-road elements removed; ways classified by road type' },
    { title: 'Lane Reconstruction',             sub: 'Each way split into individual directional lane centerlines' },
    { title: 'Curb Trimming',                   sub: 'Lane endpoints retracted to intersection entry boundaries' },
    { title: 'Bézier Connection Paths',         sub: 'Trimmed lane ends joined through intersections via cubic Bézier curves' },
    { title: 'Traffic Signal Placement',        sub: 'Intersections classified; signal heads assigned from OSM node data' },
    { title: 'Parking & Spawn Zones',           sub: 'Parking spots extracted; vehicle spawn regions defined on entry lanes' },
    { title: 'Building Footprints',             sub: 'OSM building polygons added for urban spatial context' },
    { title: 'Live Simulation',                 sub: 'Agent-based vehicles navigating the fully constructed network' },
];

// ── Export viewport overlay helpers ──────────────────────────────────────────

function simVpScreenRect() {
    if (!simExportViewport) return null;
    const { minX, maxX, minY, maxY } = simExportViewport;
    const [l, t] = worldToScreen(minX, maxY);
    const [r, b] = worldToScreen(maxX, minY);
    return { l, t, r, b };
}

function simVpHitTest(clientX, clientY) {
    if (!simExportViewport) return null;
    const rect = dbgCanvas.getBoundingClientRect();
    const sx = clientX - rect.left, sy = clientY - rect.top;
    const sr = simVpScreenRect();
    const HIT = 10;
    if (Math.hypot(sx - sr.l, sy - sr.t) < HIT) return 'tl';
    if (Math.hypot(sx - sr.r, sy - sr.t) < HIT) return 'tr';
    if (Math.hypot(sx - sr.l, sy - sr.b) < HIT) return 'bl';
    if (Math.hypot(sx - sr.r, sy - sr.b) < HIT) return 'br';
    if (sx >= sr.l && sx <= sr.r && sy >= sr.t && sy <= sr.b) return 'inside';
    return null;
}

function renderSimExportViewport(ctx, W, H) {
    if (!simExportMode) return;
    const sr = simVpScreenRect();
    ctx.save();
    if (sr && (sr.r - sr.l) > 4 && (sr.b - sr.t) > 4) {
        ctx.fillStyle = 'rgba(0,0,0,0.45)';
        ctx.beginPath();
        ctx.rect(0, 0, W, H);
        ctx.rect(sr.l, sr.t, sr.r - sr.l, sr.b - sr.t);
        ctx.fill('evenodd');
        ctx.strokeStyle = '#f5a623';
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.strokeRect(sr.l, sr.t, sr.r - sr.l, sr.b - sr.t);
        ctx.setLineDash([]);
        ctx.fillStyle = '#f5a623';
        const SZ = 8;
        for (const [cx, cy] of [[sr.l,sr.t],[sr.r,sr.t],[sr.l,sr.b],[sr.r,sr.b]])
            ctx.fillRect(cx - SZ/2, cy - SZ/2, SZ, SZ);
        const hint = 'Click ⎙ to export pipeline PNG';
        ctx.font = '13px system-ui';
        const tw = ctx.measureText(hint).width;
        const hx = Math.max(4, Math.min(sr.l, W - tw - 18));
        const hy = Math.min(sr.b + 6, H - 28);
        ctx.fillStyle = 'rgba(0,0,0,0.72)';
        ctx.fillRect(hx - 4, hy, tw + 12, 22);
        ctx.fillStyle = '#ffffff';
        ctx.textAlign = 'left'; ctx.textBaseline = 'top';
        ctx.fillText(hint, hx + 2, hy + 5);
    } else {
        ctx.fillStyle = 'rgba(0,0,0,0.28)';
        ctx.fillRect(0, 0, W, H);
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 15px system-ui';
        ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText('Drag to select export area', W / 2, H / 2);
    }
    ctx.restore();
}

function simToggleExportMode() {
    if (!mapData) return;
    simExportMode = !simExportMode;
    document.getElementById('simExportBtn').classList.toggle('active', simExportMode);
    document.getElementById('simExportGoBtn').style.display = simExportMode ? '' : 'none';
    if (simExportMode) {
        dbgCanvas.style.cursor = 'crosshair';
        if (!simExportViewport) {
            let mnX = Infinity, mxX = -Infinity, mnY = Infinity, mxY = -Infinity;
            for (const lane of (mapData.lanes || [])) {
                for (const p of (lane.points || [])) {
                    if (p.x < mnX) mnX = p.x; if (p.x > mxX) mxX = p.x;
                    if (p.y < mnY) mnY = p.y; if (p.y > mxY) mxY = p.y;
                }
            }
            if (isFinite(mnX)) {
                const px = (mxX - mnX) * 0.04, py = (mxY - mnY) * 0.04;
                simExportViewport = { minX: mnX-px, maxX: mxX+px, minY: mnY-py, maxY: mxY+py };
            }
        }
    } else {
        dbgCanvas.style.cursor = interactionMode === 'pan' ? 'grab' : 'default';
    }
}

function simCancelExportMode() {
    simExportMode = false;
    document.getElementById('simExportBtn')?.classList.remove('active');
    document.getElementById('simExportGoBtn').style.display = 'none';
    dbgCanvas.style.cursor = interactionMode === 'pan' ? 'grab' : 'default';
}

// Intercept at window capture level — fires before viewport.js's element-level listener,
// so stopPropagation truly prevents the map from panning during export interaction.
window.addEventListener('mousedown', (e) => {
    if (!simExportMode || e.target !== dbgCanvas) return;
    e.stopPropagation();

    const rect = dbgCanvas.getBoundingClientRect();
    const [wx, wy] = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);

    if (simExportViewport) {
        const hit = simVpHitTest(e.clientX, e.clientY);
        if (hit === 'tl' || hit === 'tr' || hit === 'bl' || hit === 'br') {
            _simVpDrag = { mode: 'resize', corner: hit, sx: e.clientX, sy: e.clientY, vp0: { ...simExportViewport } };
            return;
        }
        if (hit === 'inside') {
            _simVpDrag = { mode: 'move', sx: e.clientX, sy: e.clientY, vp0: { ...simExportViewport } };
            return;
        }
    }
    // Click outside (or no existing viewport) → draw a new rectangle
    _simVpDrag = { mode: 'draw', wx0: wx, wy0: wy };
    simExportViewport = { minX: wx, maxX: wx, minY: wy, maxY: wy };
}, { capture: true });

window.addEventListener('mouseup', () => { _simVpDrag = null; });

window.addEventListener('mousemove', (e) => {
    if (!simExportMode) return;
    if (!_simVpDrag) {
        const hit = simVpHitTest(e.clientX, e.clientY);
        if      (hit === 'tl' || hit === 'br') dbgCanvas.style.cursor = 'nwse-resize';
        else if (hit === 'tr' || hit === 'bl') dbgCanvas.style.cursor = 'nesw-resize';
        else if (hit === 'inside')             dbgCanvas.style.cursor = 'move';
        else                                   dbgCanvas.style.cursor = 'crosshair';
        return;
    }
    const rect = dbgCanvas.getBoundingClientRect();
    const [wx, wy] = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
    const dxW = (e.clientX - _simVpDrag.sx) / vp.scale;
    const dyW = (e.clientY - _simVpDrag.sy) / vp.scale;
    const d = _simVpDrag;
    if (d.mode === 'draw') {
        simExportViewport = {
            minX: Math.min(d.wx0, wx), maxX: Math.max(d.wx0, wx),
            minY: Math.min(d.wy0, wy), maxY: Math.max(d.wy0, wy),
        };
    } else if (d.mode === 'move') {
        simExportViewport = {
            minX: d.vp0.minX + dxW, maxX: d.vp0.maxX + dxW,
            minY: d.vp0.minY - dyW, maxY: d.vp0.maxY - dyW,
        };
    } else {
        const v = d.vp0, c = d.corner;
        simExportViewport = {
            minX: (c === 'tl' || c === 'bl') ? v.minX + dxW : v.minX,
            maxX: (c === 'tr' || c === 'br') ? v.maxX + dxW : v.maxX,
            minY: (c === 'bl' || c === 'br') ? v.minY - dyW : v.minY,
            maxY: (c === 'tl' || c === 'tr') ? v.maxY - dyW : v.maxY,
        };
    }
});

// ── Export canvas coordinate transform ───────────────────────────────────────

function simW2S(wx, wy, pc) {
    return [
        pc.ox + pc.hPad + (wx - pc.minX) * pc.scale,
        pc.oy + pc.hdrH + pc.vPad + pc.mapH - (wy - pc.minY) * pc.scale,
    ];
}

// ── Data helpers ──────────────────────────────────────────────────────────────

function simBuildLanes(data) {
    const typeOf = {};
    for (const w of (data.ways || [])) typeOf[w.id] = w.type ?? 5;
    return (data.lanes || []).map(l => {
        const pts = (l.points || []).map(p => [p.x, p.y]);
        if (pts.length < 2) return null;
        const wayId   = Math.floor(l.id / 1000);
        const laneIdx = l.id % 1000 >= 200 ? (l.id % 1000) - 200 : (l.id % 1000) - 100;
        return { wayId, laneIdx, points: pts, type: typeOf[wayId] ?? 5 };
    }).filter(Boolean);
}

// ── Drawing primitives ────────────────────────────────────────────────────────

function simDrawLanes(ctx, pc, lanes, colorFn, lwFn) {
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    for (const { type, points } of lanes) {
        ctx.strokeStyle = colorFn(type);
        ctx.lineWidth   = lwFn(type);
        ctx.beginPath();
        let first = true;
        for (const [wx, wy] of points) {
            const [sx, sy] = simW2S(wx, wy, pc);
            first ? (ctx.moveTo(sx, sy), first = false) : ctx.lineTo(sx, sy);
        }
        ctx.stroke();
    }
}

function simDrawBuildings(ctx, pc, buildings) {
    ctx.fillStyle   = SIM_BUILDING;
    ctx.strokeStyle = SIM_BLDG_STROKE;
    ctx.lineWidth   = Math.max(0.3, 0.4 * pc.ls);
    for (const b of buildings) {
        for (const ring of (b.outer || [])) {
            if (ring.length < 3) continue;
            ctx.beginPath();
            for (let i = 0; i < ring.length; i++) {
                const [sx, sy] = simW2S(ring[i].x, ring[i].y, pc);
                i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy);
            }
            ctx.closePath();
            ctx.fill(); ctx.stroke();
        }
    }
}

function simDrawIntPaths(ctx, pc, paths, color, lw) {
    ctx.strokeStyle = color; ctx.lineWidth = lw;
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    for (const p of paths) {
        const pts = p.points || [];
        if (pts.length < 2) continue;
        ctx.beginPath();
        const [sx0, sy0] = simW2S(pts[0].x, pts[0].y, pc);
        ctx.moveTo(sx0, sy0);
        for (let i = 1; i < pts.length; i++) {
            const [sx, sy] = simW2S(pts[i].x, pts[i].y, pc);
            ctx.lineTo(sx, sy);
        }
        ctx.stroke();
    }
}

function simDrawTrafficLights(ctx, pc, intersections) {
    const icoH = Math.max(6, Math.min(60, pc.scale * 9));
    const icoW = icoH * 0.56;
    const dotR = icoH * 0.18;
    for (const inter of intersections) {
        const [sx, sy] = simW2S(inter.x, inter.y, pc);
        if (inter.type === 1) {
            ctx.fillStyle = '#111111';
            ctx.fillRect(sx - icoW/2, sy - icoH/2, icoW, icoH);
            const offs   = [-icoH * 0.3, 0, icoH * 0.3];
            const lights = ['#ee2222', '#eeaa00', '#22bb22'];
            for (let i = 0; i < 3; i++) {
                ctx.fillStyle = lights[i];
                ctx.beginPath();
                ctx.arc(sx, sy + offs[i], dotR, 0, Math.PI * 2);
                ctx.fill();
            }
        } else {
            ctx.fillStyle = SIM_INT_COL;
            ctx.globalAlpha = 0.4;
            ctx.beginPath();
            ctx.arc(sx, sy, Math.max(2.5, icoH * 0.2), 0, Math.PI * 2);
            ctx.fill();
            ctx.globalAlpha = 1;
        }
    }
}

function simDrawSpawn(ctx, pc, regions) {
    ctx.setLineDash([6, 4]);
    for (const r of regions) {
        const [sx1, sy1] = simW2S(r.min.x, r.max.y, pc);
        const [sx2, sy2] = simW2S(r.max.x, r.min.y, pc);
        const rw = sx2 - sx1, rh = sy2 - sy1;
        ctx.fillStyle = SIM_SPAWN_COL + '22';
        ctx.fillRect(sx1, sy1, rw, rh);
        ctx.strokeStyle = SIM_SPAWN_COL;
        ctx.lineWidth = Math.max(0.5, 2 * pc.ls);
        ctx.strokeRect(sx1, sy1, rw, rh);
    }
    ctx.setLineDash([]);
}

function simDrawParking(ctx, pc, spots) {
    ctx.fillStyle = SIM_PARK_COL;
    for (const sp of spots) {
        const [sx, sy] = simW2S(sp.x, sp.y, pc);
        ctx.beginPath();
        ctx.arc(sx, sy, Math.max(1.5, Math.min(5, 2.5 * pc.ls)), 0, Math.PI * 2);
        ctx.fill();
    }
}

function simDrawAgents(ctx, pc, vehicles) {
    const carL = Math.max(5, pc.scale * 4.5);
    const carW = Math.max(3, pc.scale * 2.0);
    for (const v of vehicles) {
        const [sx, sy] = simW2S(v.x, v.y, pc);
        ctx.save();
        ctx.translate(sx, sy);
        ctx.rotate(-v.h);
        ctx.fillStyle = v.parked ? SIM_PARKED_COL : SIM_AGENT_COL;
        ctx.fillRect(-carL/2, -carW/2, carL, carW);
        ctx.restore();
    }
}

// ── Panel header ──────────────────────────────────────────────────────────────

function simDrawStepHeader(ctx, stepIdx, pc) {
    const { title, sub } = SIM_STEPS[stepIdx];
    const col = SIM_BADGE_COLS[stepIdx];
    const bx  = pc.ox + pc.hPad, by = pc.oy + 22;
    // Badge circle
    ctx.fillStyle = col;
    ctx.fillRect(bx, by, 54, 54);
    ctx.fillStyle = '#ffffff';
    ctx.font = `bold 30px ${SIM_FONT}`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(String(stepIdx + 1), bx + 27, by + 28);
    // Title + subtitle
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillStyle = SIM_TITLE_COL;
    ctx.font = `bold 38px ${SIM_FONT}`;
    ctx.fillText(title, bx + 70, by + 2);
    ctx.fillStyle = SIM_SUB_COL;
    ctx.font = `21px ${SIM_FONT}`;
    ctx.fillText(sub, bx + 70, by + 44);
    // Accent bar
    ctx.fillStyle = col;
    ctx.fillRect(pc.ox + pc.hPad, pc.oy + pc.hdrH - 4, SIM_TOTAL_W - 2 * pc.hPad, 3);
}

// ── Legend helpers ────────────────────────────────────────────────────────────

function simLegY(pc) {
    return pc.oy + pc.hdrH + pc.vPad + pc.mapH + pc.vPad + 8;
}

function simLegItem(ctx, x, y, color, label, shape) {
    ctx.save();
    ctx.lineCap = 'round';
    if (shape === 'line') {
        ctx.strokeStyle = color; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.moveTo(x, y+9); ctx.lineTo(x+28, y+9); ctx.stroke();
    } else if (shape === 'dashline') {
        ctx.strokeStyle = color; ctx.lineWidth = 2;
        ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(x, y+9); ctx.lineTo(x+28, y+9); ctx.stroke();
        ctx.setLineDash([]);
    } else if (shape === 'dot') {
        ctx.fillStyle = color;
        ctx.beginPath(); ctx.arc(x+14, y+9, 6, 0, Math.PI*2); ctx.fill();
    } else if (shape === 'rect') {
        ctx.fillStyle = color + '33';
        ctx.strokeStyle = color; ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 3]);
        ctx.fillRect(x+2, y+3, 24, 12); ctx.strokeRect(x+2, y+3, 24, 12);
        ctx.setLineDash([]);
    } else if (shape === 'box') {
        ctx.fillStyle = color;
        ctx.fillRect(x+2, y+3, 24, 12);
    }
    ctx.fillStyle = SIM_SUB_COL;
    ctx.font = `17px ${SIM_FONT}`;
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(label, x + 36, y + 1);
    ctx.restore();
}

// ── Main export function ──────────────────────────────────────────────────────

function simDoExport() {
    if (!mapData) { alert('No map loaded.'); return; }
    const ev = simExportViewport;
    if (!ev) { alert('Draw an export area first.'); return; }
    const viewW = ev.maxX - ev.minX, viewH = ev.maxY - ev.minY;
    if (viewW < 1 || viewH < 1) { alert('Export area too small.'); return; }

    const lanes     = simBuildLanes(mapData);
    const buildings = mapData.buildings || [];
    const rawNodes  = mapData.rawOsm?.nodes   || {};
    const rawSegs   = mapData.rawOsm?.segments || [];
    const intPaths  = mapData.intersection_paths || [];
    const inters    = mapData.intersections  || [];
    const spawnRegs = mapData.spawn_regions  || [];
    // Build edge set from retained ways' node sequences so rawOsm segments can be filtered.
    // rawOsm segments use original OSM way IDs (unrelated to ways[].id counter), so we
    // match on node-pair edges instead.
    const retEdges = new Set();
    for (const way of (mapData.ways || [])) {
        const nodes = way.nodes || [];
        for (let i = 0; i < nodes.length - 1; i++) {
            retEdges.add(`${nodes[i]}-${nodes[i+1]}`);
            retEdges.add(`${nodes[i+1]}-${nodes[i]}`);
        }
    }

    const mapPxW  = SIM_TOTAL_W - 2 * SIM_H_PAD;
    const scale   = mapPxW / viewW;
    const mapH    = Math.round(viewH * scale);
    const stepH   = SIM_HDR_H + SIM_V_PAD + mapH + SIM_V_PAD + SIM_LEG_H;
    const N       = SIM_STEPS.length;
    const OUTER   = 40;
    const TOTAL_H = OUTER + N * stepH + (N - 1) * SIM_STEP_GAP + OUTER;

    const cvs = document.createElement('canvas');
    cvs.width = SIM_TOTAL_W; cvs.height = TOTAL_H;
    const ctx = cvs.getContext('2d');
    ctx.fillStyle = SIM_BG;
    ctx.fillRect(0, 0, SIM_TOTAL_W, TOTAL_H);

    const mkPc = (i) => ({
        ox: 0, oy: OUTER + i * (stepH + SIM_STEP_GAP),
        hdrH: SIM_HDR_H, hPad: SIM_H_PAD, vPad: SIM_V_PAD,
        mapH, scale, minX: ev.minX, minY: ev.minY,
        ls: Math.sqrt(scale / 2.0),
    });

    const clip = (pc, fn) => {
        ctx.save();
        ctx.beginPath();
        ctx.rect(pc.ox + pc.hPad, pc.oy + pc.hdrH + pc.vPad, mapPxW, pc.mapH);
        ctx.clip();
        fn();
        ctx.restore();
    };

    const MARGIN = Math.max(viewW, viewH) * 0.05;
    const inView = (x, y) =>
        x >= ev.minX - MARGIN && x <= ev.maxX + MARGIN &&
        y >= ev.minY - MARGIN && y <= ev.maxY + MARGIN;

    // ── 1: Raw OSM ────────────────────────────────────────────────────────────
    {
        const pc = mkPc(0);
        simDrawStepHeader(ctx, 0, pc);
        clip(pc, () => {
            ctx.lineCap = 'round';
            for (const seg of rawSegs) {
                const nA = rawNodes[seg.from], nB = rawNodes[seg.to];
                if (!nA || !nB || (!inView(nA.x, nA.y) && !inView(nB.x, nB.y))) continue;
                const t = Math.min(seg.roadType ?? 5, 7);
                ctx.strokeStyle = SIM_ROAD_COLS[t]; ctx.lineWidth = SIM_ROAD_LW[t] * pc.ls;
                const [sx1,sy1] = simW2S(nA.x, nA.y, pc);
                const [sx2,sy2] = simW2S(nB.x, nB.y, pc);
                ctx.beginPath(); ctx.moveTo(sx1,sy1); ctx.lineTo(sx2,sy2); ctx.stroke();
            }
            for (const node of Object.values(rawNodes)) {
                if (!inView(node.x, node.y)) continue;
                const [sx, sy] = simW2S(node.x, node.y, pc);
                ctx.fillStyle = node.type === 1 ? SIM_TL_COL : '#333333';
                ctx.beginPath();
                ctx.arc(sx, sy, node.type === 1 ? 3.5 * pc.ls : 2 * pc.ls, 0, Math.PI * 2);
                ctx.fill();
            }
        });
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        for (let i = 0; i <= 4; i++) { simLegItem(ctx, lx, ly, SIM_ROAD_COLS[i], SIM_ROAD_LABELS[i], 'line'); lx += 190; }
        simLegItem(ctx, lx, ly, '#333333', 'Node', 'dot');         lx += 120;
        simLegItem(ctx, lx, ly, SIM_TL_COL, 'Traffic light node', 'dot');
    }

    // ── 2: Filtering ─────────────────────────────────────────────────────────
    {
        const pc = mkPc(1);
        simDrawStepHeader(ctx, 1, pc);
        clip(pc, () => {
            ctx.lineCap = 'round';
            for (const seg of rawSegs) {
                if (!retEdges.has(`${seg.from}-${seg.to}`)) continue;
                const nA = rawNodes[seg.from], nB = rawNodes[seg.to];
                if (!nA || !nB || (!inView(nA.x, nA.y) && !inView(nB.x, nB.y))) continue;
                const t = Math.min(seg.roadType ?? 5, 7);
                ctx.strokeStyle = SIM_ROAD_COLS[t]; ctx.lineWidth = SIM_ROAD_LW[t] * pc.ls;
                const [sx1,sy1] = simW2S(nA.x, nA.y, pc);
                const [sx2,sy2] = simW2S(nB.x, nB.y, pc);
                ctx.beginPath(); ctx.moveTo(sx1,sy1); ctx.lineTo(sx2,sy2); ctx.stroke();
            }
        });
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        for (let i = 0; i <= 4; i++) { simLegItem(ctx, lx, ly, SIM_ROAD_COLS[i], SIM_ROAD_LABELS[i], 'line'); lx += 190; }
    }

    // ── 3: Lane Reconstruction ────────────────────────────────────────────────
    // Extend endpoints ~10 m past where curb-trimming will later cut them,
    // so they visually overlap inside intersections (pre-trim state).
    const EXT_M = 10;
    const extLanes = lanes.map(l => {
        const pts = l.points;
        if (pts.length < 2) return l;
        const ext = pts.map(p => [...p]);
        const dx0 = pts[0][0] - pts[1][0], dy0 = pts[0][1] - pts[1][1];
        const d0 = Math.hypot(dx0, dy0) || 1;
        ext[0] = [pts[0][0] + dx0/d0*EXT_M, pts[0][1] + dy0/d0*EXT_M];
        const n = pts.length - 1;
        const dx1 = pts[n][0] - pts[n-1][0], dy1 = pts[n][1] - pts[n-1][1];
        const d1 = Math.hypot(dx1, dy1) || 1;
        ext[n] = [pts[n][0] + dx1/d1*EXT_M, pts[n][1] + dy1/d1*EXT_M];
        return { ...l, points: ext };
    });
    {
        const pc = mkPc(2);
        simDrawStepHeader(ctx, 2, pc);
        clip(pc, () => simDrawLanes(ctx, pc, extLanes,
            t => SIM_ROAD_COLS[Math.min(t,7)],
            t => SIM_ROAD_LW[Math.min(t,7)] * 1.3 * pc.ls,
        ));
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        for (let i = 0; i <= 5; i++) { simLegItem(ctx, lx, ly, SIM_ROAD_COLS[i], SIM_ROAD_LABELS[i], 'line'); lx += 175; }
    }

    // ── 4: Curb Trimming ──────────────────────────────────────────────────────
    {
        const pc = mkPc(3);
        simDrawStepHeader(ctx, 3, pc);
        clip(pc, () => {
            simDrawLanes(ctx, pc, lanes,
                t => SIM_ROAD_COLS[Math.min(t,7)],
                t => SIM_ROAD_LW[Math.min(t,7)] * 1.3 * pc.ls,
            );
            ctx.fillStyle = '#ff7700';
            for (const { points } of lanes) {
                if (!points.length) continue;
                for (const pt of [points[0], points[points.length-1]]) {
                    const [sx, sy] = simW2S(pt[0], pt[1], pc);
                    ctx.beginPath(); ctx.arc(sx, sy, 3.5 * pc.ls, 0, Math.PI*2); ctx.fill();
                }
            }
            ctx.strokeStyle = 'rgba(255,119,0,0.5)'; ctx.lineWidth = 1.5 * pc.ls;
            ctx.setLineDash([3, 3]);
            for (const inter of inters) {
                const [sx, sy] = simW2S(inter.x, inter.y, pc);
                ctx.beginPath(); ctx.arc(sx, sy, Math.max(3 * pc.ls, scale * 5), 0, Math.PI*2); ctx.stroke();
            }
            ctx.setLineDash([]);
        });
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        simLegItem(ctx, lx, ly, SIM_ROAD_COLS[2], 'Trimmed lane', 'line');                   lx += 220;
        simLegItem(ctx, lx, ly, '#ff7700', 'Trimmed endpoint', 'dot');                        lx += 230;
        simLegItem(ctx, lx, ly, 'rgba(255,119,0,0.5)', 'Intersection boundary', 'dashline');
    }

    // ── 5: Bézier Connections ─────────────────────────────────────────────────
    {
        const pc = mkPc(4);
        simDrawStepHeader(ctx, 4, pc);
        clip(pc, () => {
            simDrawLanes(ctx, pc, lanes, () => SIM_LANE_DIM, t => SIM_ROAD_LW[Math.min(t,7)] * 0.85 * pc.ls);
            simDrawIntPaths(ctx, pc, intPaths, SIM_BEZIER_COL, 1.8 * pc.ls);
        });
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        simLegItem(ctx, lx, ly, SIM_LANE_DIM,   'Lane centerline', 'line');          lx += 250;
        simLegItem(ctx, lx, ly, SIM_BEZIER_COL, 'Bézier connection path', 'line');
    }

    // ── 6: Traffic Signals ────────────────────────────────────────────────────
    {
        const pc = mkPc(5);
        simDrawStepHeader(ctx, 5, pc);
        clip(pc, () => {
            simDrawLanes(ctx, pc, lanes, () => SIM_LANE_DIM, t => SIM_ROAD_LW[Math.min(t,7)] * 0.85 * pc.ls);
            simDrawIntPaths(ctx, pc, intPaths, SIM_BEZIER_COL, 1.4 * pc.ls);
            simDrawTrafficLights(ctx, pc, inters);
        });
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        simLegItem(ctx, lx, ly, SIM_TL_COL,  'Traffic light', 'box');                         lx += 230;
        simLegItem(ctx, lx, ly, SIM_INT_COL, 'Priority / uncontrolled intersection', 'dot');
    }

    // ── 7: Parking & Spawn ────────────────────────────────────────────────────
    {
        const pc = mkPc(6);
        simDrawStepHeader(ctx, 6, pc);
        clip(pc, () => {
            simDrawLanes(ctx, pc, lanes, () => SIM_LANE_DIM, t => SIM_ROAD_LW[Math.min(t,7)] * 0.85 * pc.ls);
            simDrawIntPaths(ctx, pc, intPaths, SIM_LANE_DIM, 1.2 * pc.ls);
            simDrawSpawn(ctx, pc, spawnRegs);
            simDrawParking(ctx, pc, parkingSpots);
        });
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        simLegItem(ctx, lx, ly, SIM_SPAWN_COL, 'Spawn zone', 'rect');  lx += 210;
        simLegItem(ctx, lx, ly, SIM_PARK_COL,  'Parking spot', 'dot');
    }

    // ── 8: Buildings ──────────────────────────────────────────────────────────
    {
        const pc = mkPc(7);
        simDrawStepHeader(ctx, 7, pc);
        clip(pc, () => {
            simDrawBuildings(ctx, pc, buildings);
            simDrawLanes(ctx, pc, lanes,
                t => SIM_ROAD_COLS[Math.min(t,7)],
                t => SIM_ROAD_LW[Math.min(t,7)] * pc.ls,
            );
            simDrawIntPaths(ctx, pc, intPaths, SIM_LANE_DIM, 1.1 * pc.ls);
        });
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        simLegItem(ctx, lx, ly, SIM_BUILDING, 'Building footprint', 'box');  lx += 270;
        for (let i = 0; i <= 4; i++) { simLegItem(ctx, lx, ly, SIM_ROAD_COLS[i], SIM_ROAD_LABELS[i], 'line'); lx += 175; }
    }

    // ── 9: Live Simulation ────────────────────────────────────────────────────
    {
        const pc = mkPc(8);
        simDrawStepHeader(ctx, 8, pc);
        clip(pc, () => {
            simDrawBuildings(ctx, pc, buildings);
            simDrawLanes(ctx, pc, lanes,
                t => SIM_ROAD_COLS[Math.min(t,7)],
                t => SIM_ROAD_LW[Math.min(t,7)] * pc.ls,
            );
            simDrawIntPaths(ctx, pc, intPaths, SIM_LANE_DIM, 1.1 * pc.ls);
            simDrawAgents(ctx, pc, [...vehicleRenderStates.values()]);
        });
        const ly = simLegY(pc); let lx = pc.ox + pc.hPad;
        simLegItem(ctx, lx, ly, SIM_AGENT_COL,  'Moving vehicle', 'box');   lx += 230;
        simLegItem(ctx, lx, ly, SIM_PARKED_COL, 'Parked vehicle', 'box');
    }

    const a = document.createElement('a');
    a.download = `sim_pipeline_${new Date().toISOString().slice(0, 10)}.png`;
    a.href = cvs.toDataURL('image/png');
    a.click();

    simCancelExportMode();
}

document.getElementById('simExportBtn').addEventListener('click', simToggleExportMode);
document.getElementById('simExportGoBtn').addEventListener('click', simDoExport);
