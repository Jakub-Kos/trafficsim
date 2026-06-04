/*
 * Note: Claude (Anthropic) was used under human supervision to implement this
 * frontend UI. The sensitivity analysis method and DANCE scoring are the
 * author's own work in the C++ backend.
 */
// sensitivity.js — road sensitivity analysis (one-road-at-a-time DANC scan)
// Embedded in app.html #sensitivity-view

// ── State ─────────────────────────────────────────────────────────────────────
let svLaneData    = [];   // [{wayId, laneIdx, points, type}]
let svWayMeta     = {};   // wayId → {name}
let svSessionId   = null;
let svPollTimer   = null;
let svLastStatus  = null; // raw status from server or imported JSON
let svTpMult      = 0.5;  // Tp = simDurS * svTpMult
let svTeMult      = 1.0;  // Te = avgBlTripS * svTeMult
let svSortDesc    = true; // sort by fitness descending
let svHighlight   = null; // wayId highlighted by clicking in the list
let svMapReady    = false;
let svColorMin    = 0;    // fitness value mapped to green
let svColorMax    = 2;    // fitness value mapped to red
let svImported    = false; // true when svLastStatus came from an imported file

// ── Canvas state ──────────────────────────────────────────────────────────────
let svCanvas, svCtx;
const svCam = { cx: 0, cy: 0, scale: 1 };
let svDrag = null;

// ── Color helper ──────────────────────────────────────────────────────────────
function fitnessToColor(fitness) {
    // green (hue 120) at svColorMin, yellow at midpoint, red (hue 0) at svColorMax
    const range = Math.max(0.01, svColorMax - svColorMin);
    const t     = Math.max(0, Math.min(1, (fitness - svColorMin) / range));
    const hue   = Math.round(120 * (1 - t));
    return `hsl(${hue}, 88%, 52%)`;
}

// ── DANCE recomputation (frontend, no server round-trip) ──────────────────────
// Uses per-agent excludedCount from Approach B (server-side baseline replay).
// excluded agents pay Te each; truly_unserved = S0 - excluded - completed.
function calcDanceRaw(completedTravelS, completed, excludedCount, S0, Tp, Te) {
    const excluded = excludedCount ?? 0;
    const unserved = Math.max(0, S0 - excluded - completed);
    return (completedTravelS + unserved * Tp + excluded * Te) / Math.max(1, S0);
}

function recomputeFitnessMap(status, tpMult, teMult) {
    const map = new Map();
    if (!status || !status.baselineDone) return map;
    const bl      = status.baselineResult;
    const S0      = Math.max(1, status.baselineS0 || bl.spawned);
    const Tp      = status.simDurS * tpMult;
    const avgBlTrip = status.avgBlTripS ?? (bl.completed > 0 ? bl.completedTravelS / bl.completed : Tp);
    const Te      = avgBlTrip * teMult;
    // Baseline DANCE: excluded=0 by construction (no closures in baseline run)
    const blDance = calcDanceRaw(bl.completedTravelS, bl.completed, 0, S0, Tp, 0);
    for (const r of (status.results || [])) {
        const excluded = r.excludedCount ?? 0;
        const dance    = calcDanceRaw(r.completedTravelS, r.completed, excluded, S0, Tp, Te);
        map.set(r.wayId, {
            fitness: dance / Math.max(1e-9, blDance),
            danc: dance,
            name: r.name,
            spawned: r.spawned,
            completed: r.completed,
            excludedCount: excluded
        });
    }
    return map;
}

// ── Map rendering ─────────────────────────────────────────────────────────────
function svW() { return svCanvas.clientWidth; }
function svH() { return svCanvas.clientHeight; }
function svWorldToScreen(x, y) {
    return [svW() / 2 + (x - svCam.cx) * svCam.scale,
            svH() / 2 - (y - svCam.cy) * svCam.scale];
}
function svScreenToWorld(sx, sy) {
    return [(sx - svW() / 2) / svCam.scale + svCam.cx,
            -(sy - svH() / 2) / svCam.scale + svCam.cy];
}

function svResizeCanvas() {
    if (!svCanvas) return;
    const pane = svCanvas.parentElement;
    const dpr  = window.devicePixelRatio || 1;
    const w = pane.clientWidth, h = pane.clientHeight;
    svCanvas.style.width = w + 'px'; svCanvas.style.height = h + 'px';
    svCanvas.width  = Math.floor(w * dpr);
    svCanvas.height = Math.floor(h * dpr);
}

function svRender() {
    if (!svCanvas) { requestAnimationFrame(svRender); return; }
    const dpr = window.devicePixelRatio || 1;
    svCtx.save();
    svCtx.scale(dpr, dpr);
    svCtx.clearRect(0, 0, svCanvas.width / dpr, svCanvas.height / dpr);

    if (!svLaneData.length) { svCtx.restore(); requestAnimationFrame(svRender); return; }

    const fitnessMap = recomputeFitnessMap(svLastStatus, svTpMult, svTeMult);

    for (const { wayId, points } of svLaneData) {
        if (points.length < 2) continue;

        const info    = fitnessMap.get(wayId);
        const isHighlight = (svHighlight === wayId);

        let strokeColor;
        if (isHighlight) {
            strokeColor = '#ffffff';
        } else if (info != null) {
            strokeColor = fitnessToColor(info.fitness);
        } else {
            strokeColor = 'rgba(120,130,150,0.35)';
        }

        svCtx.beginPath();
        const [sx0, sy0] = svWorldToScreen(points[0][0], points[0][1]);
        svCtx.moveTo(sx0, sy0);
        for (let i = 1; i < points.length; i++) {
            const [sx, sy] = svWorldToScreen(points[i][0], points[i][1]);
            svCtx.lineTo(sx, sy);
        }
        svCtx.strokeStyle = strokeColor;
        svCtx.lineWidth   = isHighlight ? 3.5 : (info != null ? 2.5 : 1.2);
        svCtx.stroke();
    }

    svCtx.restore();
    requestAnimationFrame(svRender);
}

// ── Map loading ───────────────────────────────────────────────────────────────
async function svLoadMap() {
    try {
        const r    = await fetch('/map');
        const data = await r.json();
        svProcessMap(data);
        svMapReady = true;
    } catch (e) {
        console.error('[Sensitivity] map load failed:', e);
    }
}

function svProcessMap(data) {
    svLaneData = [];
    svWayMeta  = {};
    const wayInfo = {};
    for (const w of (data.ways || [])) {
        wayInfo[w.id] = { name: w.name || `Way ${w.id}` };
    }
    for (const l of (data.lanes || [])) {
        const pts = (l.points || []).map(p => [p.x, p.y]);
        if (pts.length < 2) continue;
        const wayId  = Math.floor(l.id / 1000);
        const info   = wayInfo[wayId] || { name: `Way ${wayId}` };
        svLaneData.push({ wayId, laneIdx: l.id % 1000, points: pts });
        if (!svWayMeta[wayId]) svWayMeta[wayId] = { name: info.name };
    }
    if (svLaneData.length) {
        let minX =  Infinity, maxX = -Infinity;
        let minY =  Infinity, maxY = -Infinity;
        for (const { points } of svLaneData) {
            for (const [x, y] of points) {
                if (x < minX) minX = x; if (x > maxX) maxX = x;
                if (y < minY) minY = y; if (y > maxY) maxY = y;
            }
        }
        svCam.cx = (minX + maxX) / 2;
        svCam.cy = (minY + maxY) / 2;
        const span = Math.max(maxX - minX, maxY - minY);
        svCam.scale = span > 0 ? Math.min(svW(), svH()) * 0.85 / span : 1;
    }
}

// ── Hit test ──────────────────────────────────────────────────────────────────
function svHitTest(sx, sy) {
    const [wx, wy] = svScreenToWorld(sx, sy);
    let bestDist = (12 / svCam.scale) ** 2;
    let bestWayId = null;
    for (const { wayId, points } of svLaneData) {
        for (let i = 0; i < points.length - 1; i++) {
            const [ax, ay] = points[i], [bx, by] = points[i + 1];
            const dx = bx - ax, dy = by - ay;
            const len2 = dx * dx + dy * dy;
            if (len2 === 0) continue;
            const t = Math.max(0, Math.min(1, ((wx - ax) * dx + (wy - ay) * dy) / len2));
            const d = (wx - ax - t * dx) ** 2 + (wy - ay - t * dy) ** 2;
            if (d < bestDist) { bestDist = d; bestWayId = wayId; }
        }
    }
    return bestWayId;
}

// ── Tooltip ───────────────────────────────────────────────────────────────────
function svShowTooltip(wayId, ex, ey) {
    const tt = document.getElementById('sv-tooltip');
    if (!tt) return;
    const meta = svWayMeta[wayId];
    const fm   = recomputeFitnessMap(svLastStatus, svTpMult, svTeMult);
    const info = fm.get(wayId);
    let html = `<div style="font-size:11px;font-weight:600;color:var(--c-text-hi)">${meta?.name || `Way ${wayId}`}</div>`;
    if (info != null) {
        html += `<div style="font-size:10px;color:${fitnessToColor(info.fitness)}">Fitness: ${info.fitness.toFixed(4)}</div>`;
        html += `<div style="font-size:10px;color:var(--c-text-dim)">DANCE: ${info.danc.toFixed(2)} s/agent</div>`;
        const pct = info.spawned > 0 ? Math.round(100 * info.completed / info.spawned) : 0;
        html += `<div style="font-size:10px;color:var(--c-text-dim)">Completion: ${pct}%</div>`;
        if (info.excludedCount > 0) {
            const S0 = Math.max(1, status?.baselineS0 || 1);
            const exclPct = Math.round(100 * info.excludedCount / S0);
            html += `<div style="font-size:10px;color:var(--c-text-dim)">Excluded agents: ${info.excludedCount} (${exclPct}%)</div>`;
        }
    } else {
        html += `<div style="font-size:10px;color:var(--c-text-dim)">Not yet tested</div>`;
    }
    tt.innerHTML = html;
    const rect = svCanvas.getBoundingClientRect();
    tt.style.left = (ex - rect.left + 14) + 'px';
    tt.style.top  = (ey - rect.top  - 10) + 'px';
    tt.style.display = 'block';
}

function svHideTooltip() {
    const tt = document.getElementById('sv-tooltip');
    if (tt) tt.style.display = 'none';
}

// ── Run control ───────────────────────────────────────────────────────────────
async function svStartAnalysis() {
    const dur      = parseInt(document.querySelector('#sv-durChips .sv-chip.sv-chip-sel')?.dataset.val || '3600');
    const parallel = parseInt(document.getElementById('sv-parallel')?.value || '5');
    const traffic  = document.getElementById('sv-traffic')?.value || 'medium';

    try {
        const r = await fetch('/sensitivity/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ simDurationS: dur, maxParallel: parallel, trafficMode: traffic })
        });
        const d = await r.json();
        if (!r.ok) throw new Error(d.error || r.statusText);
        svSessionId = d.sessionId;
        svLastStatus = null;
        svSetRunning(true);
        svPollTimer  = setInterval(svPollStatus, 1500);
    } catch (e) {
        svSetStatus(`Error: ${e.message}`);
    }
}

async function svCancelAnalysis() {
    if (!svSessionId) return;
    try {
        await fetch(`/sensitivity/${svSessionId}/cancel`, { method: 'POST' });
    } catch {}
    clearInterval(svPollTimer);
    svPollTimer = null;
    svSetRunning(false);
    svSetStatus('Cancelled.');
}

// ── Polling ───────────────────────────────────────────────────────────────────
async function svPollStatus() {
    if (!svSessionId) return;
    try {
        const r = await fetch(`/sensitivity/${svSessionId}/status`);
        if (!r.ok) return;
        const status = await r.json();
        svLastStatus = status;
        svUpdateUI(status);
        if (status.status === 'completed' || status.status === 'cancelled' || status.status === 'error') {
            clearInterval(svPollTimer);
            svPollTimer = null;
            svSetRunning(false);
        }
    } catch {}
}

// ── UI update ─────────────────────────────────────────────────────────────────
function svSetRunning(running) {
    const runBtn    = document.getElementById('sv-runBtn');
    const cancelBtn = document.getElementById('sv-cancelBtn');
    const progWrap  = document.getElementById('sv-progressWrap');
    if (runBtn)    runBtn.style.display    = running ? 'none' : '';
    if (cancelBtn) cancelBtn.style.display = running ? ''     : 'none';
    if (progWrap)  progWrap.style.display  = running ? ''     : 'none';
}

function svSetStatus(msg) {
    const el = document.getElementById('sv-statusText');
    if (el) el.textContent = msg;
}

function svFmtTime(s) {
    if (s < 0 || !isFinite(s)) return '';
    if (s < 60)   return `~${Math.ceil(s)}s remaining`;
    if (s < 3600) return `~${Math.round(s / 60)}m remaining`;
    return `~${Math.floor(s / 3600)}h ${Math.round((s % 3600) / 60)}m remaining`;
}

function svUpdateUI(status) {
    // Status text
    if (status.status === 'loading' || status.status === 'baseline') {
        svSetStatus('Computing baseline…');
    } else if (status.status === 'running') {
        svSetStatus(`Testing roads: ${status.roadsDone} / ${status.totalRoads}`);
    } else if (status.status === 'completed') {
        svSetStatus(`Done — ${status.roadsDone} roads analysed.`);
    } else if (status.status === 'cancelled') {
        svSetStatus(`Cancelled — ${status.roadsDone} roads collected.`);
    }

    // Progress bar
    const fill = document.getElementById('sv-progressFill');
    if (fill && status.totalRoads > 0) {
        // +1 for baseline phase
        const pct = Math.round(100 * (status.roadsDone + (status.baselineDone ? 1 : 0)) / (status.totalRoads + 1));
        fill.style.width = pct + '%';
    }

    // ETA
    const eta = document.getElementById('sv-etaText');
    if (eta) {
        if (!status.baselineDone) {
            eta.textContent = 'Baseline running…';
        } else {
            eta.textContent = svFmtTime(status.etaSeconds);
        }
    }

    // Show DANC tuner + legend once baseline is done
    if (status.baselineDone) {
        const dSection = document.getElementById('sv-dancSection');
        const lSection = document.getElementById('sv-legendSection');
        if (dSection) dSection.style.display = '';
        if (lSection) lSection.style.display = '';

        const blEl = document.getElementById('sv-blDancVal');
        if (blEl) {
            const bl    = status.baselineResult;
            const S0    = Math.max(1, status.baselineS0 || bl.spawned);
            const Tp    = status.simDurS * svTpMult;
            // Baseline: excluded=0, Te irrelevant
            const blDance = calcDanceRaw(bl.completedTravelS, bl.completed, 0, S0, Tp, 0);
            blEl.textContent = blDance.toFixed(2) + ' s/agent';
        }

        const tpLabel = document.getElementById('sv-tpLabel');
        if (tpLabel) {
            const Tp = status.simDurS * svTpMult;
            tpLabel.textContent = `${Tp.toFixed(0)} s (${svTpMult.toFixed(2)}× simDuration)`;
        }

        const teLabel = document.getElementById('sv-teLabel');
        if (teLabel) {
            const avgBlTrip = status.avgBlTripS
                ?? (status.baselineResult?.completed > 0
                    ? status.baselineResult.completedTravelS / status.baselineResult.completed
                    : status.simDurS * 0.5);
            const Te = avgBlTrip * svTeMult;
            teLabel.textContent = `${Te.toFixed(0)} s (${svTeMult.toFixed(2)}× avgBlTrip = ${avgBlTrip.toFixed(0)} s)`;
        }
    }

    // Results list
    if (status.results && status.results.length > 0) {
        const rSection = document.getElementById('sv-resultsSection');
        if (rSection) rSection.style.display = '';
        svRenderResultsList(status);
    }
}

function svRenderResultsList(status) {
    const container = document.getElementById('sv-resultsList');
    if (!container) return;

    const fm     = recomputeFitnessMap(status, svTpMult, svTeMult);
    const rows   = Array.from(fm.entries()).map(([wayId, info]) => ({ wayId, ...info }));
    rows.sort((a, b) => svSortDesc ? b.fitness - a.fitness : a.fitness - b.fitness);

    const html = rows.map(r => {
        const f    = r.fitness;
        const col  = fitnessToColor(f);
        const pct  = r.spawned > 0 ? Math.round(100 * r.completed / r.spawned) : 0;
        const isHi = svHighlight === r.wayId;
        return `<div class="sv-road-row${isHi ? ' sv-road-row-hi' : ''}" data-wayid="${r.wayId}" title="${r.name}">
            <span class="sv-road-swatch" style="background:${col}"></span>
            <span class="sv-road-name">${r.name}</span>
            <span class="sv-road-fitness" style="color:${col};font-family:var(--c-mono)">${f.toFixed(3)}</span>
            <span class="sv-road-compl">${pct}%</span>
        </div>`;
    }).join('');

    container.innerHTML = html || '<div style="padding:8px;font-size:11px;color:var(--c-text-dim)">No results yet.</div>';

    container.querySelectorAll('.sv-road-row').forEach(row => {
        row.addEventListener('click', () => {
            const wid = parseInt(row.dataset.wayid);
            svHighlight = (svHighlight === wid) ? null : wid;
            svRenderResultsList(status);
        });
    });
}

// ── Tp / Te sliders ───────────────────────────────────────────────────────────
function svOnTpChange(val) {
    svTpMult = parseFloat(val);
    if (svLastStatus) svUpdateUI(svLastStatus);
}

function svOnTeChange(val) {
    svTeMult = parseFloat(val);
    if (svLastStatus) svUpdateUI(svLastStatus);
}

// ── Export / Import / Reset ───────────────────────────────────────────────────
function svExportJSON() {
    if (!svLastStatus) return;
    const blob = new Blob([JSON.stringify(svLastStatus, null, 2)], { type: 'application/json' });
    const url  = URL.createObjectURL(blob);
    const a    = document.createElement('a');
    a.href = url;
    a.download = `sensitivity_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.json`;
    a.click();
    URL.revokeObjectURL(url);
}

function svImportJSON() {
    document.getElementById('sv-importFile')?.click();
}

function svOnFileSelected(e) {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = evt => {
        try {
            const data = JSON.parse(evt.target.result);
            svLastStatus = data;
            svImported   = true;
            document.getElementById('sv-resetBtn').style.display = '';
            svSetRunning(false);
            svSetStatus('Imported: ' + file.name);
            ['sv-dancSection', 'sv-legendSection', 'sv-resultsSection'].forEach(id => {
                const el = document.getElementById(id);
                if (el) el.style.display = '';
            });
            svUpdateUI(data);
        } catch (err) {
            svSetStatus('Import error: ' + err.message);
        }
    };
    reader.readAsText(file);
    e.target.value = ''; // allow re-importing the same file
}

function svResetImport() {
    svLastStatus = null;
    svImported   = false;
    document.getElementById('sv-resetBtn').style.display = 'none';
    svSetStatus('Ready.');
    ['sv-dancSection', 'sv-legendSection', 'sv-resultsSection'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.style.display = 'none';
    });
    const list = document.getElementById('sv-resultsList');
    if (list) list.innerHTML = '';
}

// ── Color range ───────────────────────────────────────────────────────────────
function svUpdateLegendLabels() {
    const mid = ((svColorMin + svColorMax) / 2).toFixed(2);
    const lblMin = document.getElementById('sv-lbl-min');
    const lblMid = document.getElementById('sv-lbl-mid');
    const lblMax = document.getElementById('sv-lbl-max');
    if (lblMin) lblMin.textContent = svColorMin % 1 === 0 ? String(svColorMin) : svColorMin.toFixed(2);
    if (lblMid) lblMid.textContent = mid;
    if (lblMax) lblMax.textContent = svColorMax % 1 === 0 ? String(svColorMax) : svColorMax.toFixed(2);
}

function svOnColorRangeChange() {
    const minEl = document.getElementById('sv-colorMin');
    const maxEl = document.getElementById('sv-colorMax');
    svColorMin = parseFloat(minEl?.value ?? '0') || 0;
    svColorMax = parseFloat(maxEl?.value ?? '2') || 2;
    if (svColorMax <= svColorMin) svColorMax = svColorMin + 0.01;
    svUpdateLegendLabels();
    if (svLastStatus) svRenderResultsList(svLastStatus);
}

// ── Sort toggle ───────────────────────────────────────────────────────────────
function svToggleSort() {
    svSortDesc = !svSortDesc;
    const btn = document.getElementById('sv-sortHdr');
    if (btn) btn.textContent = svSortDesc ? '▼ IMPACT' : '▲ IMPACT';
    if (svLastStatus) svRenderResultsList(svLastStatus);
}

// ── Init ──────────────────────────────────────────────────────────────────────
function initSensitivity() {
    if (svMapReady) return; // already initialised

    svCanvas = document.getElementById('sv-mapCanvas');
    if (!svCanvas) return;
    svCtx = svCanvas.getContext('2d');
    svResizeCanvas();
    window.addEventListener('resize', svResizeCanvas);

    // Pan
    svCanvas.addEventListener('mousedown', e => {
        if (e.button !== 0) return;
        svDrag = { sx: e.clientX, sy: e.clientY, cx0: svCam.cx, cy0: svCam.cy };
    });
    window.addEventListener('mousemove', e => {
        if (svDrag) {
            svCam.cx = svDrag.cx0 - (e.clientX - svDrag.sx) / svCam.scale;
            svCam.cy = svDrag.cy0 + (e.clientY - svDrag.sy) / svCam.scale;
        } else {
            const rect = svCanvas.getBoundingClientRect();
            const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
            if (sx >= 0 && sy >= 0 && sx <= rect.width && sy <= rect.height) {
                const wid = svHitTest(sx, sy);
                if (wid != null) svShowTooltip(wid, e.clientX, e.clientY);
                else             svHideTooltip();
            } else {
                svHideTooltip();
            }
        }
    });
    window.addEventListener('mouseup', () => { svDrag = null; });
    svCanvas.addEventListener('mouseleave', svHideTooltip);

    // Zoom
    svCanvas.addEventListener('wheel', e => {
        e.preventDefault();
        const rect   = svCanvas.getBoundingClientRect();
        const sx     = e.clientX - rect.left, sy = e.clientY - rect.top;
        const [wx, wy] = svScreenToWorld(sx, sy);
        const factor = Math.pow(1.001, -e.deltaY);
        svCam.scale  = Math.max(0.1, Math.min(500, svCam.scale * factor));
        svCam.cx     = wx - (sx - svW() / 2) / svCam.scale;
        svCam.cy     = wy + (sy - svH() / 2) / svCam.scale;
    }, { passive: false });

    // Click to highlight road
    svCanvas.addEventListener('click', e => {
        const rect = svCanvas.getBoundingClientRect();
        const wid  = svHitTest(e.clientX - rect.left, e.clientY - rect.top);
        if (wid != null) {
            svHighlight = (svHighlight === wid) ? null : wid;
            if (svLastStatus) svRenderResultsList(svLastStatus);
        }
    });

    // Chip wiring (sim duration)
    document.querySelectorAll('#sv-durChips .sv-chip').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('#sv-durChips .sv-chip').forEach(b => b.classList.remove('sv-chip-sel'));
            btn.classList.add('sv-chip-sel');
        });
    });

    // Run / cancel buttons
    document.getElementById('sv-runBtn')?.addEventListener('click', svStartAnalysis);
    document.getElementById('sv-cancelBtn')?.addEventListener('click', svCancelAnalysis);

    // Export / Import / Reset
    document.getElementById('sv-exportBtn')?.addEventListener('click', svExportJSON);
    document.getElementById('sv-importBtn')?.addEventListener('click', svImportJSON);
    document.getElementById('sv-resetBtn')?.addEventListener('click', svResetImport);
    document.getElementById('sv-importFile')?.addEventListener('change', svOnFileSelected);

    // Tp / Te sliders
    const tpSlider = document.getElementById('sv-tpSlider');
    if (tpSlider) tpSlider.addEventListener('input', e => svOnTpChange(e.target.value));
    const teSlider = document.getElementById('sv-teSlider');
    if (teSlider) teSlider.addEventListener('input', e => svOnTeChange(e.target.value));

    // Color range inputs
    document.getElementById('sv-colorMin')?.addEventListener('input', svOnColorRangeChange);
    document.getElementById('sv-colorMax')?.addEventListener('input', svOnColorRangeChange);

    // Sort header click
    document.getElementById('sv-sortHdr')?.addEventListener('click', svToggleSort);

    requestAnimationFrame(svRender);
    svLoadMap();
}

window.__initSensitivity = initSensitivity;