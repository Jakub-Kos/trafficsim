/*
 * Note: Claude (Anthropic) was used under human supervision to implement the
 * app shell UI in this file — menubar, toolbar, dock section management, and
 * statusbar — building upon the overall frontend architecture established by
 * the author.
 */
/**
 * app.js — App shell controller
 * Drives: menubar dropdowns, left toolbar, dock section collapse,
 *         Window menu sync, project name display, statusbar.
 */

// ── Dock section collapse ─────────────────────────────────────────────────────
// Sections with class "open" show their .dock-body; others collapse.
// State persisted in localStorage.

const DOCK_STORAGE_KEY = 'dockState_v1';

function loadDockState() {
    try { return JSON.parse(localStorage.getItem(DOCK_STORAGE_KEY) || '{}'); } catch { return {}; }
}
function saveDockState() {
    const out = {};
    document.querySelectorAll('.dock-section[id]').forEach(s => {
        out[s.id] = s.classList.contains('open');
    });
    try { localStorage.setItem(DOCK_STORAGE_KEY, JSON.stringify(out)); } catch {}
}

// Apply persisted open/closed state on load
(function initDockState() {
    const saved = loadDockState();
    document.querySelectorAll('.dock-section[id]').forEach(sec => {
        if (sec.id === 'dock-inspector') return; // inspector managed separately
        if (sec.id in saved) {
            sec.classList.toggle('open', saved[sec.id]);
        }
    });
})();

// Wire dock header clicks
document.querySelectorAll('.dock-hdr[data-dock]').forEach(hdr => {
    hdr.addEventListener('click', () => {
        const sec = document.getElementById(hdr.dataset.dock);
        if (!sec) return;
        sec.classList.toggle('open');
        saveDockState();
        syncWindowMenu();
    });
});

// Toggle dock section visibility from Window menu
function setDockSectionVisible(id, visible) {
    const sec = document.getElementById(id);
    if (!sec) return;
    if (visible) {
        sec.style.display = '';
        sec.classList.add('open');
    } else {
        sec.style.display = 'none';
    }
    saveDockState();
    syncWindowMenu();
}

function isDockSectionVisible(id) {
    const sec = document.getElementById(id);
    return sec ? sec.style.display !== 'none' : false;
}

// ── Project name ─────────────────────────────────────────────────────────────
// Ask the backend which project is loaded — this is authoritative and fixes
// stale sessionStorage (e.g. user had "New York" loaded before, now has "Default").
function applyProjectName(name) {
    if (!name) return;
    sessionStorage.setItem('currentProject', name);
    document.title = `TrafficSim — ${name}`;
    const el1 = document.getElementById('mbProjectName');
    const el2 = document.getElementById('sb-proj');
    if (el1) el1.textContent = name;
    if (el2) el2.textContent = name;
}

// Apply whatever sessionStorage has immediately (avoids blank flash)
applyProjectName(sessionStorage.getItem('currentProject') || '');

// Then confirm with the backend and correct if needed
fetch('/simulation/project')
    .then(r => r.ok ? r.json() : null)
    .then(data => { if (data?.projectName) applyProjectName(data.projectName); })
    .catch(() => {});

// ── Left toolbar — tool selection ─────────────────────────────────────────────
const toolSelect  = document.getElementById('toolSelect');
const toolRoad    = document.getElementById('toolRoad');
const roadEditBtn = document.getElementById('roadEditBtn');

toolSelect.addEventListener('click', () => {
    toolSelect.classList.add('active');
    toolRoad.classList.remove('active');
    if (roadEditBtn.classList.contains('active')) roadEditBtn.click();
});
toolRoad.addEventListener('click', () => {
    toolRoad.classList.add('active');
    toolSelect.classList.remove('active');
    if (!roadEditBtn.classList.contains('active')) roadEditBtn.click();
});

// Keep left toolbar in sync when road edit is toggled from keyboard/other sources
new MutationObserver(() => {
    const active = roadEditBtn.classList.contains('active');
    toolRoad.classList.toggle('active', active);
    toolSelect.classList.toggle('active', !active);
}).observe(roadEditBtn, { attributes: true, attributeFilter: ['class'] });

// ── Menubar dropdown system ───────────────────────────────────────────────────
function closeAllMenus() {
    document.querySelectorAll('.mb-dropdown.open').forEach(d => d.classList.remove('open'));
    document.querySelectorAll('.mb-menu-btn.open').forEach(b => b.classList.remove('open'));
}

document.querySelectorAll('.mb-menu-btn[data-menu]').forEach(btn => {
    btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const dropdown = document.getElementById('menu-' + btn.dataset.menu);
        const wasOpen  = btn.classList.contains('open');
        closeAllMenus();
        if (!wasOpen) {
            btn.classList.add('open');
            dropdown.classList.add('open');
        }
    });
});

document.addEventListener('click', closeAllMenus);
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') closeAllMenus(); });

// ── File menu ─────────────────────────────────────────────────────────────────
document.getElementById('mbNewProject').addEventListener('click',      () => { closeAllMenus(); window.location.href = '/?new=1'; });
document.getElementById('mbBackToProjects').addEventListener('click',  () => { closeAllMenus(); window.location.href = '/'; });

// ── View menu — sync checkmarks with toolbar view buttons ─────────────────────
const viewBtnMap = {
    agents:     document.getElementById('viewAgents'),
    density:    document.getElementById('viewDensity'),
    congestion: document.getElementById('viewCongestion'),
};

function updateViewChecks() {
    for (const [mode, btn] of Object.entries(viewBtnMap)) {
        const chk = document.getElementById('chk-' + mode);
        if (btn && chk) chk.textContent = btn.classList.contains('active') ? '✓' : ' ';
    }
}

Object.entries(viewBtnMap).forEach(([mode, btn]) => {
    if (!btn) return;
    new MutationObserver(updateViewChecks).observe(btn, { attributes: true, attributeFilter: ['class'] });
    const item = document.querySelector(`.mb-view-item[data-mode="${mode}"]`);
    if (item) item.addEventListener('click', () => { closeAllMenus(); btn.click(); });
});
updateViewChecks();

document.getElementById('mbFitView').addEventListener('click', () => {
    closeAllMenus();
    window.dispatchEvent(new KeyboardEvent('keydown', { key: 'f', code: 'KeyF', bubbles: true }));
});

// ── Simulation menu ───────────────────────────────────────────────────────────
document.getElementById('mbPlayPause').addEventListener('click', () => {
    closeAllMenus(); document.getElementById('pauseBtn').click();
});
document.querySelectorAll('[data-sim-speed]').forEach(item => {
    item.addEventListener('click', () => {
        closeAllMenus();
        const btn = document.querySelector(`[data-speed="${item.dataset.simSpeed}"]`);
        if (btn) btn.click();
    });
});

// ── Window menu — toggle dock sections ───────────────────────────────────────
function syncWindowMenu() {
    const map = { 'dock-metrics': 'chk-metrics', 'dock-debug': 'chk-debug', 'dock-spawn': 'chk-spawn', 'dock-perf': 'chk-perf' };
    for (const [id, chkId] of Object.entries(map)) {
        const chk = document.getElementById(chkId);
        if (chk) chk.textContent = isDockSectionVisible(id) ? '✓' : ' ';
    }
}

document.querySelectorAll('[data-dock]').forEach(item => {
    // Only wire Window menu items (not dock headers, which have data-dock too)
    if (item.closest('.mb-dropdown')) {
        item.addEventListener('click', () => {
            closeAllMenus();
            const id = item.dataset.dock;
            setDockSectionVisible(id, !isDockSectionVisible(id));
        });
    }
});
syncWindowMenu();

// ── Pause button — statusbar indicator ───────────────────────────────────────
const sbIndicator = document.getElementById('sbIndicator');
const pauseBtn    = document.getElementById('pauseBtn');
function syncIndicator() { sbIndicator.classList.toggle('paused', pauseBtn.classList.contains('paused')); }
new MutationObserver(syncIndicator).observe(pauseBtn, { attributes: true, attributeFilter: ['class'] });
syncIndicator();

// ── Statusbar FPS (mirror from perf panel) ────────────────────────────────────
const sbFps   = document.getElementById('sb-fps');
const perfFps = document.getElementById('perf-fps');
if (perfFps && sbFps) {
    new MutationObserver(() => {
        const match = (perfFps.innerText || '').match(/(\d+)\s*FPS/);
        sbFps.textContent = match ? match[1] + ' FPS' : '';
    }).observe(perfFps, { childList: true, subtree: true });
}

// ── Keyboard shortcuts ────────────────────────────────────────────────────────
window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT') return;
    if (e.key.toLowerCase() === 's') toolSelect.click();
    if (e.key.toLowerCase() === 'r') toolRoad.click();
});

// ── View tab switching ─────────────────────────────────────────────────────────

const analyticsView    = document.getElementById('analytics-view');
const optimizerView    = document.getElementById('optimizer-view');
const sensitivityView  = document.getElementById('sensitivity-view');
const tabSim           = document.getElementById('tabSim');
const tabAnalytics     = document.getElementById('tabAnalytics');
const tabOptimizer     = document.getElementById('tabOptimizer');
const tabSensitivity   = document.getElementById('tabSensitivity');

function showAnalyticsView() {
    analyticsView.classList.add('open');
    optimizerView?.classList.remove('open');
    sensitivityView?.classList.remove('open');
    tabSim.classList.remove('active');
    tabAnalytics.classList.add('active');
    tabOptimizer?.classList.remove('active');
    tabSensitivity?.classList.remove('active');
    refreshSidebar();
    updateClosedHint();
}

function showSimView() {
    analyticsView.classList.remove('open');
    optimizerView?.classList.remove('open');
    sensitivityView?.classList.remove('open');
    tabSim.classList.add('active');
    tabAnalytics.classList.remove('active');
    tabOptimizer?.classList.remove('active');
    tabSensitivity?.classList.remove('active');
}

function showOptimizerView() {
    analyticsView.classList.remove('open');
    optimizerView?.classList.add('open');
    sensitivityView?.classList.remove('open');
    tabSim.classList.remove('active');
    tabAnalytics.classList.remove('active');
    tabOptimizer?.classList.add('active');
    tabSensitivity?.classList.remove('active');
    window.__initOptimizer?.();
}

function showSensitivityView() {
    analyticsView.classList.remove('open');
    optimizerView?.classList.remove('open');
    sensitivityView?.classList.add('open');
    tabSim.classList.remove('active');
    tabAnalytics.classList.remove('active');
    tabOptimizer?.classList.remove('active');
    tabSensitivity?.classList.add('active');
    window.__initSensitivity?.();
}

tabSim?.addEventListener('click', showSimView);
tabAnalytics?.addEventListener('click', showAnalyticsView);
tabOptimizer?.addEventListener('click', showOptimizerView);
tabSensitivity?.addEventListener('click', showSensitivityView);

document.getElementById('avProgBackBtn')?.addEventListener('click', showSimView);

// ── Page navigation ───────────────────────────────────────────────────────────

function showPage(id) {
    document.querySelectorAll('.av-page').forEach(p => p.classList.remove('active'));
    document.getElementById(id)?.classList.add('active');
}

// ── Chip groups ───────────────────────────────────────────────────────────────

function wireChips(groupId) {
    const g = document.getElementById(groupId);
    if (!g) return;
    g.querySelectorAll('.av-chip').forEach(btn => {
        btn.addEventListener('click', () => {
            g.querySelectorAll('.av-chip').forEach(b => b.classList.remove('sel'));
            btn.classList.add('sel');
        });
    });
}
wireChips('avRunCountChips');
wireChips('avTrafficChips');
wireChips('avDurationChips');

function chipVal(groupId) {
    return document.getElementById(groupId)?.querySelector('.av-chip.sel')?.dataset.val ?? null;
}

// ── Closed-roads hint ─────────────────────────────────────────────────────────

async function updateClosedHint() {
    const el = document.getElementById('avClosedHint');
    if (!el) return;
    try {
        const r = await fetch('/road/closed');
        if (!r.ok) return;
        const data = await r.json();
        const n = (data.closedRoadIds || []).length;
        el.innerHTML = n > 0
            ? `<strong>${n} road${n !== 1 ? 's' : ''} closed</strong> — will run as <em>scenario</em>`
            : `No roads closed — will run as <em>baseline</em>`;
    } catch { el.innerHTML = ''; }
}

// ── State ─────────────────────────────────────────────────────────────────────

let _avAnalyses    = [];           // [{analysisId, name, createdAt, sessions:[]}]
let _avPollInterval = null;
let _avActiveSession = null;       // {id, analysisId, label, total}
let _avCurrentAnalysisId = null;   // which analysis is selected in comparison view
let _avIntersectionPos = null;     // cached [{id,x,y}] from /intersections/positions

// ── Sidebar ───────────────────────────────────────────────────────────────────

async function refreshSidebar() {
    const proj = sessionStorage.getItem('currentProject') || '';
    const el   = document.getElementById('av-sessions-list');
    if (!el) return;
    if (!proj) {
        el.innerHTML = '<div style="padding:12px 8px;font-size:11px;color:var(--c-text-dim)">No project loaded.</div>';
        return;
    }
    try {
        const r = await fetch(`/projects/${encodeURIComponent(proj)}/analyses`);
        if (!r.ok) {
            el.innerHTML = '<div style="padding:12px 8px;font-size:11px;color:var(--c-text-dim)">DB unavailable</div>';
            return;
        }
        _avAnalyses = await r.json();
        renderSidebar();
    } catch {
        el.innerHTML = '<div style="padding:12px 8px;font-size:11px;color:var(--c-danger)">Failed to load</div>';
    }
}

function renderSidebar() {
    const el = document.getElementById('av-sessions-list');
    if (!el) return;
    if (!_avAnalyses.length) {
        el.innerHTML = '<div style="padding:12px 8px;font-size:11px;color:var(--c-text-dim)">No analyses yet. Click "+ New" to create one.</div>';
        return;
    }

    el.innerHTML = _avAnalyses.map(a => {
        const isOpen   = a.analysisId === _avCurrentAnalysisId;
        const sessions = (a.sessions || []);
        const anyRunning = sessions.some(s => s.status === 'running');
        const dotCls = anyRunning ? 'av-dot av-dot-warn' : 'av-dot av-dot-ok';

        const simItems = sessions.map(s => {
            const dot = s.status === 'running' ? 'av-dot-warn' : s.status === 'failed' ? 'av-dot-err' : 'av-dot-ok';
            const baselineTag = s.isBaseline ? '<span class="baseline-tag">BL</span>' : '';
            return `<div class="av-sim-item" data-sid="${escHtml(s.sessionId)}" data-aid="${escHtml(a.analysisId)}">
                <div class="av-dot ${dot}" style="width:5px;height:5px;flex-shrink:0"></div>
                <div class="av-sim-label">${escHtml(s.name || 'Simulation')}${baselineTag}</div>
            </div>`;
        }).join('');

        return `<div class="av-grp${isOpen ? ' active open' : ''}" data-aid="${escHtml(a.analysisId)}">
            <div class="av-grp-hdr" data-aid="${escHtml(a.analysisId)}">
                <span class="av-grp-arrow">▶</span>
                <span class="av-grp-name">${escHtml(a.name)}</span>
                <button class="av-grp-del" data-aid="${escHtml(a.analysisId)}" title="Delete analysis">✕</button>
            </div>
            <div class="av-grp-sessions">${simItems}</div>
        </div>`;
    }).join('');

    // Wire group header clicks
    el.querySelectorAll('.av-grp-hdr').forEach(hdr => {
        hdr.addEventListener('click', e => {
            if (e.target.closest('.av-grp-del')) return;
            const aid = hdr.dataset.aid;
            const grp = hdr.closest('.av-grp');
            const wasOpen = grp.classList.contains('open');
            // Collapse all, then open clicked
            el.querySelectorAll('.av-grp').forEach(g => g.classList.remove('open', 'active'));
            if (!wasOpen) {
                grp.classList.add('open', 'active');
                _avCurrentAnalysisId = aid;
                showComparisonPage(aid);
            } else {
                _avCurrentAnalysisId = null;
                showPage('av-pg-welcome');
            }
        });
    });

    // Wire sim item clicks
    el.querySelectorAll('.av-sim-item').forEach(item => {
        item.addEventListener('click', () => {
            const sid = item.dataset.sid;
            const aid = item.dataset.aid;
            el.querySelectorAll('.av-sim-item').forEach(i => i.classList.remove('active'));
            item.classList.add('active');
            const analysis = _avAnalyses.find(a => a.analysisId === aid);
            const session  = analysis?.sessions?.find(s => s.sessionId === sid);
            if (session) showSimPage(session, analysis);
        });
    });

    // Wire delete buttons
    el.querySelectorAll('.av-grp-del').forEach(btn => {
        btn.addEventListener('click', e => {
            e.stopPropagation();
            deleteAnalysis(btn.dataset.aid);
        });
    });
}

async function deleteAnalysis(analysisId) {
    const a = _avAnalyses.find(x => x.analysisId === analysisId);
    const name = a?.name || analysisId.slice(0, 8);
    const hasRunning = (a?.sessions || []).some(s => s.status === 'running');
    const msg = hasRunning
        ? `Stop all running simulations and delete "${name}"?\nThis cannot be undone.`
        : `Delete analysis "${name}" and all its simulations?\nThis cannot be undone.`;
    if (!confirm(msg)) return;

    // Stop polling if any session of this analysis is active
    if (_avActiveSession?.analysisId === analysisId) {
        if (_avPollInterval) { clearInterval(_avPollInterval); _avPollInterval = null; }
        _avActiveSession = null;
    }

    try {
        const r = await fetch(`/analyses/${encodeURIComponent(analysisId)}`, { method: 'DELETE' });
        if (!r.ok) { const d = await r.json().catch(() => ({})); alert('Delete failed: ' + (d.error || r.status)); return; }
        if (_avCurrentAnalysisId === analysisId) {
            _avCurrentAnalysisId = null;
            showPage('av-pg-welcome');
        }
        await refreshSidebar();
    } catch (e) {
        alert('Delete failed: ' + e.message);
    }
}

// ── Create Analysis (Step 1) ──────────────────────────────────────────────────

document.getElementById('av-new-btn')?.addEventListener('click', () => showPage('av-pg-new-analysis'));
document.getElementById('avWelcomeNewBtn')?.addEventListener('click', () => showPage('av-pg-new-analysis'));
document.getElementById('avNewAnalysisCancelBtn')?.addEventListener('click', () => showPage('av-pg-welcome'));

document.getElementById('avCreateAnalysisBtn')?.addEventListener('click', async () => {
    const name = document.getElementById('avAnalysisName')?.value?.trim() || '';
    if (!name) {
        document.getElementById('avAnalysisName')?.focus();
        return;
    }
    try {
        const r = await fetch('/analyses', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name })
        });
        const data = await r.json();
        if (!r.ok) { alert('Failed to create analysis: ' + (data.error || r.status)); return; }
        document.getElementById('avAnalysisName').value = '';
        _avCurrentAnalysisId = data.analysisId;
        await refreshSidebar();
        openAddSimForm(data.analysisId, name);
    } catch (e) {
        alert('Could not create analysis: ' + e.message);
    }
});

// ── Add Simulation Form (Step 2) ──────────────────────────────────────────────

function openAddSimForm(analysisId, analysisName) {
    document.getElementById('avAddSimAnalysisName').textContent = analysisName || '';
    document.getElementById('avSimLabel').value = '';

    // Determine default label: if no sessions yet, suggest "Baseline"
    const analysis = _avAnalyses.find(a => a.analysisId === analysisId);
    const hasBaseline = analysis?.sessions?.some(s => s.isBaseline);
    document.getElementById('avSimLabel').placeholder = hasBaseline ? 'e.g. Close Main St' : 'Baseline';

    document.getElementById('avAddSimBtn')._analysisId = analysisId;
    document.getElementById('avAddSimBackLink')._analysisId = analysisId;
    document.querySelector('#av-pg-add-sim')._analysisId = analysisId;

    updateClosedHint();
    showPage('av-pg-add-sim');
}

document.getElementById('avAddSimBackLink')?.addEventListener('click', () => {
    const aid = document.getElementById('avAddSimBackLink')._analysisId;
    if (aid) showComparisonPage(aid);
    else showPage('av-pg-welcome');
});

document.getElementById('avAddSimBtn')?.addEventListener('click', () => {
    const aid = document.getElementById('avAddSimBtn')._analysisId;
    if (!aid) return;
    const analysis = _avAnalyses.find(a => a.analysisId === aid);
    openAddSimForm(aid, analysis?.name || '');
});

document.getElementById('avRunBtn')?.addEventListener('click', async () => {
    const btn = document.getElementById('avRunBtn');
    if (btn?.disabled) return;

    const pg         = document.querySelector('#av-pg-add-sim');
    const analysisId = pg?._analysisId || '';
    if (!analysisId) { alert('No analysis selected.'); return; }

    const analysis   = _avAnalyses.find(a => a.analysisId === analysisId);
    const label      = document.getElementById('avSimLabel')?.value?.trim()
                       || document.getElementById('avSimLabel')?.placeholder || '';
    const runCount     = parseInt(chipVal('avRunCountChips') || '3', 10);
    const simDurationS = parseInt(chipVal('avDurationChips') || '3600', 10);
    const trafficMode  = chipVal('avTrafficChips') || 'medium';

    try {
        const r = await fetch('/analysis/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ analysisId, label, runCount, simDurationS, trafficMode })
        });
        const data = await r.json();
        if (!r.ok) { alert('Failed to start simulation: ' + (data.error || r.status)); return; }

        _avActiveSession = { id: data.sessionId, analysisId, label, total: data.runCount };
        await refreshSidebar();
        startProgressPage(_avActiveSession, analysis?.name || '');
    } catch (e) {
        alert('Could not start simulation — is the database running?\n' + e.message);
    }
});

// ── Progress page ─────────────────────────────────────────────────────────────

function startProgressPage(session, analysisName) {
    showPage('av-pg-progress');
    const titleEl = document.getElementById('avProgTitle');
    const subEl   = document.getElementById('avProgSub');
    const barEl   = document.getElementById('avProgBar');
    const dotsEl  = document.getElementById('avRunDots');

    if (titleEl) titleEl.textContent = (analysisName ? analysisName + ' — ' : '') + (session.label || 'Simulation');
    if (barEl)   barEl.style.width = '0%';

    if (dotsEl) {
        dotsEl.innerHTML = Array.from({ length: session.total }, (_, i) =>
            `<div class="av-run-dot active" id="avDot${i}" title="Run ${i+1}"><span id="avDotTxt${i}">0%</span></div>`
        ).join('');
    }

    if (_avPollInterval) clearInterval(_avPollInterval);
    _avPollInterval = setInterval(async () => {
        try {
            const r = await fetch(`/analysis/${encodeURIComponent(session.id)}/status`);
            if (!r.ok) return;
            const data = await r.json();
            const done    = data.completed ?? 0;
            const total   = data.total ?? session.total;
            const pct     = data.pct ?? 0;
            const runPcts = data.runPcts ?? [];

            if (barEl) barEl.style.width = pct + '%';
            if (subEl) subEl.textContent = `${done} / ${total} runs complete — ${pct}% overall`;

            for (let i = 0; i < total; i++) {
                const dot    = document.getElementById(`avDot${i}`);
                const dotTxt = document.getElementById(`avDotTxt${i}`);
                if (!dot) continue;
                const rp = runPcts[i] ?? 0;
                if (rp >= 100 || i < done) {
                    dot.classList.remove('active'); dot.classList.add('done');
                    if (dotTxt) dotTxt.textContent = '✓';
                } else {
                    dot.classList.add('active'); dot.classList.remove('done');
                    if (dotTxt) dotTxt.textContent = rp + '%';
                }
            }

            if (data.status === 'completed' || done >= total) {
                clearInterval(_avPollInterval);
                _avPollInterval = null;
                await refreshSidebar();
                showComparisonPage(session.analysisId);
            }
        } catch { /* ignore */ }
    }, 1500);
}

// ── Comparison page ───────────────────────────────────────────────────────────

async function showComparisonPage(analysisId) {
    _avCurrentAnalysisId = analysisId;
    const analysis = _avAnalyses.find(a => a.analysisId === analysisId);
    if (!analysis) return;

    showPage('av-pg-compare');

    // Title + meta
    const titleEl = document.getElementById('avCompareTitle');
    if (titleEl) titleEl.textContent = analysis.name;
    const metaEl = document.getElementById('avCompareMeta');
    if (metaEl) metaEl.textContent = analysis.createdAt ? analysis.createdAt.slice(0, 10) : '';

    // Wire "Add Simulation" button
    const addBtn = document.getElementById('avAddSimBtn');
    if (addBtn) {
        addBtn._analysisId = analysisId;
        addBtn.onclick = () => openAddSimForm(analysisId, analysis.name);
    }

    // Fetch fresh session data
    let sessions = [];
    try {
        const r = await fetch(`/analyses/${encodeURIComponent(analysisId)}/sessions`);
        if (r.ok) sessions = await r.json();
    } catch { /* use cached */ }
    if (!sessions.length) sessions = analysis.sessions || [];

    // Build comparison table
    const tbody = document.getElementById('avCompTableBody');
    if (tbody) {
        tbody.innerHTML = sessions.map(s => {
            const rowCls  = s.isBaseline ? 'baseline-row' : s.status === 'failed' ? 'failed-row' : '';
            const danc    = s.avgDancComparable != null ? Number(s.avgDancComparable).toFixed(1) + ' s/ag'
                          : s.avgDancScore     != null ? Number(s.avgDancScore).toFixed(1) + ' s/ag*' : '—';
            const excessM = s.avgExcessTimeSec != null ? (Number(s.avgExcessTimeSec)/60).toFixed(1) + ' min' : '—';
            const rate    = s.avgCompletionRate != null ? (Number(s.avgCompletionRate)*100).toFixed(1) + '%' : '—';
            const fitnessRaw = s.isBaseline ? null : (s.avgFitnessScore != null ? Number(s.avgFitnessScore) : null);
            const fitnessTxt = fitnessRaw != null ? fitnessRaw.toFixed(3) : (s.isBaseline ? 'baseline' : '—');
            const fitnessCls = fitnessRaw == null ? '' : fitnessRaw <= 1.05 ? 'style="color:var(--c-ok)"'
                : fitnessRaw > 1.3 ? 'style="color:var(--c-danger)"' : '';
            const badge    = `<span class="av-running-badge ${s.status}">${s.status}</span>`;
            const nameLink = `<span style="cursor:pointer;text-decoration:underline;text-underline-offset:2px;text-decoration-color:rgba(255,255,255,0.2)"
                data-sid="${escHtml(s.sessionId)}" class="av-comp-sim-link">${escHtml(s.name || 'Simulation')}</span>
                ${s.isBaseline ? '<span class="av-running-badge completed" style="margin-left:4px">BL</span>' : ''}`;
            return `<tr class="${rowCls}">
                <td>${nameLink}</td>
                <td class="num" ${fitnessCls}>${fitnessTxt}</td>
                <td class="num">${danc}</td>
                <td class="num">${rate}</td>
                <td class="num">${excessM}</td>
                <td>${badge}</td>
            </tr>`;
        }).join('');

        // Wire simulation name clicks
        tbody.querySelectorAll('.av-comp-sim-link').forEach(link => {
            link.addEventListener('click', () => {
                const sid = link.dataset.sid;
                const s   = sessions.find(x => x.sessionId === sid);
                if (s) showSimPage(s, analysis);
            });
        });
    }

    // Build bar chart using DANC score (demand-adjusted; accounts for unserved agents)
    const chartEl = document.getElementById('avCompChart');
    if (chartEl) {
        const completed = sessions.filter(s => s.status === 'completed'
            && (s.avgDancComparable != null || s.avgDancScore != null));
        if (!completed.length) {
            chartEl.innerHTML = '<div style="font-size:12px;color:var(--c-text-dim)">No completed simulations yet.</div>';
        } else {
            const maxVal = Math.max(...completed.map(s =>
                Number(s.avgDancComparable ?? s.avgDancScore)));
            chartEl.innerHTML = completed.map(s => {
                const val     = Number(s.avgDancComparable ?? s.avgDancScore);
                const pct     = maxVal > 0 ? (val / maxVal * 100) : 0;
                const fitness = s.avgFitnessScore != null ? Number(s.avgFitnessScore) : null;
                const color   = s.isBaseline ? 'var(--c-ok)'
                    : (fitness != null && fitness <= 1.05) ? 'var(--c-ok)'
                    : (fitness != null && fitness >  1.3)  ? 'var(--c-danger)'
                    : 'var(--c-accent)';
                return `<div class="av-chart-bar-row">
                    <div class="av-chart-bar-label" title="${escHtml(s.name)}">${escHtml(s.name || 'Sim')}</div>
                    <div class="av-chart-bar-bg">
                        <div class="av-chart-bar-fill" style="width:${pct.toFixed(1)}%;background:${color}"></div>
                    </div>
                    <div class="av-chart-bar-val">${val.toFixed(1)} s/ag</div>
                </div>`;
            }).join('');
        }
    }
}

// ── Per-simulation page ───────────────────────────────────────────────────────

async function showSimPage(session, analysis) {
    showPage('av-pg-sim');

    // Breadcrumb
    const analysisName = analysis?.name || '';
    document.getElementById('avSimAnalysisName').textContent = analysisName;
    document.getElementById('avSimLabel2').textContent = session.name || 'Simulation';
    document.getElementById('avSimBackLink').onclick = () => showComparisonPage(analysis?.analysisId || _avCurrentAnalysisId);

    // Title + badge
    document.getElementById('avSimTitle').textContent = session.name || 'Simulation';
    const badgeEl = document.getElementById('avSimBadge');
    if (badgeEl) {
        badgeEl.className = 'av-running-badge ' + (session.isBaseline ? 'completed' : session.status === 'completed' ? 'completed' : session.status);
        badgeEl.textContent = session.isBaseline ? 'Baseline' : session.status === 'completed' ? 'Scenario' : session.status;
    }

    // Meta
    const metaEl = document.getElementById('avSimMeta');
    if (metaEl) {
        const date = session.createdAt ? session.createdAt.slice(0, 16).replace('T', ' ') : '';
        const dur  = session.simDurationS ? (session.simDurationS / 3600).toFixed(1) + ' h' : '—';
        metaEl.textContent = `${date}  ·  ${session.trafficMode ?? ''} traffic  ·  ${session.runCount ?? ''}× runs  ·  ${dur} per run`;
    }

    // Stats grid
    const grid = document.getElementById('avSimStats');
    if (grid) {
        if (session.status === 'running') {
            grid.innerHTML = '<div style="font-size:13px;color:var(--c-warn)">⏳ Simulation still running…</div>';
        } else if (session.status === 'failed') {
            grid.innerHTML = '<div style="font-size:13px;color:var(--c-danger)">This simulation failed.</div>';
        } else {
            const fitness  = (!session.isBaseline && session.avgFitnessScore != null) ? Number(session.avgFitnessScore).toFixed(3) : null;
            const danc     = session.avgDancComparable != null ? Number(session.avgDancComparable).toFixed(1) + ' s/ag'
                           : session.avgDancScore      != null ? Number(session.avgDancScore).toFixed(1) + ' s/ag*' : '—';
            const excessH  = session.avgExcessVehicleHours != null ? Number(session.avgExcessVehicleHours).toFixed(2) + ' vh' : '—';
            const excessM  = session.avgExcessTimeSec != null ? (Number(session.avgExcessTimeSec)/60).toFixed(1) + ' min' : '—';
            const rate     = session.avgCompletionRate != null ? (Number(session.avgCompletionRate)*100).toFixed(1) + '%' : '—';
            const spawned  = session.avgVehiclesSpawned   != null ? Math.round(Number(session.avgVehiclesSpawned))   : null;
            const done     = session.avgVehiclesCompleted != null ? Math.round(Number(session.avgVehiclesCompleted)) : null;
            const unserved = (spawned != null && done != null) ? spawned - done : null;
            const avgFF    = session.avgFreeflowAllSpawnedS != null ? Number(session.avgFreeflowAllSpawnedS).toFixed(0) : null;
            const demandSub = unserved != null
                ? `${done} completed · ${unserved} unserved (spawn-suppressed + stuck)`
                : 'vehicles that reached destination';
            grid.innerHTML = `
                ${fitness != null ? statCard('DANC Fitness', fitness, 'corrected metric — &lt;1.0 better, &gt;1.0 worse than baseline') : ''}
                ${statCard('DANC Score', danc, 'delay + unserved-demand penalty per baseline-demand agent')}
                ${statCard('Completion Rate', rate, demandSub)}
                ${statCard('Avg Delay / Trip', excessM, 'completed trips only — excludes unserved agents')}
                ${statCard('Excess Veh-Hours', excessH, 'completed trips only — not the primary metric')}
                ${spawned != null ? statCard('Vehicles Spawned', String(spawned) + (avgFF ? ' · ff ' + avgFF + 's' : ''), 'avg across runs · avg freeflow time') : ''}
            `;
        }
    }

    if (session.status !== 'completed') {
        document.getElementById('avHistCanvas').getContext('2d')?.clearRect(0, 0, 9999, 9999);
        document.getElementById('avHotspotList').innerHTML = '<div style="padding:20px;font-size:11px;color:var(--c-text-dim)">Complete the simulation to see hotspots.</div>';
        document.getElementById('avMinimapCanvas').getContext('2d')?.clearRect(0, 0, 9999, 9999);
        return;
    }

    // Load trip histogram and hotspots in parallel
    const [tripsData, hotspotsData] = await Promise.all([
        fetch(`/sessions/${encodeURIComponent(session.sessionId)}/trips`).then(r => r.ok ? r.json() : []).catch(() => []),
        fetch(`/sessions/${encodeURIComponent(session.sessionId)}/hotspots`).then(r => r.ok ? r.json() : []).catch(() => [])
    ]);

    drawHistogram(tripsData);
    await drawHotspots(hotspotsData);
}

// ── Travel time histogram ─────────────────────────────────────────────────────

function drawHistogram(trips) {
    const canvas = document.getElementById('avHistCanvas');
    if (!canvas || !trips.length) return;

    const W  = canvas.parentElement.clientWidth - 32; // padding
    const H  = 160;
    canvas.width  = W;
    canvas.height = H;
    const ctx = canvas.getContext('2d');

    const times = trips.map(t => t.actualTravelTimeSec / 60); // → minutes
    const maxT  = Math.min(Math.max(...times), 120); // cap at 120 min
    const BINS  = 30;
    const binW  = maxT / BINS;
    const counts = new Array(BINS).fill(0);
    times.forEach(t => {
        const b = Math.min(Math.floor(t / binW), BINS - 1);
        counts[b]++;
    });
    const maxCount = Math.max(...counts);

    const PAD = { l: 36, r: 12, t: 12, b: 28 };
    const chartW = W - PAD.l - PAD.r;
    const chartH = H - PAD.t - PAD.b;
    const barW   = chartW / BINS - 1;

    ctx.clearRect(0, 0, W, H);

    // Bars
    counts.forEach((c, i) => {
        const x  = PAD.l + i * (chartW / BINS);
        const bH = maxCount > 0 ? (c / maxCount) * chartH : 0;
        const y  = PAD.t + chartH - bH;
        ctx.fillStyle = `rgba(0,200,240,0.55)`;
        ctx.fillRect(x, y, barW, bH);
    });

    // Axes
    ctx.strokeStyle = 'rgba(255,255,255,0.1)';
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.moveTo(PAD.l, PAD.t);
    ctx.lineTo(PAD.l, PAD.t + chartH);
    ctx.lineTo(PAD.l + chartW, PAD.t + chartH);
    ctx.stroke();

    // X labels (every 30 min)
    ctx.fillStyle = 'rgba(255,255,255,0.35)';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    [0, 30, 60, 90, 120].forEach(m => {
        if (m > maxT) return;
        const x = PAD.l + (m / maxT) * chartW;
        ctx.fillText(m + 'm', x, H - 8);
    });

    // Y label
    ctx.save();
    ctx.translate(10, PAD.t + chartH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText('trips', 0, 0);
    ctx.restore();
}

// ── Hotspot minimap ───────────────────────────────────────────────────────────

async function drawHotspots(hotspots) {
    // Populate ranked list
    const listEl = document.getElementById('avHotspotList');
    if (listEl) {
        if (!hotspots.length) {
            listEl.innerHTML = '<div style="padding:20px;font-size:11px;color:var(--c-text-dim)">No intersection conflict data recorded.</div>';
        } else {
            const maxConf = hotspots[0]?.conflictCount || 1;
            listEl.innerHTML = hotspots.slice(0, 15).map((h, i) => {
                const pct = Math.round(h.conflictCount / maxConf * 100);
                return `<div class="av-hotspot-row">
                    <div class="av-hotspot-rank">${i + 1}</div>
                    <div class="av-hotspot-id">#${h.intersectionId}</div>
                    <div class="av-hotspot-bar-bg"><div class="av-hotspot-bar-fill" style="width:${pct}%"></div></div>
                    <div class="av-hotspot-count">${h.conflictCount}</div>
                </div>`;
            }).join('');
        }
    }

    // Minimap canvas
    const canvas = document.getElementById('avMinimapCanvas');
    if (!canvas || !hotspots.length) return;

    // Load intersection positions if not cached
    if (!_avIntersectionPos) {
        try {
            const r = await fetch('/intersections/positions');
            if (r.ok) _avIntersectionPos = await r.json();
        } catch { _avIntersectionPos = []; }
    }
    const positions = _avIntersectionPos || [];
    if (!positions.length) return;

    // Build id→pos map
    const posMap = new Map(positions.map(p => [p.id, p]));
    const hotIds = new Set(hotspots.map(h => h.intersectionId));

    // Compute bounds
    const xs = positions.map(p => p.x), ys = positions.map(p => p.y);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const rangeX = maxX - minX || 1, rangeY = maxY - minY || 1;

    const size = canvas.parentElement.clientWidth || 280;
    canvas.width = size; canvas.height = size;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, size, size);

    const PAD = 16;
    const toX = x => PAD + ((x - minX) / rangeX) * (size - 2 * PAD);
    const toY = y => size - PAD - ((y - minY) / rangeY) * (size - 2 * PAD);

    // Background intersections (dim dots)
    ctx.fillStyle = 'rgba(255,255,255,0.08)';
    for (const p of positions) {
        if (hotIds.has(p.id)) continue;
        ctx.beginPath();
        ctx.arc(toX(p.x), toY(p.y), 1.5, 0, Math.PI * 2);
        ctx.fill();
    }

    // Hotspot circles (sized + colored by severity)
    const maxConf = hotspots[0]?.conflictCount || 1;
    for (const h of hotspots) {
        const pos = posMap.get(h.intersectionId);
        if (!pos) continue;
        const t   = h.conflictCount / maxConf;
        const r   = 3 + t * 12;
        const hue = 60 - t * 60; // green(120) → yellow(60) → red(0)
        ctx.fillStyle = `hsla(${hue},100%,60%,${0.3 + t * 0.5})`;
        ctx.strokeStyle = `hsla(${hue},100%,60%,0.9)`;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(toX(pos.x), toY(pos.y), r, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

function statCard(label, val, unit) {
    return `<div class="av-stat">
        <div class="av-stat-lbl">${escHtml(label)}</div>
        <div class="av-stat-val">${escHtml(String(val))}</div>
        <div class="av-stat-unit">${escHtml(unit)}</div>
    </div>`;
}

function escHtml(s) {
    return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

// ── Run recording ─────────────────────────────────────────────────────────────
const mbStartRun  = document.getElementById('mbStartRun');
const mbFinishRun = document.getElementById('mbFinishRun');
const recIndicator = document.getElementById('recIndicator');

let _activeRunId = '';

function _syncRunState(runId) {
    _activeRunId = runId || '';
    if (recIndicator) {
        recIndicator.style.display = _activeRunId ? '' : 'none';
    }
    if (mbFinishRun) mbFinishRun.classList.toggle('disabled', !_activeRunId);
    if (mbStartRun)  mbStartRun.classList.toggle('disabled',  !!_activeRunId);
}

// Check if a run is already active on the backend (e.g. page reload mid-run)
fetch('/runs/active')
    .then(r => r.ok ? r.json() : null)
    .then(data => { if (data?.active) _syncRunState(data.runId); })
    .catch(() => {});

if (mbStartRun) {
    mbStartRun.addEventListener('click', async () => {
        if (_activeRunId) return;

        // Validate run type
        const VALID_TYPES = ['baseline', 'scenario', 'optimization'];
        let runType = '';
        while (!VALID_TYPES.includes(runType)) {
            runType = (prompt('Run type:\n  baseline — no closures, sets the reference\n  scenario — with closed roads\n  optimization — EA-driven', 'baseline') || '').trim().toLowerCase();
            if (runType === null) return; // cancelled
            if (!VALID_TYPES.includes(runType)) alert('Must be: baseline, scenario, or optimization');
        }
        const label = (prompt('Label (optional):', '') || '').trim();

        try {
            const r = await fetch('/runs/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ runType, label })
            });
            const data = await r.json();
            if (!r.ok) { alert('Failed to start run: ' + (data.error || r.status)); return; }
            _syncRunState(data.runId);
            const closedCount = (data.closedRoadIds || []).length;
            const baselineMsg = data.baselineRunId ? `\nBaseline: ${data.baselineRunId.slice(0,8)}…` : '';
            console.log(`[Analytics] Run started: ${data.runId} (${runType}), ${closedCount} closed roads${baselineMsg}`);
        } catch (e) {
            alert('Could not start run — is the database running?\n' + e.message);
        }
    });
}

if (mbFinishRun) {
    mbFinishRun.addEventListener('click', async () => {
        if (!_activeRunId) return;
        try {
            const r = await fetch(`/runs/${_activeRunId}/finish`, { method: 'POST' });
            const data = await r.json();
            if (!r.ok) { alert('Failed to finish run: ' + (data.error || r.status)); return; }
            console.log('[Analytics] Run finished:', _activeRunId);
            _syncRunState('');
        } catch (e) {
            alert('Could not finish run.\n' + e.message);
        }
    });
}
