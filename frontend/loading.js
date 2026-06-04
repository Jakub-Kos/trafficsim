/*
 * Note: Claude (Anthropic) was used under human supervision to implement this
 * step-by-step map preview as a feature addition for new project setup.
 */
// ── Step definitions ──────────────────────────────────────────────────────────
const STEPS = [
    {
        name: 'Raw OSM Parse',
        desc: 'Every node and segment as read directly from the OSM XML — no filtering, no processing.',
    },
    {
        name: 'Road Classification',
        desc: 'Segments filtered and coloured by road type: motorway, primary, secondary, residential…',
    },
    {
        name: 'Junction Clustering',
        desc: 'Nearby endpoints merged into single junctions (cluster radius ≈ 6 m). Traffic lights and stop signs identified.',
    },
    {
        name: 'Road Chains',
        desc: 'Cleaned road chains — sequences of junction-to-junction segments with forward/backward lane counts.',
    },
    {
        name: 'Lane Geometry',
        desc: 'Driveable lane centre-lines offset laterally from the road spine based on lane width and count.',
    },
    {
        name: 'Intersection Paths',
        desc: 'Bézier-curved routing paths through each intersection connecting incoming to outgoing lanes.',
    },
    {
        name: 'Buildings',
        desc: 'Building footprints from OSM outer/inner polygon rings.',
    },
    {
        name: 'Spawn Regions',
        desc: 'Vehicle entry points auto-placed at dead-end road edges where traffic can enter the map.',
    },
];

const ROAD_COLORS = [
    '#e05050', // 0 motorway
    '#e07840', // 1 trunk
    '#d4b020', // 2 primary
    '#70b840', // 3 secondary
    '#4090c8', // 4 tertiary
    '#708090', // 5 residential / unclassified
];
const ROAD_LABELS = ['Motorway', 'Trunk', 'Primary', 'Secondary', 'Tertiary', 'Residential'];

// ── DOM refs ──────────────────────────────────────────────────────────────────
const canvas      = document.getElementById('mapCanvas');
const ctx         = canvas.getContext('2d');
const loadOverlay = document.getElementById('loadOverlay');
const loadMsg     = document.getElementById('loadMsg');
const errorMsg    = document.getElementById('errorMsg');
const stepNameEl  = document.getElementById('step-name');
const stepDescEl  = document.getElementById('step-desc');
const prevBtn     = document.getElementById('prevBtn');
const nextBtn     = document.getElementById('nextBtn');
const startBtn    = document.getElementById('startBtn');
const autoPlay    = document.getElementById('autoPlay');
const progressBar = document.getElementById('progress-bar');

// ── State ─────────────────────────────────────────────────────────────────────
let mapData     = null;
let osmNodeMap  = {};    // id → {x, y, type}  built from mapData.osmNodes
let currentStep = 0;
let autoTimer   = null;

// ── Transform (world → canvas) ────────────────────────────────────────────────
// transform.x/y = canvas pixel position of world origin (0,0)
// transform.zoom = canvas pixels per world metre
const transform = { x: 0, y: 0, zoom: 1 };

function wx(worldX) { return transform.x + worldX * transform.zoom; }
function wy(worldY) { return transform.y - worldY * transform.zoom; }  // Y-up in world

// ── Canvas resize ─────────────────────────────────────────────────────────────
function syncSize() {
    const r = canvas.getBoundingClientRect();
    const w = Math.round(r.width  * devicePixelRatio);
    const h = Math.round(r.height * devicePixelRatio);
    if (canvas.width !== w || canvas.height !== h) {
        canvas.width  = w;
        canvas.height = h;
        if (mapData) { fitToScreen(); redraw(); }
    }
}
window.addEventListener('resize', syncSize);

// ── Fit all data into the canvas ──────────────────────────────────────────────
function fitToScreen() {
    const pts = [];
    const raw = mapData.rawOsm;
    if (raw && raw.nodes) for (const n of Object.values(raw.nodes)) pts.push(n);
    if (mapData.lanes)    for (const l of mapData.lanes) for (const p of l.points) pts.push(p);

    if (pts.length === 0) { transform.x = canvas.width/2; transform.y = canvas.height/2; transform.zoom = 1; return; }

    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const p of pts) {
        if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
    }

    const W = canvas.width, H = canvas.height;
    const pad = 48 * devicePixelRatio;
    const sx = (W - pad*2) / (maxX - minX || 1);
    const sy = (H - pad*2) / (maxY - minY || 1);
    transform.zoom = Math.min(sx, sy);
    transform.x = W/2 - ((minX + maxX) / 2) * transform.zoom;
    transform.y = H/2 + ((minY + maxY) / 2) * transform.zoom;
}

// ── Pan & zoom ────────────────────────────────────────────────────────────────
let drag = null;   // { startX, startY, tx0, ty0 }

canvas.addEventListener('mousedown', e => {
    drag = { startX: e.clientX, startY: e.clientY, tx0: transform.x, ty0: transform.y };
    canvas.style.cursor = 'grabbing';
});
window.addEventListener('mousemove', e => {
    if (!drag) return;
    const dpr = devicePixelRatio;
    transform.x = drag.tx0 + (e.clientX - drag.startX) * dpr;
    transform.y = drag.ty0 + (e.clientY - drag.startY) * dpr;
    redraw();
});
window.addEventListener('mouseup', () => { drag = null; canvas.style.cursor = 'default'; });

canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const cx = (e.clientX - rect.left) * devicePixelRatio;
    const cy = (e.clientY - rect.top)  * devicePixelRatio;

    // world position under cursor (before zoom)
    const worldX = (cx - transform.x) / transform.zoom;
    const worldY = (transform.y - cy) / transform.zoom;  // Y-flip

    const factor = e.deltaY < 0 ? 1.12 : 1 / 1.12;
    transform.zoom *= factor;

    // keep world position under cursor fixed
    transform.x = cx - worldX * transform.zoom;
    transform.y = cy + worldY * transform.zoom;

    redraw();
}, { passive: false });

// Touch support
let lastTouchDist = null;
let lastTouchMid  = null;
canvas.addEventListener('touchstart', e => {
    if (e.touches.length === 1) {
        const t = e.touches[0];
        drag = { startX: t.clientX, startY: t.clientY, tx0: transform.x, ty0: transform.y };
    } else if (e.touches.length === 2) {
        drag = null;
        const dx = e.touches[0].clientX - e.touches[1].clientX;
        const dy = e.touches[0].clientY - e.touches[1].clientY;
        lastTouchDist = Math.sqrt(dx*dx + dy*dy);
        lastTouchMid = {
            x: (e.touches[0].clientX + e.touches[1].clientX) / 2,
            y: (e.touches[0].clientY + e.touches[1].clientY) / 2,
        };
    }
    e.preventDefault();
}, { passive: false });
canvas.addEventListener('touchmove', e => {
    if (e.touches.length === 1 && drag) {
        const t = e.touches[0];
        const dpr = devicePixelRatio;
        transform.x = drag.tx0 + (t.clientX - drag.startX) * dpr;
        transform.y = drag.ty0 + (t.clientY - drag.startY) * dpr;
        redraw();
    } else if (e.touches.length === 2 && lastTouchDist !== null) {
        const dx = e.touches[0].clientX - e.touches[1].clientX;
        const dy = e.touches[0].clientY - e.touches[1].clientY;
        const dist = Math.sqrt(dx*dx + dy*dy);
        const mid  = {
            x: (e.touches[0].clientX + e.touches[1].clientX) / 2,
            y: (e.touches[0].clientY + e.touches[1].clientY) / 2,
        };
        const rect = canvas.getBoundingClientRect();
        const cx = (mid.x - rect.left) * devicePixelRatio;
        const cy = (mid.y - rect.top)  * devicePixelRatio;
        const worldX = (cx - transform.x) / transform.zoom;
        const worldY = (transform.y - cy) / transform.zoom;
        transform.zoom *= dist / lastTouchDist;
        transform.x = cx - worldX * transform.zoom;
        transform.y = cy + worldY * transform.zoom;
        lastTouchDist = dist;
        lastTouchMid  = mid;
        redraw();
    }
    e.preventDefault();
}, { passive: false });
canvas.addEventListener('touchend', () => { drag = null; lastTouchDist = null; });

// ── Progress bar ──────────────────────────────────────────────────────────────
function buildProgressBar() {
    progressBar.innerHTML = '';
    for (let i = 0; i < STEPS.length; i++) {
        if (i > 0) {
            const conn = document.createElement('div');
            conn.className = 'step-connector';
            progressBar.appendChild(conn);
        }
        const dot = document.createElement('div');
        dot.className = 'step-dot';
        dot.title = STEPS[i].name;
        dot.textContent = i + 1;
        dot.addEventListener('click', () => goToStep(i));
        progressBar.appendChild(dot);
    }
}

function updateProgressBar() {
    progressBar.querySelectorAll('.step-dot').forEach((dot, i) => {
        dot.classList.toggle('done',    i < currentStep);
        dot.classList.toggle('current', i === currentStep);
    });
}

// ── Drawing helpers ───────────────────────────────────────────────────────────
function drawBackground() {
    ctx.fillStyle = '#080c12';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
}

// Step 0 — raw OSM segments (monochrome) + all nodes as dots
function drawRawOsm() {
    const raw = mapData.rawOsm;
    if (!raw) return;

    if (raw.segments && raw.nodes) {
        ctx.strokeStyle = 'rgba(80,110,130,0.5)';
        ctx.lineWidth = 0.8 * devicePixelRatio;
        for (const seg of raw.segments) {
            const a = raw.nodes[seg.from], b = raw.nodes[seg.to];
            if (!a || !b) continue;
            ctx.beginPath();
            ctx.moveTo(wx(a.x), wy(a.y));
            ctx.lineTo(wx(b.x), wy(b.y));
            ctx.stroke();
        }
    }

    if (raw.nodes) {
        ctx.fillStyle = 'rgba(100,150,180,0.45)';
        const r = 1.2 * devicePixelRatio;
        for (const n of Object.values(raw.nodes)) {
            ctx.beginPath();
            ctx.arc(wx(n.x), wy(n.y), r, 0, Math.PI*2);
            ctx.fill();
        }
    }
}

// Step 1 — road classification: recolour segments by type
function drawClassifiedSegments() {
    const raw = mapData.rawOsm;
    if (!raw || !raw.segments || !raw.nodes) return;

    for (const seg of raw.segments) {
        const a = raw.nodes[seg.from], b = raw.nodes[seg.to];
        if (!a || !b) continue;
        const rt = Math.min(seg.roadType ?? 5, ROAD_COLORS.length - 1);
        ctx.strokeStyle = ROAD_COLORS[rt];
        ctx.lineWidth = Math.max(0.8, (5 - rt) * 0.55 + 0.7) * devicePixelRatio;
        ctx.beginPath();
        ctx.moveTo(wx(a.x), wy(a.y));
        ctx.lineTo(wx(b.x), wy(b.y));
        ctx.stroke();
    }
}

// Step 2 — junction nodes after clustering
function drawClusteredNodes() {
    if (!mapData.osmNodes) return;
    for (const n of mapData.osmNodes) {
        const type = n.type ?? 0;
        if (type === 1)      ctx.fillStyle = 'rgba(255,220,40,0.90)';   // traffic light
        else if (type === 2) ctx.fillStyle = 'rgba(255,130,60,0.80)';   // stop/yield
        else                 ctx.fillStyle = 'rgba(80,160,220,0.55)';   // plain junction

        const r = (type > 0 ? 4 : 2.5) * devicePixelRatio;
        ctx.beginPath();
        ctx.arc(wx(n.pos.x), wy(n.pos.y), r, 0, Math.PI*2);
        ctx.fill();
    }
}

// Step 3 — road chains (ways)
function drawRoadChains() {
    if (!mapData.ways || !mapData.osmNodes) return;

    for (const way of mapData.ways) {
        const rt = Math.min(way.type ?? 5, ROAD_COLORS.length - 1);
        ctx.strokeStyle = ROAD_COLORS[rt];
        ctx.lineWidth = Math.max(1, (5 - rt) * 0.5 + 1) * devicePixelRatio;
        ctx.beginPath();
        let first = true;
        for (const nodeId of way.nodes) {
            const n = osmNodeMap[nodeId];
            if (!n) continue;
            if (first) { ctx.moveTo(wx(n.pos.x), wy(n.pos.y)); first = false; }
            else ctx.lineTo(wx(n.pos.x), wy(n.pos.y));
        }
        ctx.stroke();

        // label lane count at midpoint
        if (way.nodes.length >= 2) {
            const mid = Math.floor(way.nodes.length / 2);
            const na = osmNodeMap[way.nodes[mid]];
            if (na) {
                const fwd = way.lanesForward ?? 1;
                const bwd = way.lanesBackward ?? 1;
                ctx.fillStyle = 'rgba(200,230,255,0.65)';
                ctx.font = `${9 * devicePixelRatio}px monospace`;
                ctx.fillText(`${fwd}↑${bwd}↓`, wx(na.pos.x) + 3*devicePixelRatio, wy(na.pos.y) - 3*devicePixelRatio);
            }
        }
    }
}

// Step 4 — computed lane centre-lines
function drawLanes() {
    if (!mapData.lanes) return;
    ctx.strokeStyle = 'rgba(0,210,255,0.70)';
    ctx.lineWidth = 1.1 * devicePixelRatio;
    for (const lane of mapData.lanes) {
        if (lane.points.length < 2) continue;
        ctx.beginPath();
        ctx.moveTo(wx(lane.points[0].x), wy(lane.points[0].y));
        for (let i = 1; i < lane.points.length; i++) {
            ctx.lineTo(wx(lane.points[i].x), wy(lane.points[i].y));
        }
        ctx.stroke();
    }
}

// Step 5 — intersection routing paths + intersection dots
function drawIntersectionPaths() {
    if (!mapData.intersection_paths) return;
    ctx.strokeStyle = 'rgba(255,180,60,0.50)';
    ctx.lineWidth   = 1.0 * devicePixelRatio;
    ctx.setLineDash([3*devicePixelRatio, 3*devicePixelRatio]);
    for (const path of mapData.intersection_paths) {
        if (path.points.length < 2) continue;
        ctx.beginPath();
        ctx.moveTo(wx(path.points[0].x), wy(path.points[0].y));
        for (let i = 1; i < path.points.length; i++) {
            ctx.lineTo(wx(path.points[i].x), wy(path.points[i].y));
        }
        ctx.stroke();
    }
    ctx.setLineDash([]);
}

function drawIntersections() {
    if (!mapData.intersections) return;
    for (const inter of mapData.intersections) {
        const tl = inter.type === 1;
        ctx.fillStyle   = tl ? 'rgba(255,220,40,0.85)' : 'rgba(255,130,60,0.80)';
        ctx.strokeStyle = tl ? 'rgba(255,220,40,0.30)' : 'rgba(255,130,60,0.30)';
        ctx.lineWidth   = 1 * devicePixelRatio;
        const r = 5 * devicePixelRatio;
        ctx.beginPath(); ctx.arc(wx(inter.x), wy(inter.y), r, 0, Math.PI*2); ctx.fill();
        ctx.beginPath(); ctx.arc(wx(inter.x), wy(inter.y), r+3*devicePixelRatio, 0, Math.PI*2); ctx.stroke();
    }
}

// Step 6 — buildings
function drawBuildings() {
    if (!mapData.buildings) return;
    ctx.fillStyle   = 'rgba(70,80,110,0.40)';
    ctx.strokeStyle = 'rgba(110,130,190,0.38)';
    ctx.lineWidth   = 0.7 * devicePixelRatio;
    for (const b of mapData.buildings) {
        if (!b.outer) continue;
        for (const ring of b.outer) {
            if (ring.length < 2) continue;
            ctx.beginPath();
            ctx.moveTo(wx(ring[0].x), wy(ring[0].y));
            for (let i = 1; i < ring.length; i++) ctx.lineTo(wx(ring[i].x), wy(ring[i].y));
            ctx.closePath(); ctx.fill(); ctx.stroke();
        }
    }
}

// Step 7 — spawn regions
function drawSpawnRegions() {
    if (!mapData.spawn_regions) return;
    ctx.fillStyle   = 'rgba(0,230,120,0.16)';
    ctx.strokeStyle = 'rgba(0,230,120,0.65)';
    ctx.lineWidth   = 1.5 * devicePixelRatio;
    for (const r of mapData.spawn_regions) {
        const x1 = wx(r.min.x), y1 = wy(r.min.y);
        const x2 = wx(r.max.x), y2 = wy(r.max.y);
        const lx = Math.min(x1,x2), ty = Math.min(y1,y2);
        const w  = Math.abs(x2-x1),  h  = Math.abs(y2-y1);
        ctx.fillRect(lx,ty,w,h); ctx.strokeRect(lx,ty,w,h);
    }
}

// ── Legends ───────────────────────────────────────────────────────────────────
function drawRoadLegend() {
    const pad = 12 * devicePixelRatio;
    let y = 18 * devicePixelRatio;
    ctx.font = `bold ${9*devicePixelRatio}px system-ui`;
    ctx.fillStyle = 'rgba(140,170,195,0.60)';
    ctx.fillText('ROAD TYPE', pad, y); y += 11*devicePixelRatio;
    for (let i = 0; i < ROAD_LABELS.length; i++) {
        ctx.fillStyle = ROAD_COLORS[i];
        ctx.fillRect(pad, y, 12*devicePixelRatio, 3*devicePixelRatio);
        ctx.fillStyle = 'rgba(200,220,240,0.75)';
        ctx.font = `${9*devicePixelRatio}px system-ui`;
        ctx.fillText(ROAD_LABELS[i], pad + 16*devicePixelRatio, y + 3*devicePixelRatio);
        y += 13*devicePixelRatio;
    }
}

function drawJunctionLegend() {
    const pad = 12 * devicePixelRatio;
    let y = 18 * devicePixelRatio;
    ctx.font = `bold ${9*devicePixelRatio}px system-ui`;
    ctx.fillStyle = 'rgba(140,170,195,0.60)';
    ctx.fillText('JUNCTION TYPE', pad, y); y += 14*devicePixelRatio;
    const items = [
        ['rgba(255,220,40,0.90)', 'Traffic Light'],
        ['rgba(255,130,60,0.80)', 'Stop / Yield'],
        ['rgba(80,160,220,0.55)',  'Plain Junction'],
    ];
    for (const [col, label] of items) {
        ctx.fillStyle = col;
        ctx.beginPath(); ctx.arc(pad+5*devicePixelRatio, y, 4*devicePixelRatio, 0, Math.PI*2); ctx.fill();
        ctx.fillStyle = 'rgba(200,220,240,0.75)';
        ctx.font = `${9*devicePixelRatio}px system-ui`;
        ctx.fillText(label, pad+14*devicePixelRatio, y+3*devicePixelRatio);
        y += 13*devicePixelRatio;
    }
}

function drawIntersectionLegend() {
    const pad = 12 * devicePixelRatio;
    let y = 18 * devicePixelRatio;
    ctx.font = `bold ${9*devicePixelRatio}px system-ui`;
    ctx.fillStyle = 'rgba(140,170,195,0.60)';
    ctx.fillText('INTERSECTION', pad, y); y += 14*devicePixelRatio;
    const items = [
        ['rgba(255,220,40,0.85)', 'Traffic Light'],
        ['rgba(255,130,60,0.80)', 'Uncontrolled'],
    ];
    for (const [col, label] of items) {
        ctx.fillStyle = col;
        ctx.beginPath(); ctx.arc(pad+5*devicePixelRatio, y, 4*devicePixelRatio, 0, Math.PI*2); ctx.fill();
        ctx.fillStyle = 'rgba(200,220,240,0.75)';
        ctx.font = `${9*devicePixelRatio}px system-ui`;
        ctx.fillText(label, pad+14*devicePixelRatio, y+3*devicePixelRatio);
        y += 13*devicePixelRatio;
    }
}

// ── Hint ──────────────────────────────────────────────────────────────────────
function drawPanHint() {
    const txt = 'Drag to pan  ·  Scroll to zoom';
    ctx.font = `${9*devicePixelRatio}px system-ui`;
    const tw = ctx.measureText(txt).width;
    const x = canvas.width - tw - 10*devicePixelRatio;
    const y = canvas.height - 8*devicePixelRatio;
    ctx.fillStyle = 'rgba(60,90,110,0.70)';
    ctx.fillText(txt, x, y);
}

// ── Main redraw ───────────────────────────────────────────────────────────────
function redraw() {
    if (!mapData) return;
    drawBackground();
    const s = currentStep;

    if (s >= 0) drawRawOsm();
    if (s >= 1) drawClassifiedSegments();
    if (s >= 2) drawClusteredNodes();
    if (s >= 3) drawRoadChains();
    if (s >= 4) drawLanes();
    if (s >= 5) { drawIntersectionPaths(); drawIntersections(); }
    if (s >= 6) drawBuildings();
    if (s >= 7) drawSpawnRegions();

    // Only one legend at a time — whichever is most relevant to the current step
    if      (s === 1 || s === 3) drawRoadLegend();
    else if (s === 2)            drawJunctionLegend();
    else if (s === 5)            drawIntersectionLegend();

    drawPanHint();
}

// ── Step navigation ───────────────────────────────────────────────────────────
function goToStep(idx) {
    idx = Math.max(0, Math.min(STEPS.length - 1, idx));
    currentStep = idx;
    stepNameEl.textContent = `Step ${idx + 1} / ${STEPS.length}  —  ${STEPS[idx].name}`;
    stepDescEl.textContent = STEPS[idx].desc;

    prevBtn.disabled = idx === 0;
    nextBtn.disabled = idx === STEPS.length - 1;
    nextBtn.textContent = idx === STEPS.length - 2 ? 'Finish ✓' : 'Next →';
    const lastStep = idx === STEPS.length - 1;
    startBtn.classList.toggle('visible', lastStep);
    const editBtn = document.getElementById('editBtn');
    if (editBtn) editBtn.classList.toggle('visible', lastStep);

    updateProgressBar();
    redraw();
}

prevBtn.addEventListener('click', () => { clearAutoTimer(); goToStep(currentStep - 1); });
nextBtn.addEventListener('click', () => { clearAutoTimer(); goToStep(currentStep + 1); });

const _projectCtx = sessionStorage.getItem('currentProject') || '';
startBtn.addEventListener('click', () => { window.location.href = '/app'; });

// ── Auto-play ─────────────────────────────────────────────────────────────────
function clearAutoTimer() { if (autoTimer) { clearInterval(autoTimer); autoTimer = null; } }

autoPlay.addEventListener('change', () => {
    clearAutoTimer();
    if (autoPlay.checked) {
        autoTimer = setInterval(() => {
            if (currentStep < STEPS.length - 1) goToStep(currentStep + 1);
            else { clearAutoTimer(); autoPlay.checked = false; }
        }, 1800);
    }
});

// ── Init ──────────────────────────────────────────────────────────────────────
async function init() {
    loadMsg.textContent = 'Fetching map data…';

    let elapsed = 0;
    const timer = setInterval(() => {
        elapsed++;
        loadMsg.textContent = `Fetching map data…  ${elapsed}s`;
    }, 1000);

    try {
        const resp = await fetch('/map');
        if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
        mapData = await resp.json();
    } catch (err) {
        clearInterval(timer);
        errorMsg.textContent = `Failed to load map: ${err.message}`;
        errorMsg.classList.add('visible');
        loadMsg.textContent = '';
        return;
    }

    clearInterval(timer);

    if (mapData.osmNodes) {
        for (const n of mapData.osmNodes) osmNodeMap[n.id] = n;
    }

    syncSize();
    fitToScreen();
    loadOverlay.classList.add('hidden');
    buildProgressBar();
    goToStep(0);
}

init();
