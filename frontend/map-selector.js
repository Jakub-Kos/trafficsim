/*
 * Note: Claude (Anthropic) was used under human supervision to implement this
 * map selection UI as a feature addition for new project creation.
 */
// ── Constants ─────────────────────────────────────────────────────────────────
const TILE_SIZE    = 256;          // OSM slippy-map tile pixels

// Grid steps: fixed at page load for the initial center so the whole grid is aligned.
// LAT_STEP ≈ 500 m; LNG_STEP compensates for Mercator so cells look square.
const INIT_LAT     = 50.08;
const LAT_STEP     = 0.0045;
const LNG_STEP     = LAT_STEP / Math.cos(INIT_LAT * Math.PI / 180);  // ≈ 0.007°

// ── Map view state ────────────────────────────────────────────────────────────
let centerLat = INIT_LAT;
let centerLng = 14.43;
let zoom = 14;

// ── Canvas ────────────────────────────────────────────────────────────────────
const canvas = document.getElementById('mapCanvas');
const ctx    = canvas.getContext('2d');

function syncCanvasSize() {
    const r = canvas.getBoundingClientRect();
    const w = Math.round(r.width);
    const h = Math.round(r.height);
    if (canvas.width !== w || canvas.height !== h) {
        canvas.width  = w;
        canvas.height = h;
    }
}
// Size it immediately so coordinate math is correct before the first frame
syncCanvasSize();

// ── Web-Mercator helpers ──────────────────────────────────────────────────────

function latLngToTileF(lat, lng, z) {
    // Returns floating-point tile coordinates
    const n   = 1 << z;
    const tx  = (lng + 180) / 360 * n;
    const sin = Math.sin(lat * Math.PI / 180);
    const ty  = (1 - Math.log((1 + sin) / (1 - sin)) / (2 * Math.PI)) / 2 * n;
    return [tx, ty];
}

function tileToLatLng(tx, ty, z) {
    const n   = 1 << z;
    const lng = tx / n * 360 - 180;
    const lat = Math.atan(Math.sinh(Math.PI * (1 - 2 * ty / n))) * 180 / Math.PI;
    return [lat, lng];
}

function screenToLatLng(sx, sy) {
    const [cx, cy] = latLngToTileF(centerLat, centerLng, zoom);
    return tileToLatLng(
        cx + (sx - canvas.width  / 2) / TILE_SIZE,
        cy + (sy - canvas.height / 2) / TILE_SIZE,
        zoom
    );
}

function latLngToScreen(lat, lng) {
    const [cx, cy] = latLngToTileF(centerLat, centerLng, zoom);
    const [tx, ty] = latLngToTileF(lat, lng, zoom);
    return [
        (tx - cx) * TILE_SIZE + canvas.width  / 2,
        (ty - cy) * TILE_SIZE + canvas.height / 2,
    ];
}

// ── OSM tile image cache ──────────────────────────────────────────────────────
const imgCache = new Map();

function getOsmTile(z, x, y) {
    const maxT = 1 << z;
    x = ((x % maxT) + maxT) % maxT;
    if (y < 0 || y >= maxT) return null;
    const key = `${z}/${x}/${y}`;
    if (!imgCache.has(key)) {
        const img = new Image();
        img.crossOrigin = 'anonymous';
        img.src = `https://${'abc'[(x + y) % 3]}.tile.openstreetmap.org/${z}/${x}/${y}.png`;
        img.onload = () => { dirty = true; };
        imgCache.set(key, img);
    }
    return imgCache.get(key);
}

// ── Render loop ───────────────────────────────────────────────────────────────
let dirty = true;

function drawCell(slat, slng, fillStyle, strokeStyle) {
    // NW corner (top-left on screen): higher lat, same lng
    // SE corner (bottom-right on screen): lower lat, higher lng
    const [x0, y0] = latLngToScreen(slat + LAT_STEP, slng);
    const [x1, y1] = latLngToScreen(slat, slng + LNG_STEP);
    ctx.fillStyle   = fillStyle;
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
    ctx.strokeStyle = strokeStyle;
    ctx.lineWidth   = 1.5;
    ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
}

function frame() {
    requestAnimationFrame(frame);   // always schedule next — avoids double-scheduling bug
    syncCanvasSize();
    if (!dirty) return;
    dirty = false;

    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    // ── 1. OSM tile images ────────────────────────────────────────────────────
    const [cx, cy] = latLngToTileF(centerLat, centerLng, zoom);
    const tx0 = Math.floor(cx - W / 2 / TILE_SIZE);
    const ty0 = Math.floor(cy - H / 2 / TILE_SIZE);
    const tx1 = Math.ceil( cx + W / 2 / TILE_SIZE);
    const ty1 = Math.ceil( cy + H / 2 / TILE_SIZE);

    for (let ty = ty0; ty <= ty1; ty++) {
        for (let tx = tx0; tx <= tx1; tx++) {
            const sx = (tx - cx) * TILE_SIZE + W / 2;
            const sy = (ty - cy) * TILE_SIZE + H / 2;
            const img = getOsmTile(zoom, tx, ty);
            if (img && img.complete && img.naturalWidth > 0) {
                ctx.drawImage(img, sx, sy, TILE_SIZE, TILE_SIZE);
            } else {
                ctx.fillStyle = '#2a2a30';
                ctx.fillRect(sx, sy, TILE_SIZE, TILE_SIZE);
                ctx.strokeStyle = '#3a3a44';
                ctx.lineWidth = 0.5;
                ctx.strokeRect(sx, sy, TILE_SIZE, TILE_SIZE);
            }
        }
    }

    // ── 2. Selected cells (coloured by download state) ───────────────────────
    for (const [key, tile] of tiles) {
        const [slatStr, slngStr] = key.split(':');
        const { fill, stroke } = TILE_STYLE[tile.state] || TILE_STYLE.selected;
        drawCell(parseFloat(slatStr), parseFloat(slngStr), fill, stroke);
    }

    // ── 3. Box-select preview ─────────────────────────────────────────────────
    if (isBoxSelecting && boxStartLat !== null && boxEndLat !== null) {
        const minLat = Math.min(boxStartLat, boxEndLat);
        const maxLat = Math.max(boxStartLat, boxEndLat);
        const minLng = Math.min(boxStartLng, boxEndLng);
        const maxLng = Math.max(boxStartLng, boxEndLng);
        const [x0, y0] = latLngToScreen(maxLat, minLng);
        const [x1, y1] = latLngToScreen(minLat, maxLng);
        ctx.fillStyle   = 'rgba(0,200,240,0.12)';
        ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
        ctx.strokeStyle = 'rgba(0,210,250,0.85)';
        ctx.lineWidth   = 1.5;
        ctx.setLineDash([4, 3]);
        ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
        ctx.setLineDash([]);
    }

    // ── 4. Hover highlight ────────────────────────────────────────────────────
    if (hoverLat !== null && !isDragging && !isBoxSelecting) {
        const slat = Math.floor(hoverLat / LAT_STEP) * LAT_STEP;
        const slng = Math.floor(hoverLng / LNG_STEP) * LNG_STEP;
        if (!tiles.has(tileKey(slat, slng)) || tiles.get(tileKey(slat, slng))?.state === 'selected') {
            drawCell(slat, slng, 'rgba(0,200,240,0.10)', 'rgba(0,210,250,0.65)');
        }
    }

    // ── 5. Zoom label ─────────────────────────────────────────────────────────
    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.fillRect(8, H - 26, 62, 18);
    ctx.fillStyle = '#999';
    ctx.font = '11px system-ui';
    ctx.textBaseline = 'middle';
    ctx.fillText(`zoom ${zoom}`, 14, H - 17);
}

requestAnimationFrame(frame);
window.addEventListener('resize', () => { dirty = true; });

// ── Pan interaction ───────────────────────────────────────────────────────────
let isDragging    = false;
let didDrag       = false;
let dragStartX    = 0, dragStartY    = 0;
let dragCenterTX  = 0, dragCenterTY  = 0;

// ── Box-select interaction ────────────────────────────────────────────────────
let isBoxSelecting = false;
let boxStartLat = null, boxStartLng = null;
let boxEndLat   = null, boxEndLng   = null;

canvas.addEventListener('mousedown', e => {
    if (e.button !== 0) return;
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    if (e.shiftKey) {
        // Shift+drag: box select
        isBoxSelecting = true;
        [boxStartLat, boxStartLng] = screenToLatLng(sx, sy);
        boxEndLat = boxStartLat;
        boxEndLng = boxStartLng;
        canvas.style.cursor = 'crosshair';
    } else {
        // Plain drag: pan
        isDragging = true;
        didDrag    = false;
        dragStartX = e.clientX;
        dragStartY = e.clientY;
        [dragCenterTX, dragCenterTY] = latLngToTileF(centerLat, centerLng, zoom);
        canvas.style.cursor = 'grabbing';
    }
});

window.addEventListener('mousemove', e => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    if (isBoxSelecting) {
        [boxEndLat, boxEndLng] = screenToLatLng(sx, sy);
        dirty = true;
        return;
    }

    if (isDragging) {
        const dx = dragStartX - e.clientX;
        const dy = dragStartY - e.clientY;
        if (Math.abs(dx) > 4 || Math.abs(dy) > 4) didDrag = true;
        const [lat, lng] = tileToLatLng(
            dragCenterTX + dx / TILE_SIZE,
            dragCenterTY + dy / TILE_SIZE,
            zoom
        );
        centerLat = lat;
        centerLng = lng;
        dirty = true;
    }

    // Hover
    if (sx >= 0 && sx <= rect.width && sy >= 0 && sy <= rect.height) {
        [hoverLat, hoverLng] = screenToLatLng(sx, sy);
        dirty = true;
    } else {
        hoverLat = null; hoverLng = null;
    }
});

window.addEventListener('mouseup', e => {
    if (isBoxSelecting) {
        isBoxSelecting = false;
        canvas.style.cursor = 'crosshair';

        // Add all tiles within the selected rectangle
        if (boxStartLat !== null && boxEndLat !== null) {
            const minLat = Math.floor(Math.min(boxStartLat, boxEndLat) / LAT_STEP) * LAT_STEP;
            const maxLat = Math.floor(Math.max(boxStartLat, boxEndLat) / LAT_STEP) * LAT_STEP;
            const minLng = Math.floor(Math.min(boxStartLng, boxEndLng) / LNG_STEP) * LNG_STEP;
            const maxLng = Math.floor(Math.max(boxStartLng, boxEndLng) / LNG_STEP) * LNG_STEP;

            for (let lat = minLat; lat <= maxLat + LAT_STEP * 0.5; lat = +(lat + LAT_STEP).toFixed(7)) {
                for (let lng = minLng; lng <= maxLng + LNG_STEP * 0.5; lng = +(lng + LNG_STEP).toFixed(7)) {
                    if (!tiles.has(tileKey(lat, lng))) tiles.set(tileKey(lat, lng), { state: 'selected' });
                }
            }
            renderSidebar();
            dirty = true;
        }
        boxStartLat = boxEndLat = boxStartLng = boxEndLng = null;
        return;
    }

    if (!isDragging) return;
    isDragging = false;
    canvas.style.cursor = 'crosshair';

    if (!didDrag) {
        // Click: toggle the cell under the cursor
        const rect = canvas.getBoundingClientRect();
        const sx = e.clientX - rect.left;
        const sy = e.clientY - rect.top;
        if (sx >= 0 && sx <= rect.width && sy >= 0 && sy <= rect.height) {
            const [lat, lng] = screenToLatLng(sx, sy);
            const slat = Math.floor(lat / LAT_STEP) * LAT_STEP;
            const slng = Math.floor(lng / LNG_STEP) * LNG_STEP;
            const key = tileKey(slat, slng);
            if (tiles.has(key)) tiles.delete(key); else tiles.set(key, { state: 'selected' });
            renderSidebar();
            dirty = true;
        }
    }
});

canvas.addEventListener('mouseleave', () => { hoverLat = null; hoverLng = null; dirty = true; });

// ── Zoom ──────────────────────────────────────────────────────────────────────
canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    // World coords under cursor — should stay fixed after zoom
    const [lat, lng] = screenToLatLng(sx, sy);

    zoom = Math.max(10, Math.min(18, zoom + (e.deltaY < 0 ? 1 : -1)));

    // Recompute center so the cursor point stays put
    const [tx, ty] = latLngToTileF(lat, lng, zoom);
    const [newLat, newLng] = tileToLatLng(
        tx - (sx - canvas.width  / 2) / TILE_SIZE,
        ty - (sy - canvas.height / 2) / TILE_SIZE,
        zoom
    );
    centerLat = newLat;
    centerLng = newLng;
    dirty = true;
}, { passive: false });

document.getElementById('zoomIn').addEventListener('click',  () => { zoom = Math.min(18, zoom + 1); dirty = true; });
document.getElementById('zoomOut').addEventListener('click', () => { zoom = Math.max(10, zoom - 1); dirty = true; });

// ── Hover state ───────────────────────────────────────────────────────────────
let hoverLat = null;
let hoverLng = null;

// ── Tile store ────────────────────────────────────────────────────────────────
// Map of "lat:lng" → { state: 'selected'|'downloading'|'ready'|'error' }
const tiles = new Map();

const TILE_STYLE = {
    selected:    { fill: 'rgba(40,210,130,0.22)',  stroke: '#2d8' },
    downloading: { fill: 'rgba(255,190,40,0.35)',  stroke: '#fb3' },
    ready:       { fill: 'rgba(0,200,255,0.30)',   stroke: '#0cf' },
    error:       { fill: 'rgba(255,60,60,0.28)',   stroke: '#f44' },
};

function tileKey(slat, slng) {
    return `${slat.toFixed(7)}:${slng.toFixed(7)}`;
}

// ── Sidebar ───────────────────────────────────────────────────────────────────

const tileListEl = document.getElementById('tileList');
const statsEl    = document.getElementById('stats');
const statusEl   = document.getElementById('statusMsg');
const applyBtn   = document.getElementById('applyBtn');

function setStatus(msg, cls = '') {
    statusEl.textContent = msg;
    statusEl.className = cls;
}

const DOT_STATE = { selected: 'queued', downloading: 'downloading', ready: 'ready', error: 'error' };

function renderSidebar() {
    if (tiles.size === 0) {
        tileListEl.innerHTML = '<div class="tile-empty">No tiles selected yet.</div>';
        statsEl.textContent  = '';
        applyBtn.disabled    = true;
        loadBtn.style.display = 'none';
        return;
    }
    tileListEl.innerHTML = '';
    for (const [key, tile] of tiles) {
        const [slatStr, slngStr] = key.split(':');
        const entry = document.createElement('div');
        entry.className = 'tile-entry';
        const dot = document.createElement('div');
        dot.className = `tile-dot ${DOT_STATE[tile.state] || 'queued'}`;
        const coords = document.createElement('div');
        coords.className = 'tile-coords';
        coords.textContent = `${parseFloat(slatStr).toFixed(4)}, ${parseFloat(slngStr).toFixed(4)}`;
        const rm = document.createElement('div');
        rm.className = 'tile-remove'; rm.textContent = '×'; rm.title = 'Remove';
        rm.addEventListener('click', () => { tiles.delete(key); downloadedXml = null; loadBtn.style.display = 'none'; renderSidebar(); dirty = true; });
        entry.append(dot, coords, rm);
        tileListEl.appendChild(entry);
    }
    const areaDeg2 = tiles.size * LAT_STEP * LNG_STEP;
    const areaKm2  = (areaDeg2 * 111 * 111 * Math.cos(INIT_LAT * Math.PI / 180)).toFixed(1);
    statsEl.textContent = `${tiles.size} tile${tiles.size !== 1 ? 's' : ''} · ~${areaKm2} km²`;
    applyBtn.disabled = false;
}

// ── Download & Apply ──────────────────────────────────────────────────────────
// Splits tiles into chunks of BATCH_SIZE, runs MAX_PARALLEL chunks concurrently,
// then merges results. Avoids both serial slowness and single-giant-query 504s.

// Use the main OSM API — same source as openstreetmap.org/export, direct DB access,
// much faster than Overpass for small areas with no query compilation or rate-limit waits.
// Each tile is ~0.00003 deg², nowhere near the 0.25 deg² / 50k-node limit.
const BATCH_SIZE    = 1;   // one tile per request keeps each well under OSM API limits
const MAX_PARALLEL  = 2;   // OSM API usage policy: be polite, don't spike requests
const REQUEST_GAP   = 500; // ms between each tile start to avoid bandwidth bursts

async function downloadChunk(keys) {
    // Mark tiles as downloading
    for (const key of keys) { const t = tiles.get(key); if (t) t.state = 'downloading'; }
    dirty = true; renderSidebar();

    let minLat = Infinity, maxLat = -Infinity, minLng = Infinity, maxLng = -Infinity;
    for (const key of keys) {
        const [slatStr, slngStr] = key.split(':');
        const s = parseFloat(slatStr), w = parseFloat(slngStr);
        minLat = Math.min(minLat, s);      maxLat = Math.max(maxLat, s + LAT_STEP);
        minLng = Math.min(minLng, w);      maxLng = Math.max(maxLng, w + LNG_STEP);
    }
    // bbox = left,bottom,right,top  (lng,lat order for OSM API)
    const url = `https://api.openstreetmap.org/api/0.6/map?bbox=${minLng.toFixed(7)},${minLat.toFixed(7)},${maxLng.toFixed(7)},${maxLat.toFixed(7)}`;

    for (let attempt = 0; attempt < 3; attempt++) {
        if (attempt > 0) await new Promise(r => setTimeout(r, 3000));
        const resp = await fetch(url);
        if (resp.status === 429 || resp.status === 509) {
            await new Promise(r => setTimeout(r, 5000)); // back off on rate limit
            continue;
        }
        if (resp.status === 400) {
            const txt = await resp.text();
            for (const key of keys) { const t = tiles.get(key); if (t) t.state = 'error'; }
            dirty = true; renderSidebar();
            throw new Error(txt.includes('nodes') ? 'tile has too many nodes — select a smaller area' : `HTTP 400`);
        }
        if (!resp.ok) {
            for (const key of keys) { const t = tiles.get(key); if (t) t.state = 'error'; }
            dirty = true; renderSidebar();
            throw new Error(`HTTP ${resp.status}`);
        }
        const xml = await resp.text();
        if (!xml.includes('<osm')) {
            for (const key of keys) { const t = tiles.get(key); if (t) t.state = 'error'; }
            dirty = true; renderSidebar();
            throw new Error('unexpected response format');
        }
        // Success
        for (const key of keys) { const t = tiles.get(key); if (t) t.state = 'ready'; }
        dirty = true; renderSidebar();
        return xml;
    }
    for (const key of keys) { const t = tiles.get(key); if (t) t.state = 'error'; }
    dirty = true; renderSidebar();
    throw new Error('rate limited — try again shortly');
}

function mergeOSM(xmlStrings) {
    const nodesSeen = new Set(), waysSeen = new Set(), relsSeen = new Set();
    const parser = new DOMParser();
    let nodes = '', ways = '', rels = '';
    for (const xml of xmlStrings) {
        const doc = parser.parseFromString(xml, 'text/xml');
        for (const el of doc.querySelectorAll('node')) {
            const id = el.getAttribute('id');
            if (!nodesSeen.has(id)) { nodesSeen.add(id); nodes += el.outerHTML + '\n'; }
        }
        for (const el of doc.querySelectorAll('way')) {
            const id = el.getAttribute('id');
            if (!waysSeen.has(id)) { waysSeen.add(id); ways += el.outerHTML + '\n'; }
        }
        for (const el of doc.querySelectorAll('relation')) {
            const id = el.getAttribute('id');
            if (!relsSeen.has(id)) { relsSeen.add(id); rels += el.outerHTML + '\n'; }
        }
    }
    return `<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6">\n${nodes}${ways}${rels}</osm>`;
}

let downloadedXml = null;
const loadBtn = document.getElementById('loadBtn');

applyBtn.addEventListener('click', async () => {
    if (tiles.size === 0) return;
    applyBtn.disabled = true;
    loadBtn.style.display = 'none';
    downloadedXml = null;

    // Reset any previously-downloaded tiles back to selected so colours refresh
    for (const tile of tiles.values()) tile.state = 'selected';
    dirty = true; renderSidebar();

    const tileArray = [...tiles.keys()];
    const chunks = [];
    for (let i = 0; i < tileArray.length; i += BATCH_SIZE)
        chunks.push(tileArray.slice(i, i + BATCH_SIZE));

    setStatus(`Downloading… 0 / ${chunks.length} tile${chunks.length !== 1 ? 's' : ''}`);

    const xmlResults = new Array(chunks.length).fill(null);
    let completed = 0;
    let firstError = null;

    // Stagger worker starts so all workers don't fire simultaneously
    const queue = chunks.map((chunk, i) => ({ chunk, i }));
    await Promise.all(Array.from({ length: Math.min(MAX_PARALLEL, chunks.length) }, async (_, workerIdx) => {
        await new Promise(r => setTimeout(r, workerIdx * REQUEST_GAP));
        while (queue.length > 0) {
            const { chunk, i } = queue.shift();
            try {
                xmlResults[i] = await downloadChunk(chunk);
                completed++;
                setStatus(`Downloading… ${completed} / ${chunks.length} tile${chunks.length !== 1 ? 's' : ''}`);
            } catch (err) {
                if (!firstError) firstError = err;
            }
            if (queue.length > 0) await new Promise(r => setTimeout(r, REQUEST_GAP));
        }
    }));

    applyBtn.disabled = false;

    if (firstError) {
        setStatus(`Download failed: ${firstError.message}`, 'err');
        return;
    }

    downloadedXml = xmlResults.length === 1 ? xmlResults[0] : mergeOSM(xmlResults.filter(Boolean));
    setStatus('All tiles ready.', 'ok');
    loadBtn.style.display = '';
});

loadBtn.addEventListener('click', async () => {
    if (!downloadedXml) return;
    loadBtn.disabled = true;
    applyBtn.disabled = true;

    const urlProject = new URLSearchParams(location.search).get('project')
                    || sessionStorage.getItem('currentProject');
    try {
        setStatus('Loading map into simulation…');
        const resp = await fetch('/map/reload', {
            method: 'POST',
            headers: { 'Content-Type': 'text/xml' },
            body: downloadedXml,
        });
        const json = await resp.json();
        if (!resp.ok) throw new Error(json.error || `HTTP ${resp.status}`);

        if (urlProject) {
            setStatus('Saving map to project…');
            await fetch(`/projects/${encodeURIComponent(urlProject)}/osm`, {
                method: 'POST',
                headers: { 'Content-Type': 'text/xml' },
                body: downloadedXml,
            });
            sessionStorage.setItem('currentProject', urlProject);
        }

        setStatus('Map loaded! Redirecting…', 'ok');
        const dest = urlProject
            ? '/app'
            : (localStorage.getItem('stepByStepMode') === '1' ? '/loading.html' : '/sim');
        setTimeout(() => { window.location.href = dest; }, 1200);
    } catch (err) {
        setStatus(`Load failed: ${err.message}`, 'err');
        loadBtn.disabled = false;
        applyBtn.disabled = false;
    }
});

document.getElementById('clearBtn').addEventListener('click', () => {
    tiles.clear(); downloadedXml = null;
    loadBtn.style.display = 'none';
    renderSidebar(); setStatus(''); dirty = true;
});

renderSidebar();

// ── Location search (Nominatim) ───────────────────────────────────────────────
const searchInput   = document.getElementById('searchInput');
const searchSpinner = document.getElementById('searchSpinner');
const searchResults = document.getElementById('searchResults');

let searchTimer = null;

searchInput.addEventListener('input', () => {
    clearTimeout(searchTimer);
    const q = searchInput.value.trim();
    if (q.length < 3) { searchResults.style.display = 'none'; return; }
    searchTimer = setTimeout(() => runSearch(q), 400);
});

searchInput.addEventListener('keydown', e => {
    if (e.key === 'Escape') { searchResults.style.display = 'none'; searchInput.blur(); return; }
    if (e.key === 'Enter') {
        e.preventDefault();
        const q = searchInput.value.trim();
        if (!q) return;
        // If dropdown already has results, pick the first one
        const first = searchResults.querySelector('.search-result[data-lat]');
        if (first) { first.click(); return; }
        // Otherwise fire a search immediately
        clearTimeout(searchTimer);
        runSearch(q);
    }
});

document.addEventListener('click', e => {
    if (!e.target.closest('#searchWrap')) searchResults.style.display = 'none';
});

async function runSearch(q) {
    searchSpinner.style.display = '';
    searchResults.style.display = 'none';
    try {
        const url = `https://nominatim.openstreetmap.org/search?q=${encodeURIComponent(q)}&format=json&limit=6&addressdetails=0`;
        const resp = await fetch(url, { headers: { 'Accept-Language': 'en' } });
        const results = await resp.json();
        showResults(results);
    } catch {
        // silently ignore search errors
    } finally {
        searchSpinner.style.display = 'none';
    }
}

function showResults(results) {
    // Position dropdown flush under the input using fixed coords
    const rect = searchInput.getBoundingClientRect();
    searchResults.style.left  = rect.left + 'px';
    searchResults.style.top   = rect.bottom + 4 + 'px';
    searchResults.style.width = rect.width + 'px';

    searchResults.innerHTML = '';
    if (!results.length) {
        const el = document.createElement('div');
        el.className = 'search-result';
        el.style.color = 'var(--c-text-dim)';
        el.textContent = 'No results found';
        searchResults.appendChild(el);
    } else {
        for (const r of results) {
            const el = document.createElement('div');
            el.className = 'search-result';
            el.dataset.lat = r.lat;
            el.dataset.lon = r.lon;
            el.textContent = r.display_name;
            el.title = r.display_name;
            el.addEventListener('click', () => {
                centerLat = parseFloat(r.lat);
                centerLng = parseFloat(r.lon);
                zoom = 15;
                dirty = true;
                searchResults.style.display = 'none';
                searchInput.value = r.display_name.split(',')[0].trim();
            });
            searchResults.appendChild(el);
        }
    }
    searchResults.style.display = '';
}
