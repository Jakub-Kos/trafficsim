/*
 * Note: The WebSocket protocol design and API layer architecture are the
 * author's own work. Claude (Anthropic) was used under human supervision to
 * add new endpoint functions as features were incrementally introduced.
 */

// Module-level send handle — set when the WS connection opens.
// All command functions write through this so callers need no ws reference.
let _send = null;

function wsSend(cmd) {
    if (_send) {
        _send(cmd);
    } else {
        console.warn('[net] WS not ready, dropping command:', cmd.cmd);
    }
}

// ── Queries (HTTP — request/response semantics) ──────────────────────────────

export async function fetchMap() {
    const r = await fetch('/map');
    if (!r.ok) throw new Error(`GET /map failed: ${r.status}`);
    return r.json();
}

export async function fetchVehicleDetail(id) {
    const r = await fetch(`/vehicle/detail?id=${id}`);
    if (!r.ok) return null;
    return r.json();
}

export async function fetchHeatmap(minX, minY, maxX, maxY, resX = 20, resY = 20) {
    const params = new URLSearchParams({ minX, minY, maxX, maxY, resX, resY });
    const r = await fetch(`/heatmap?${params}`);
    if (!r.ok) return null;
    return r.json();
}

export async function fetchLaneStats() {
    const r = await fetch('/heatmap/lanes');
    if (!r.ok) return null;
    return r.json();
}

export async function fetchParkingSpots() {
    const r = await fetch('/parking/spots');
    if (!r.ok) return null;
    return r.json();
}

// ── Live stream (WebSocket) ───────────────────────────────────────────────────

export function connectLive(onStates) {
    const wsHost = window.location.hostname;
    const wsUrl = `ws://${wsHost}:9002/`;
    const ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        console.log('WS connected', wsUrl);
        _send = (cmd) => ws.send(JSON.stringify(cmd));
    };
    ws.onmessage = (ev) => {
        try { onStates(JSON.parse(ev.data)); } catch (e) { console.error('Bad WS JSON:', e); }
    };
    ws.onerror = (e) => console.error('WS error:', e);
    ws.onclose = () => {
        console.log('WS closed');
        _send = null;
    };

    return ws;
}

// ── Commands (WebSocket — fire and forget) ────────────────────────────────────

export function setSimSpeed(speedVal) {
    wsSend({ cmd: 'setSpeed', value: speedVal });
}

export function setVehicleRoute(vehicleId, routeId) {
    wsSend({ cmd: 'setVehicleRoute', vehicleId, routeId });
}

export function setRoadClosed(wayId, isClosed) {
    wsSend({ cmd: 'setRoadClosed', wayId, isClosed });
}

export function sendViewport(minX, minY, maxX, maxY) {
    wsSend({ cmd: 'setViewport', minX, minY, maxX, maxY });
}

// Tell the server which streaming mode to use for this client.
// 'agents' = vehicle positions, 'density' = KDE grid, 'congestion' = lane stats
export function setStreamMode(mode) {
    wsSend({ cmd: 'setStreamMode', mode });
}

// ── Project API ───────────────────────────────────────────────────────────────

export async function listProjects() {
    const r = await fetch('/projects');
    if (!r.ok) throw new Error(`GET /projects failed: ${r.status}`);
    return r.json();
}

export async function createProject(name, description = '') {
    const r = await fetch('/projects', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name, description }),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
    return j;
}

export async function saveProjectOsm(name, osmXml) {
    const r = await fetch(`/projects/${encodeURIComponent(name)}/osm`, {
        method: 'POST',
        headers: { 'Content-Type': 'text/xml' },
        body: osmXml,
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
    return j;
}

export async function loadProject(name) {
    const r = await fetch(`/projects/${encodeURIComponent(name)}/load`, {
        method: 'POST',
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
    return j;
}

export async function deleteProject(name) {
    const r = await fetch(`/projects/${encodeURIComponent(name)}`, {
        method: 'DELETE',
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
    return j;
}

export async function updateProject(oldName, newName, description) {
    const r = await fetch(`/projects/${encodeURIComponent(oldName)}`, {
        method: 'PATCH',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name: newName, description }),
    });
    const j = await r.json();
    if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
    return j;
}