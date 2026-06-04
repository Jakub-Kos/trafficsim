/*
 * Note: The WebGL 2.0 rendering pipeline and all GLSL shader programs in this
 * file are the author's own work. Claude (Anthropic) was used under human
 * supervision to add rendering features on top of this foundation — specifically
 * the density heatmap overlay and per-lane congestion colouring modes.
 */
// frontend/renderer.js — WebGL 2.0 hardware-accelerated renderer

// ── Shader sources ────────────────────────────────────────────────────────────

const FLAT_VERT = `#version 300 es
precision highp float;
uniform mat3 u_view;
in vec2 a_pos;
void main() {
  vec3 p = u_view * vec3(a_pos, 1.0);
  gl_Position = vec4(p.xy, 0.0, 1.0);
}`;

const FLAT_FRAG = `#version 300 es
precision mediump float;
uniform vec4 u_color;
out vec4 outColor;
void main() { outColor = u_color; }`;

// Per-vertex: a_corner (unit quad). Per-instance: pos, heading, r, g, b, accel.
const VEHICLE_VERT = `#version 300 es
precision highp float;
uniform mat3 u_view;
in vec2  a_corner;    // [-0.5..0.5]²
in vec2  a_pos;       // world position
in float a_heading;   // orientation (radians, CCW from +X)
in float a_r;
in float a_g;
in float a_b;
in float a_accel;
out vec2  v_uv;
out vec3  v_col;
flat out float v_braking;
void main() {
  v_uv     = a_corner + 0.5;
  v_col    = vec3(a_r, a_g, a_b);
  v_braking = step(a_accel, -1.0);
  float c = cos(a_heading), s = sin(a_heading);
  vec2 local = vec2(a_corner.x * 4.5, a_corner.y * 2.0);
  vec2 rot   = vec2(local.x*c - local.y*s, local.x*s + local.y*c);
  vec3 p     = u_view * vec3(rot + a_pos, 1.0);
  gl_Position = vec4(p.xy, 0.0, 1.0);
}`;

const VEHICLE_FRAG = `#version 300 es
precision mediump float;
uniform float u_alpha;
in vec2  v_uv;
in vec3  v_col;
flat in float v_braking;
out vec4 outColor;
void main() {
  // Rounded-rect signed-distance discard
  vec2 d = abs(v_uv - 0.5) - vec2(0.41, 0.43);
  float dist = length(max(d, 0.0)) - 0.07;
  if (dist > 0.0) discard;

  // Brake lights: rear corners (x < 0.18, y near edges)
  float rear  = 1.0 - step(0.18, v_uv.x);
  float edge  = max(step(0.62, v_uv.y), 1.0 - step(0.38, v_uv.y));
  float brake = rear * edge * v_braking;

  // Windshield: front centre strip
  float wind = step(0.60, v_uv.x) * step(0.18, v_uv.y) * (1.0 - step(0.82, v_uv.y));

  vec3 col = mix(v_col,       vec3(1.0, 0.04, 0.04), brake);
  col       = mix(col,        col * 0.10,             wind);
  outColor  = vec4(col, u_alpha);
}`;

// ── Geometry helpers ──────────────────────────────────────────────────────────

// Builds a seamless triangle strip for a polyline using miter joints at corners.
// No gaps or overlaps at bends.
function polylineQuads(points, halfWidth, out) {
    const n = points.length;
    if (n < 2) return;

    // Per-segment left normals (perpendicular to direction, pointing left)
    const snx = [], sny = [];
    for (let i = 0; i < n - 1; i++) {
        const dx = points[i+1].x - points[i].x;
        const dy = points[i+1].y - points[i].y;
        const len = Math.hypot(dx, dy);
        if (len < 1e-9) { snx.push(snx[i-1] ?? 0); sny.push(sny[i-1] ?? 1); continue; }
        snx.push(-dy / len);
        sny.push( dx / len);
    }

    // Per-vertex left/right offset points via miter joints
    const lx = [], ly = [], rx = [], ry = [];
    for (let i = 0; i < n; i++) {
        let mnx, mny, scale;
        if (i === 0) {
            mnx = snx[0]; mny = sny[0]; scale = halfWidth;
        } else if (i === n - 1) {
            mnx = snx[n-2]; mny = sny[n-2]; scale = halfWidth;
        } else {
            // Bisector of adjacent normals
            const bx = snx[i-1] + snx[i];
            const by = sny[i-1] + sny[i];
            const blen = Math.hypot(bx, by);
            if (blen < 1e-9) {
                mnx = snx[i-1]; mny = sny[i-1]; scale = halfWidth;
            } else {
                mnx = bx / blen; mny = by / blen;
                const dot = mnx * snx[i-1] + mny * sny[i-1];
                // scale = hw/sin(half-angle); clamp to avoid degenerate spikes
                scale = dot > 0.2 ? halfWidth / dot : halfWidth * 5.0;
            }
        }
        lx.push(points[i].x + mnx * scale);
        ly.push(points[i].y + mny * scale);
        rx.push(points[i].x - mnx * scale);
        ry.push(points[i].y - mny * scale);
    }

    // Emit quads as two triangles each
    for (let i = 0; i < n - 1; i++) {
        out.push(
            lx[i],   ly[i],   rx[i],   ry[i],   lx[i+1], ly[i+1],
            lx[i+1], ly[i+1], rx[i],   ry[i],   rx[i+1], ry[i+1],
        );
    }
}

// Ear-clipping polygon triangulation — handles convex AND non-convex OSM buildings.
function earclipTriangulate(ring, out) {
    // OSM closed rings repeat the first node at the end — strip it.
    let pts = ring;
    while (pts.length > 1) {
        const f = pts[0], l = pts[pts.length - 1];
        if (Math.abs(f.x - l.x) < 1e-6 && Math.abs(f.y - l.y) < 1e-6)
            pts = pts.slice(0, -1);
        else break;
    }
    // Also remove any consecutive duplicate vertices that would make degenerate triangles.
    const deduped = [pts[0]];
    for (let i = 1; i < pts.length; i++) {
        const prev = deduped[deduped.length - 1], cur = pts[i];
        if (Math.abs(cur.x - prev.x) > 1e-6 || Math.abs(cur.y - prev.y) > 1e-6)
            deduped.push(cur);
    }
    pts = deduped;

    const n = pts.length;
    if (n < 3) return;
    if (n === 3) {
        out.push(pts[0].x, pts[0].y, pts[1].x, pts[1].y, pts[2].x, pts[2].y);
        return;
    }

    // Ensure CCW winding (cross-product sum > 0)
    let area2 = 0;
    for (let i = 0; i < n; i++) {
        const j = (i + 1) % n;
        area2 += pts[i].x * pts[j].y - pts[j].x * pts[i].y;
    }
    const verts = area2 >= 0 ? [...pts] : [...pts].reverse();

    const idx = Array.from({ length: verts.length }, (_, i) => i);

    function cross(ax, ay, bx, by, cx, cy) {
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    }
    function inTriangle(px, py, ax, ay, bx, by, cx, cy) {
        return cross(ax,ay, bx,by, px,py) >= 0 &&
               cross(bx,by, cx,cy, px,py) >= 0 &&
               cross(cx,cy, ax,ay, px,py) >= 0;
    }

    let rem = idx.length;
    let safety = rem * rem + 8;
    let i = 0;

    while (rem > 3 && safety-- > 0) {
        const ii   = i % rem;
        i = ii; // keep i bounded
        const pi   = idx[(ii - 1 + rem) % rem];
        const ci   = idx[ii];
        const ni   = idx[(ii + 1) % rem];
        const a = verts[pi], b = verts[ci], c = verts[ni];

        // Convex vertex check
        if (cross(a.x,a.y, b.x,b.y, c.x,c.y) > 0) {
            // No reflex vertex inside the ear triangle
            let ear = true;
            for (let k = 0; k < rem && ear; k++) {
                const ki = idx[k];
                if (ki === pi || ki === ci || ki === ni) continue;
                const p = verts[ki];
                if (inTriangle(p.x,p.y, a.x,a.y, b.x,b.y, c.x,c.y)) ear = false;
            }
            if (ear) {
                out.push(a.x, a.y, b.x, b.y, c.x, c.y);
                idx.splice(ii, 1);
                rem--;
                continue; // re-check same position
            }
        }
        i++;
    }
    if (rem === 3) {
        const a = verts[idx[0]], b = verts[idx[1]], c = verts[idx[2]];
        out.push(a.x, a.y, b.x, b.y, c.x, c.y);
    }
}

function rectOutlineQuads(minX, minY, maxX, maxY, halfW, out) {
    const corners = [
        { x: minX, y: minY }, { x: maxX, y: minY },
        { x: maxX, y: maxY }, { x: minX, y: maxY },
        { x: minX, y: minY },
    ];
    polylineQuads(corners, halfW, out);
}

// ── WebGL boilerplate ─────────────────────────────────────────────────────────

function compileShader(gl, type, src) {
    const s = gl.createShader(type);
    gl.shaderSource(s, src);
    gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS))
        throw new Error('Shader compile error:\n' + gl.getShaderInfoLog(s));
    return s;
}

function createProgram(gl, vertSrc, fragSrc) {
    const prog = gl.createProgram();
    gl.attachShader(prog, compileShader(gl, gl.VERTEX_SHADER,   vertSrc));
    gl.attachShader(prog, compileShader(gl, gl.FRAGMENT_SHADER, fragSrc));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS))
        throw new Error('Program link error:\n' + gl.getProgramInfoLog(prog));
    return prog;
}

function uploadStaticBuffer(gl, data) {
    const buf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(data), gl.STATIC_DRAW);
    return buf;
}

// ── Color helpers ─────────────────────────────────────────────────────────────

function lerpRGB(c1, c2, t) {
    return [
        c1[0] + (c2[0] - c1[0]) * t,
        c1[1] + (c2[1] - c1[1]) * t,
        c1[2] + (c2[2] - c1[2]) * t,
    ];
}

const COL_CRUISE = [0.33, 0.40, 0.53];
const COL_BRAKE  = [1.00, 0.27, 0.27];
const COL_ACCEL  = [0.31, 1.00, 0.60];
const COL_SELECT = [1.00, 1.00, 1.00];
const COL_PARKED      = [0.50, 0.50, 0.50]; // gray for parked vehicles
const COL_DEPART_SOON = [0.20, 0.80, 0.30]; // green for vehicles departing soon
const DEPART_WINDOW   = 1800;               // 30 min in seconds

function parkedColor(timeUntilDeparture) {
    if (timeUntilDeparture < 0) return COL_PARKED; // unknown / not set
    const t = Math.max(0, Math.min(1, 1 - timeUntilDeparture / DEPART_WINDOW));
    return lerpRGB(COL_PARKED, COL_DEPART_SOON, t);
}

function vehicleColor(accel) {
    const t = Math.min(Math.abs(accel) / 2.5, 1.0);
    return accel < 0 ? lerpRGB(COL_CRUISE, COL_BRAKE, t)
                     : lerpRGB(COL_CRUISE, COL_ACCEL, t);
}

// ── Main renderer class ───────────────────────────────────────────────────────

const MAX_VEHICLES = 16384; // matches agentPoolCapacity in simulation.json
// Per-instance floats: x, y, heading, r, g, b, accel  = 7 floats = 28 bytes
const INST_FLOATS  = 7;

export class WebGLRenderer {
    constructor(canvas) {
        this.canvas = canvas;
        const gl = canvas.getContext('webgl2', { antialias: true, alpha: true });
        if (!gl) throw new Error('WebGL 2 not supported');
        this.gl = gl;

        // Programs
        this.flatProg    = createProgram(gl, FLAT_VERT, FLAT_FRAG);
        this.vehicleProg = createProgram(gl, VEHICLE_VERT, VEHICLE_FRAG);

        // Flat shader locations
        this.flatViewLoc  = gl.getUniformLocation(this.flatProg, 'u_view');
        this.flatColorLoc = gl.getUniformLocation(this.flatProg, 'u_color');
        this.flatPosLoc   = gl.getAttribLocation(this.flatProg, 'a_pos');

        // Vehicle shader locations
        this.vehViewLoc    = gl.getUniformLocation(this.vehicleProg, 'u_view');
        this.vehAlphaLoc   = gl.getUniformLocation(this.vehicleProg, 'u_alpha');
        this.vehCornerLoc  = gl.getAttribLocation(this.vehicleProg, 'a_corner');
        this.vehPosLoc     = gl.getAttribLocation(this.vehicleProg, 'a_pos');
        this.vehHeadLoc    = gl.getAttribLocation(this.vehicleProg, 'a_heading');
        this.vehRLoc       = gl.getAttribLocation(this.vehicleProg, 'a_r');
        this.vehGLoc       = gl.getAttribLocation(this.vehicleProg, 'a_g');
        this.vehBLoc       = gl.getAttribLocation(this.vehicleProg, 'a_b');
        this.vehAccelLoc   = gl.getAttribLocation(this.vehicleProg, 'a_accel');

        // Vehicle base quad (6 verts, 2 triangles)
        this._vehicleQuadBuf = uploadStaticBuffer(gl, [
            -0.5, -0.5,  -0.5,  0.5,   0.5,  0.5,
            -0.5, -0.5,   0.5,  0.5,   0.5, -0.5,
        ]);

        // Dynamic instance buffer (preallocated)
        this._instanceBuf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, this._instanceBuf);
        gl.bufferData(gl.ARRAY_BUFFER, MAX_VEHICLES * INST_FLOATS * 4, gl.DYNAMIC_DRAW);
        this._instanceData = new Float32Array(MAX_VEHICLES * INST_FLOATS);

        // Static geometry buffers (populated by loadMap)
        this._geom = null;

        // Closed ways set
        this._closedWays = new Set();
        this._closedLaneVerts = 0;
        this._closedBuf = null;

        // Lane metadata for road-click detection
        this._lanes = [];

        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    }

    loadMap(mapData) {
        const gl = this.gl;

        // Build wayId → layer map (layer < 0 = tunnel, 0 = normal, > 0 = bridge)
        const wayLayerMap = new Map();
        for (const way of (mapData.ways || [])) {
            if (way.layer) wayLayerMap.set(way.id, way.layer);
        }
        const laneLayer = (lane) => wayLayerMap.get(Math.floor(lane.id / 1000)) ?? 0;

        // --- Road surface geometry split by layer ---
        // surfT/markT = tunnel (layer < 0)   → rendered faint/dark
        // surfN/markN = normal (layer == 0)  → rendered standard
        // surfB/markB = bridge (layer > 0)   → rendered on top with casing
        // casingB     = wider version of bridge surface for the elevated-edge effect
        const surfT = [], markT = [];
        const surfN = [], markN = [];
        const surfB = [], markB = [], casingB = [];

        for (const lane of (mapData.lanes || [])) {
            const pts = lane.points || lane.centerline || [];
            const ly = laneLayer(lane);
            if (ly < 0) {
                polylineQuads(pts, 3.5, surfT);
                polylineQuads(pts, 0.15, markT);
            } else if (ly > 0) {
                polylineQuads(pts, 4.3, casingB);  // slightly wider → creates edge rim
                polylineQuads(pts, 3.5, surfB);
                polylineQuads(pts, 0.15, markB);
            } else {
                polylineQuads(pts, 3.5, surfN);
                polylineQuads(pts, 0.15, markN);
            }
        }

        // Intersection paths are always at ground level
        const ipaths = mapData.intersection_paths || mapData.intersectionPaths || [];
        for (const path of ipaths) {
            polylineQuads(path.points || [], 3.5, surfN);
            polylineQuads(path.points || [], 0.15, markN);
        }

        // --- Buildings ---
        const bldgVerts = [];
        for (const b of (mapData.buildings || [])) {
            for (const ring of (b.outer || [])) earclipTriangulate(ring, bldgVerts);
        }

        // --- Spawn regions (orange outline) ---
        const spawnVerts = [];
        for (const r of (mapData.spawn_regions || [])) {
            rectOutlineQuads(r.min.x, r.min.y, r.max.x, r.max.y, 0.5, spawnVerts);
        }

        const mkBuf = (v) => ({ buf: uploadStaticBuffer(gl, v), count: v.length / 2 });
        this._geom = {
            surfT: mkBuf(surfT), markT: mkBuf(markT),
            surfN: mkBuf(surfN), markN: mkBuf(markN),
            surfB: mkBuf(surfB), markB: mkBuf(markB), casingB: mkBuf(casingB),
            bldg:  mkBuf(bldgVerts),
            spawn: mkBuf(spawnVerts),
        };

        // Store lane info for road-click detection, closure overlay, and layer lookup.
        // Backend encodes: laneId = wayId * 1000 + 100/200 + laneIndex
        this._lanes = (mapData.lanes || []).map(l => ({
            wayId: Math.floor(l.id / 1000),
            layer: laneLayer(l),
            points: l.points || [],
        }));

        // Expose layer map for vehicle rendering in main.js
        this._wayLayerMap = wayLayerMap;

        this._buildClosedOverlay();
    }

    setClosedWays(closedSet) {
        this._closedWays = closedSet;
        this._buildClosedOverlay();
    }

    _buildClosedOverlay() {
        if (!this._lanes.length) return;
        const gl = this.gl;
        const verts = [];
        for (const lane of this._lanes) {
            if (this._closedWays.has(lane.wayId)) {
                polylineQuads(lane.points, 3.8, verts);
            }
        }
        this._closedLaneVerts = verts.length / 2;
        if (!this._closedBuf) this._closedBuf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, this._closedBuf);
        if (verts.length > 0) {
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(verts), gl.DYNAMIC_DRAW);
        }
    }

    // Finds the wayId of the lane closest to world point (wx, wy)
    laneAtWorldPos(wx, wy, thresholdMeters = 5) {
        let bestDist = thresholdMeters, bestWayId = null;
        for (const lane of this._lanes) {
            const pts = lane.points;
            for (let i = 0; i < pts.length - 1; i++) {
                const ax = pts[i].x, ay = pts[i].y;
                const bx = pts[i+1].x, by = pts[i+1].y;
                const dx = bx - ax, dy = by - ay;
                const len2 = dx*dx + dy*dy;
                if (len2 < 1e-9) continue;
                const t = Math.max(0, Math.min(1, ((wx-ax)*dx + (wy-ay)*dy) / len2));
                const px = ax + t*dx, py = ay + t*dy;
                const d = Math.hypot(wx - px, wy - py);
                if (d < bestDist) { bestDist = d; bestWayId = lane.wayId; }
            }
        }
        return bestWayId;
    }

    // Draws one VAO-less buffer with the flat shader (already bound)
    _drawFlat(buf, count, color) {
        const gl = this.gl;
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.enableVertexAttribArray(this.flatPosLoc);
        gl.vertexAttribPointer(this.flatPosLoc, 2, gl.FLOAT, false, 0, 0);
        gl.uniform4fv(this.flatColorLoc, color);
        gl.drawArrays(gl.TRIANGLES, 0, count);
    }

    render(viewMatrix, vehicles, selectedId, sceneMode = false) {
        const gl = this.gl;
        const dpr = window.devicePixelRatio || 1;
        gl.viewport(0, 0, this.canvas.width, this.canvas.height);
        if (sceneMode) {
            gl.clearColor(0, 0, 0, 0); // transparent — background canvas shows through
        } else {
            gl.clearColor(0.031, 0.047, 0.071, 1.0); // #080c12
        }
        gl.clear(gl.COLOR_BUFFER_BIT);

        if (!this._geom) return;

        // ── Static geometry (flat shader) ──────────────────────────────────
        if (!sceneMode) {
            gl.useProgram(this.flatProg);
            gl.uniformMatrix3fv(this.flatViewLoc, false, viewMatrix);

            // Buildings
            this._drawFlat(this._geom.bldg.buf, this._geom.bldg.count,
                [0.149, 0.157, 0.188, 0.85]);

            // Tunnel roads (ghost effect — faded dark)
            this._drawFlat(this._geom.surfT.buf, this._geom.surfT.count,
                [0.07, 0.08, 0.10, 0.40]);
            this._drawFlat(this._geom.markT.buf, this._geom.markT.count,
                [0.20, 0.22, 0.25, 0.15]);

            // Normal roads
            this._drawFlat(this._geom.surfN.buf, this._geom.surfN.count,
                [0.235, 0.247, 0.275, 1.0]);
            this._drawFlat(this._geom.markN.buf, this._geom.markN.count,
                [0.55, 0.60, 0.65, 0.35]);

            // Bridge casing (slightly wider, lighter rim rendered first)
            this._drawFlat(this._geom.casingB.buf, this._geom.casingB.count,
                [0.32, 0.35, 0.40, 1.0]);
            // Bridge surface on top
            this._drawFlat(this._geom.surfB.buf, this._geom.surfB.count,
                [0.235, 0.247, 0.275, 1.0]);
            this._drawFlat(this._geom.markB.buf, this._geom.markB.count,
                [0.55, 0.60, 0.65, 0.35]);

            // Spawn regions (orange)
            this._drawFlat(this._geom.spawn.buf, this._geom.spawn.count,
                [1.0, 0.73, 0.18, 0.55]);
        }

        // Closed road overlay (red-orange)
        if (this._closedLaneVerts > 0) {
            this._drawFlat(this._closedBuf, this._closedLaneVerts,
                [1.0, 0.22, 0.14, 0.72]);
        }

        // ── Vehicles (instanced) ───────────────────────────────────────────
        if (vehicles.length === 0) return;

        let count = 0;
        const inst = this._instanceData;
        for (const v of vehicles) {
            if (count >= MAX_VEHICLES) break;
            const base = count * INST_FLOATS;
            const col = v.id === selectedId ? COL_SELECT
                      : v.parked            ? parkedColor(v.timeUntilDeparture ?? -1)
                      : vehicleColor(v.a ?? 0);
            inst[base + 0] = v.x;
            inst[base + 1] = v.y;
            inst[base + 2] = v.h ?? 0;
            inst[base + 3] = col[0];
            inst[base + 4] = col[1];
            inst[base + 5] = col[2];
            inst[base + 6] = v.a ?? 0;
            count++;
        }

        gl.bindBuffer(gl.ARRAY_BUFFER, this._instanceBuf);
        gl.bufferSubData(gl.ARRAY_BUFFER, 0, inst.subarray(0, count * INST_FLOATS));

        gl.useProgram(this.vehicleProg);
        gl.uniformMatrix3fv(this.vehViewLoc, false, viewMatrix);

        const S = INST_FLOATS * 4; // stride in bytes

        // Base quad (per-vertex, divisor 0)
        gl.bindBuffer(gl.ARRAY_BUFFER, this._vehicleQuadBuf);
        gl.enableVertexAttribArray(this.vehCornerLoc);
        gl.vertexAttribPointer(this.vehCornerLoc, 2, gl.FLOAT, false, 0, 0);
        gl.vertexAttribDivisor(this.vehCornerLoc, 0);

        // Per-instance attributes (divisor 1)
        gl.bindBuffer(gl.ARRAY_BUFFER, this._instanceBuf);

        const setInst = (loc, size, offset) => {
            gl.enableVertexAttribArray(loc);
            gl.vertexAttribPointer(loc, size, gl.FLOAT, false, S, offset * 4);
            gl.vertexAttribDivisor(loc, 1);
        };

        setInst(this.vehPosLoc,   2, 0);  // x, y
        setInst(this.vehHeadLoc,  1, 2);  // heading
        setInst(this.vehRLoc,     1, 3);  // r
        setInst(this.vehGLoc,     1, 4);  // g
        setInst(this.vehBLoc,     1, 5);  // b
        setInst(this.vehAccelLoc, 1, 6);  // accel

        gl.uniform1f(this.vehAlphaLoc, 1.0);
        gl.drawArraysInstanced(gl.TRIANGLES, 0, 6, count);

        // Reset divisors (avoid contaminating future draws)
        for (const loc of [this.vehCornerLoc, this.vehPosLoc, this.vehHeadLoc,
                            this.vehRLoc, this.vehGLoc, this.vehBLoc, this.vehAccelLoc]) {
            gl.vertexAttribDivisor(loc, 0);
            gl.disableVertexAttribArray(loc);
        }
        gl.disableVertexAttribArray(this.flatPosLoc);
    }
}

// ── 2D canvas debug overlay functions (unchanged API) ────────────────────────
// These draw onto a separate Canvas 2D context laid over the WebGL canvas.

export function drawDebugLayers(ctx, worldToScreen, mapData, vp, dynamicIntersections, opts, spawnData, simTime) {
    if (!opts.master) return;
    if (opts.lights)      drawTrafficLights(ctx, worldToScreen, mapData, vp, dynamicIntersections);
    if (opts.conflict)    drawConflictPoints(ctx, worldToScreen, mapData, vp, dynamicIntersections);
    if (opts.rawosm)      drawRawOSM(ctx, worldToScreen, mapData, vp);
    if (opts.spawnrates)  drawSpawnRateLabels(ctx, worldToScreen, spawnData, simTime, vp);
    if (opts.ids)         {} // drawn per-vehicle in drawVehicleIds
}

// Cluster spawn regions whose centres are within CLUSTER_RADIUS world-metres of each other,
// aggregate their spawn counts, then draw a label per cluster.
const SPAWN_CLUSTER_RADIUS = 50.0;

function drawSpawnRateLabels(ctx, worldToScreen, spawnData, simTime, vp) {
    if (!spawnData?.regions?.length) return;
    const simHours = simTime / 3600;

    // Greedy spatial clustering
    const regions = spawnData.regions;
    const assigned = new Uint8Array(regions.length);
    const clusters = [];

    for (let i = 0; i < regions.length; i++) {
        if (assigned[i]) continue;
        const cluster = { x: 0, y: 0, spawnCount: 0, totalRate: 0, n: 0 };
        for (let j = i; j < regions.length; j++) {
            if (assigned[j]) continue;
            const dx = regions[j].x - regions[i].x;
            const dy = regions[j].y - regions[i].y;
            if (Math.sqrt(dx*dx + dy*dy) <= SPAWN_CLUSTER_RADIUS) {
                cluster.x          += regions[j].x;
                cluster.y          += regions[j].y;
                cluster.spawnCount += regions[j].spawnCount;
                cluster.totalRate  += regions[j].totalRate;
                cluster.n++;
                assigned[j] = 1;
            }
        }
        cluster.x /= cluster.n;
        cluster.y /= cluster.n;
        clusters.push(cluster);
    }

    ctx.save();
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';

    for (const c of clusters) {
        const [sx, sy] = worldToScreen(c.x, c.y);

        // Show actual spawned/hr if we have enough sim time, else fall back to configured rate
        const perHr = simHours > 0.05
            ? Math.round(c.spawnCount / simHours)
            : Math.round(c.totalRate * 3600);

        const label = `${perHr}/hr`;

        const pad = 5, fontSize = 11;
        ctx.font = `bold ${fontSize}px system-ui`;
        const tw  = ctx.measureText(label).width;

        // Background pill
        ctx.fillStyle = 'rgba(15,20,30,0.82)';
        ctx.beginPath();
        const rx = sx - tw/2 - pad, ry = sy - fontSize/2 - pad + 1;
        ctx.roundRect(rx, ry, tw + pad*2, fontSize + pad*2 - 2, 4);
        ctx.fill();

        // Accent dot
        ctx.beginPath();
        ctx.arc(sx, sy + fontSize/2 + pad + 5, 3, 0, Math.PI * 2);
        ctx.fillStyle = '#4af';
        ctx.fill();

        // Text
        ctx.fillStyle = '#e8f4ff';
        ctx.fillText(label, sx, sy + 1);
    }

    ctx.restore();
}

export function drawVehicleIds(ctx, worldToScreen, vehicles, vp) {
    if (vp.scale < 8) return; // only show at higher zoom
    ctx.font = `bold ${Math.round(Math.max(8, 9 * vp.scale / 15))}px monospace`;
    ctx.fillStyle = 'rgba(255,255,255,0.75)';
    ctx.textAlign = 'center';
    for (const v of vehicles) {
        const [sx, sy] = worldToScreen(v.x, v.y);
        ctx.fillText(v.id, sx, sy - 10 * vp.scale / 10);
    }
}

function drawConflictPoints(ctx, worldToScreen, map, vp, dynamicIntersections) {
    if (!map.intersections) return;
    const locksByIntersection = new Map();
    if (dynamicIntersections) {
        for (const di of dynamicIntersections) {
            if (di.locks) {
                const s = new Set(); for (const l of di.locks) s.add(l.c);
                locksByIntersection.set(di.id, s);
            }
        }
    }
    for (const inter of map.intersections) {
        if (!inter.conflicts) continue;
        const activeLocks = locksByIntersection.get(inter.id);
        for (const c of inter.conflicts) {
            const [sx, sy] = worldToScreen(c.x, c.y);
            const isLocked = activeLocks ? activeLocks.has(c.id) : false;
            const bufR = 5.0 * vp.scale;
            ctx.beginPath(); ctx.arc(sx, sy, bufR, 0, Math.PI * 2);
            ctx.strokeStyle = isLocked ? 'rgba(255,60,60,0.45)' : 'rgba(80,255,120,0.18)';
            ctx.lineWidth = 1; ctx.setLineDash([4, 4]); ctx.stroke(); ctx.setLineDash([]);
            ctx.beginPath(); ctx.arc(sx, sy, 0.5 * vp.scale, 0, Math.PI * 2);
            ctx.fillStyle = isLocked ? '#f44' : '#4f9'; ctx.fill();
        }
    }
}

function drawTrafficLights(ctx, worldToScreen, map, vp, dynamicIntersections) {
    if (!map.intersections) return;
    const lightMap = new Map();
    if (dynamicIntersections) {
        for (const di of dynamicIntersections) {
            if (di.lights) {
                const m = new Map(); for (const l of di.lights) m.set(l.p, l.s);
                lightMap.set(di.id, m);
            }
        }
    }
    for (const inter of map.intersections) {
        const states = lightMap.get(inter.id);
        if (!states || !inter.paths) continue;
        inter.paths.forEach((pathInfo, idx) => {
            const state = states.get(idx); if (state === undefined) return;
            const [sx, sy] = worldToScreen(pathInfo.x, pathInfo.y);
            ctx.beginPath(); ctx.arc(sx, sy, 1.5 * vp.scale, 0, Math.PI * 2);
            ctx.fillStyle = state === 0 ? '#f00' : state === 1 ? '#fe0' : '#0f0';
            ctx.fill();
            ctx.strokeStyle = '#000'; ctx.lineWidth = 1; ctx.stroke();
        });
    }
}

export function drawSelectedDebug(ctx, worldToScreen, vp, selected, leader, detail, opts) {
    if (!selected) return;
    const [sx, sy] = worldToScreen(selected.x, selected.y);

    // ── Route polyline ────────────────────────────────────────────────────
    if (detail && detail.routePolyline && detail.routePolyline.length > 1) {
        const isPark = detail.isParkingRoute;
        ctx.strokeStyle = isPark ? 'rgba(255,200,60,0.75)' : 'rgba(80,180,255,0.75)';
        ctx.lineWidth   = Math.max(1.5, vp.scale * 0.4);
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        const p0 = worldToScreen(detail.routePolyline[0].x, detail.routePolyline[0].y);
        ctx.moveTo(p0[0], p0[1]);
        for (let i = 1; i < detail.routePolyline.length; i++) {
            const p = worldToScreen(detail.routePolyline[i].x, detail.routePolyline[i].y);
            ctx.lineTo(p[0], p[1]);
        }
        ctx.stroke();
        ctx.setLineDash([]);

        // Arrowhead at the end
        const last = detail.routePolyline[detail.routePolyline.length - 1];
        const prev = detail.routePolyline[detail.routePolyline.length - 2];
        const [ex, ey] = worldToScreen(last.x, last.y);
        const [px, py] = worldToScreen(prev.x, prev.y);
        const ang = Math.atan2(ey - py, ex - px);
        const arrowLen = Math.max(8, vp.scale * 1.2);
        ctx.fillStyle = isPark ? 'rgba(255,200,60,0.85)' : 'rgba(80,180,255,0.85)';
        ctx.beginPath();
        ctx.moveTo(ex, ey);
        ctx.lineTo(ex - arrowLen * Math.cos(ang - 0.4), ey - arrowLen * Math.sin(ang - 0.4));
        ctx.lineTo(ex - arrowLen * Math.cos(ang + 0.4), ey - arrowLen * Math.sin(ang + 0.4));
        ctx.closePath();
        ctx.fill();
    }

    // ── Parking destination area ──────────────────────────────────────────
    if (detail && detail.isParkingRoute && detail.parkingDest && detail.parkingSearchRadius > 0) {
        const [dx, dy] = worldToScreen(detail.parkingDest.x, detail.parkingDest.y);
        const radiusPx = detail.parkingSearchRadius * vp.scale;

        // Filled circle (low opacity)
        ctx.beginPath();
        ctx.arc(dx, dy, radiusPx, 0, Math.PI * 2);
        ctx.fillStyle = detail.parkingSearchRadius > 10
            ? 'rgba(255,200,60,0.10)'   // search zone (large)
            : 'rgba(255,150,50,0.20)';  // claimed spot (small)
        ctx.fill();

        // Ring stroke
        ctx.beginPath();
        ctx.arc(dx, dy, radiusPx, 0, Math.PI * 2);
        ctx.strokeStyle = detail.parkingSearchRadius > 10
            ? 'rgba(255,200,60,0.60)'
            : 'rgba(255,150,50,0.80)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([5, 3]);
        ctx.stroke();
        ctx.setLineDash([]);

        // Label
        if (vp.scale > 3) {
            ctx.fillStyle = 'rgba(255,220,80,0.9)';
            ctx.font = `bold ${Math.round(Math.max(10, 11 * vp.scale / 15))}px monospace`;
            ctx.textAlign = 'center';
            const label = detail.parkingSearchRadius > 10 ? 'PARKING SEARCH' : 'TARGET SPOT';
            ctx.fillText(label, dx, dy - radiusPx - 6);
        }
    }

    // ── Parked state badge ────────────────────────────────────────────────────
    if (detail && detail.isParked) {
        const t = detail.timeUntilDeparture ?? 0;
        const h = Math.floor(t / 3600);
        const m = Math.floor((t % 3600) / 60);
        const s = Math.floor(t % 60);
        const timeStr = h > 0
            ? `${h}h ${m.toString().padStart(2,'0')}m`
            : `${m}m ${s.toString().padStart(2,'0')}s`;
        const label1 = 'PARKED';
        const label2 = `departs in ${timeStr}`;

        const fontSize = Math.round(Math.max(11, 12 * vp.scale / 15));
        ctx.font = `bold ${fontSize}px monospace`;
        const w1 = ctx.measureText(label1).width;
        ctx.font = `${fontSize - 1}px monospace`;
        const w2 = ctx.measureText(label2).width;
        const boxW = Math.max(w1, w2) + 14;
        const boxH = fontSize * 2.6;
        const bx = sx - boxW / 2;
        const by = sy - 28 - vp.scale * 2 - boxH;

        // Background pill
        ctx.fillStyle = 'rgba(20,20,30,0.82)';
        ctx.beginPath();
        ctx.roundRect(bx, by, boxW, boxH, 5);
        ctx.fill();
        ctx.strokeStyle = 'rgba(150,150,255,0.55)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // "PARKED" label
        ctx.fillStyle = '#aaf';
        ctx.font = `bold ${fontSize}px monospace`;
        ctx.textAlign = 'center';
        ctx.fillText(label1, sx, by + fontSize + 1);

        // Departure countdown
        ctx.fillStyle = 'rgba(200,220,255,0.85)';
        ctx.font = `${fontSize - 1}px monospace`;
        ctx.fillText(label2, sx, by + boxH - 4);
    }

    // Selection ring
    ctx.beginPath();
    ctx.arc(sx, sy, 22 + vp.scale * 2, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(255,255,255,0.75)'; ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]); ctx.stroke(); ctx.setLineDash([]);

    // Leader line
    if (leader) {
        const [lx, ly] = worldToScreen(leader.x, leader.y);
        const safeGap  = (selected.speed ?? 0) * 2.0;
        const gap      = detail ? detail.gap : 0;
        ctx.strokeStyle = gap < safeGap ? '#f44' : '#4f9';
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(lx, ly); ctx.stroke();
        ctx.fillStyle = '#fff'; ctx.font = 'bold 12px monospace'; ctx.textAlign = 'center';
        ctx.fillText(detail ? detail.gap.toFixed(1) + 'm' : '', (sx + lx) / 2, (sy + ly) / 2 - 4);
    }

    // Yield line
    if (opts && opts.yield && detail && detail.isYielding && detail.conflictPoint) {
        const [cx, cy] = worldToScreen(detail.conflictPoint.x, detail.conflictPoint.y);
        ctx.strokeStyle = '#f0f'; ctx.lineWidth = 2;
        ctx.setLineDash([2, 2]);
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(cx, cy); ctx.stroke();
        ctx.fillStyle = '#f0f'; ctx.font = 'bold 11px monospace'; ctx.textAlign = 'center';
        ctx.fillText('YIELD', cx, cy - 10);
        ctx.setLineDash([]);
    }

    // ── Agent history panel ────────────────────────────────────────────────────
    if (detail && detail.history && detail.history.length > 0) {
        const events = detail.history;
        const fSz   = 11;
        const lineH = fSz + 3;
        const PAD   = 6;
        const panelW = 220;
        const panelH = PAD * 2 + events.length * lineH;

        // Position to the right of the vehicle
        const panelX = sx + 28 + vp.scale * 2;
        const panelY = sy - panelH / 2;

        // Background
        ctx.fillStyle = 'rgba(10,12,20,0.88)';
        ctx.beginPath();
        ctx.roundRect(panelX, panelY, panelW, panelH, 5);
        ctx.fill();
        ctx.strokeStyle = 'rgba(100,140,255,0.45)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // Title
        ctx.fillStyle = 'rgba(160,180,255,0.9)';
        ctx.font = `bold ${fSz}px monospace`;
        ctx.textAlign = 'left';

        // Type → color map
        const typeColor = {
            Spawned:              '#6f9',
            StartedParkingSearch: '#fc6',
            TargetedSpot:         '#fa0',
            Parked:               '#aaf',
            Departed:             '#4df',
            RouteRegenerated:     '#ccc',
        };

        for (let i = 0; i < events.length; i++) {
            const ev   = events[i];
            const y    = panelY + PAD + i * lineH + fSz - 1;
            const mins = (ev.simTime / 60).toFixed(1);
            const col  = typeColor[ev.type] ?? '#ccc';

            ctx.fillStyle = 'rgba(120,130,150,0.7)';
            ctx.font = `${fSz - 1}px monospace`;
            ctx.fillText(`${mins}m`, panelX + PAD, y);

            ctx.fillStyle = col;
            ctx.font = `bold ${fSz}px monospace`;
            ctx.fillText(ev.type, panelX + PAD + 38, y);

            if (ev.detail) {
                ctx.fillStyle = 'rgba(200,210,230,0.75)';
                ctx.font = `${fSz - 1}px monospace`;
                // Truncate long detail text
                const maxW = panelW - PAD - 38 - PAD - ctx.measureText(ev.type + ' ').width;
                let detail_text = ev.detail;
                while (detail_text.length > 4 && ctx.measureText(detail_text).width > maxW) {
                    detail_text = detail_text.slice(0, -4) + '…';
                }
                ctx.fillText(detail_text, panelX + PAD + 38 + ctx.measureText(ev.type + ' ').width, y);
            }
        }
        ctx.textAlign = 'left';
    }

    // Hitbox
    if (opts && opts.hitbox && detail && detail.hitbox && detail.hitbox.length > 0) {
        ctx.beginPath();
        const p0 = worldToScreen(detail.hitbox[0].x, detail.hitbox[0].y);
        ctx.moveTo(p0[0], p0[1]);
        for (let i = 1; i < detail.hitbox.length; i++) {
            const p = worldToScreen(detail.hitbox[i].x, detail.hitbox[i].y);
            ctx.lineTo(p[0], p[1]);
        }
        ctx.closePath();
        ctx.fillStyle = 'rgba(0,255,255,0.12)'; ctx.fill();
        ctx.strokeStyle = 'rgba(0,255,255,0.5)'; ctx.lineWidth = 1;
        ctx.setLineDash([2, 2]); ctx.stroke(); ctx.setLineDash([]);
    }
}

export function drawRawOSM(ctx, worldToScreen, map, vp) {
    if (!map.rawOsm) return;
    const raw = map.rawOsm;
    ctx.strokeStyle = 'rgba(0,255,255,0.55)'; ctx.lineWidth = 1;
    ctx.beginPath();
    for (const seg of raw.segments) {
        const n1 = raw.nodes[seg.from], n2 = raw.nodes[seg.to];
        if (!n1 || !n2) continue;
        const [sx1, sy1] = worldToScreen(n1.x, n1.y);
        const [sx2, sy2] = worldToScreen(n2.x, n2.y);
        ctx.moveTo(sx1, sy1); ctx.lineTo(sx2, sy2);
    }
    ctx.stroke();
    ctx.fillStyle = 'rgba(255,0,255,0.65)';
    const r = Math.max(1.5, 2.5 * vp.scale);
    for (const id in raw.nodes) {
        const n = raw.nodes[id];
        const [sx, sy] = worldToScreen(n.x, n.y);
        ctx.beginPath(); ctx.arc(sx, sy, r, 0, Math.PI * 2); ctx.fill();
    }
}

/**
 * Draw parking spots as small oriented rectangles on the 2D overlay.
 * orientation: 1=Parallel, 2=Diagonal, 3=Perpendicular
 * Only drawn when scale is large enough to be visible (> 3 px/m).
 */
export function drawParkingSpots(ctx, worldToScreen, spots, vp) {
    if (!spots || spots.length === 0) return;
    if (vp.scale < 2) return; // too zoomed out to see

    // Spot dimensions in world units (metres) by orientation
    const DIMS = {
        1: { hw: 3.0, hd: 1.1 },  // parallel: 6m × 2.2m
        2: { hw: 1.8, hd: 2.5 },  // diagonal:  ~3.5m × 5m
        3: { hw: 1.25, hd: 2.5 }, // perp: 2.5m × 5m
    };

    for (const spot of spots) {
        const dim = DIMS[spot.orientation] ?? DIMS[1];
        const hw = dim.hw; // half-width along heading
        const hd = dim.hd; // half-depth perpendicular to heading

        const c = Math.cos(spot.heading), s = Math.sin(spot.heading);
        // four corners in world space
        const corners = [
            { x: spot.x + c * hw - s * hd, y: spot.y + s * hw + c * hd },
            { x: spot.x + c * hw + s * hd, y: spot.y + s * hw - c * hd },
            { x: spot.x - c * hw + s * hd, y: spot.y - s * hw - c * hd },
            { x: spot.x - c * hw - s * hd, y: spot.y - s * hw + c * hd },
        ];
        const sc = corners.map(p => worldToScreen(p.x, p.y));

        ctx.beginPath();
        ctx.moveTo(sc[0][0], sc[0][1]);
        for (let i = 1; i < 4; i++) ctx.lineTo(sc[i][0], sc[i][1]);
        ctx.closePath();

        ctx.fillStyle   = spot.occupied ? 'rgba(220,80,80,0.25)' : 'rgba(80,200,120,0.18)';
        ctx.strokeStyle = spot.occupied ? 'rgba(220,80,80,0.70)' : 'rgba(80,200,120,0.55)';
        ctx.lineWidth   = 1;
        ctx.fill();
        ctx.stroke();
    }
}
