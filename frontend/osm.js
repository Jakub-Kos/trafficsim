/*
 * Note: Claude (Anthropic) was used under human supervision to implement this
 * OSM data inspector as a feature addition. The author directed the feature
 * set and reviewed all resulting code.
 */
import { fetchMap } from './net.js';
import { makeViewport } from './viewport.js';

const canvas = document.getElementById('osmCanvas');
const ctx = canvas.getContext('2d');
const statsDiv = document.getElementById('stats');

// UI Controls
const uiShowNodes = document.getElementById('showNodes');
const uiShowWays = document.getElementById('showWays');
const uiShowTrafficLights = document.getElementById('showTrafficLights');
const uiFilterRoadType = document.getElementById('filterRoadType');
const uiShowCalcLanes = document.getElementById('showCalcLanes');
const uiShowCalcPaths = document.getElementById('showCalcPaths');
const uiFilterWay = document.getElementById('filterWay');

const { vp, step, worldToScreen, screenToWorld } = makeViewport(canvas, ctx);

let rawData = null;
let fullMapData = null;

fetchMap().then(data => {
    fullMapData = data;
    if (data.rawOsm) {
        rawData = data.rawOsm;
        const nodeCount = Object.keys(rawData.nodes).length;
        const segmentCount = rawData.segments ? rawData.segments.length : 0;
        statsDiv.innerHTML = `Nodes: ${nodeCount}<br>Segments: ${segmentCount}`;
    } else {
        statsDiv.innerHTML = `<span style="color:#f44">No rawOsm data found!</span>`;
    }
}).catch(e => { statsDiv.innerHTML = `Error: ${e.message}`; });

// --- Math helper for clicking on a line segment ---
function distToSegmentSquared(px, py, vx, vy, wx, wy) {
    const l2 = (vx - wx) ** 2 + (vy - wy) ** 2;
    if (l2 === 0) return (px - vx) ** 2 + (py - vy) ** 2;
    let t = ((px - vx) * (wx - vx) + (py - vy) * (wy - vy)) / l2;
    t = Math.max(0, Math.min(1, t));
    return (px - (vx + t * (wx - vx))) ** 2 + (py - (vy + t * (wy - vy))) ** 2;
}

// Click to identify a Way
canvas.addEventListener('click', (e) => {
    if (!rawData || !rawData.segments) return;
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const [wx, wy] = screenToWorld(sx, sy);

    let bestWayId = null;
    let bestDistSq = 25.0; // Click radius squared (5m ^ 2)

    for (const seg of rawData.segments) {
        const n1 = rawData.nodes[String(seg.from)];
        const n2 = rawData.nodes[String(seg.to)];
        if (!n1 || !n2) continue;

        const dSq = distToSegmentSquared(wx, wy, n1.x, n1.y, n2.x, n2.y);
        if (dSq < bestDistSq) {
            bestDistSq = dSq;
            bestWayId = String(seg.wayId);
        }
    }

    if (bestWayId) {
        console.log(`%c[OSM Way Clicked] ID: ${bestWayId}`, 'color: #4f9; font-weight: bold;');
        uiFilterWay.value = bestWayId;
    } else {
        uiFilterWay.value = "";
    }
});

// Main Render Loop
function render() {
    step();
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#111';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (!rawData) {
        requestAnimationFrame(render);
        return;
    }

    const targetWayId = uiFilterWay.value.trim();
    const targetRoadType = parseInt(uiFilterRoadType.value);
    const showWays = uiShowWays.checked;
    const showNodes = uiShowNodes.checked;
    const showTL = uiShowTrafficLights.checked;
    const showLanes = uiShowCalcLanes.checked;
    const showPaths = uiShowCalcPaths.checked;

    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    // --- 0. Draw Calculated Overlay (Thinner) ---
    if (fullMapData) {
        if (showLanes && fullMapData.lanes) {
            ctx.lineWidth = Math.max(0.5, 1.5 * vp.scale); // Much thinner
            ctx.strokeStyle = 'rgba(200, 200, 255, 0.25)';
            for (const lane of fullMapData.lanes) {
                if (!lane.points || lane.points.length < 2) continue;
                ctx.beginPath();
                for (let i = 0; i < lane.points.length; i++) {
                    const [sx, sy] = worldToScreen(lane.points[i].x, lane.points[i].y);
                    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
                }
                ctx.stroke();
            }
        }

        if (showPaths && fullMapData.intersection_paths) {
            ctx.lineWidth = Math.max(0.5, 1.0 * vp.scale); // Very thin for yellow splines
            ctx.strokeStyle = 'rgba(255, 200, 50, 0.4)';
            for (const path of fullMapData.intersection_paths) {
                if (!path.points || path.points.length < 2) continue;
                ctx.beginPath();
                for (let i = 0; i < path.points.length; i++) {
                    const [sx, sy] = worldToScreen(path.points[i].x, path.points[i].y);
                    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
                }
                ctx.stroke();
            }
        }
    }

    // --- 1. Draw Raw Ways (Segments) ---
    if (showWays && rawData.segments) {
        ctx.lineWidth = Math.max(1, 2 * vp.scale);

        for (const seg of rawData.segments) {
            if (targetWayId && String(seg.wayId) !== targetWayId) continue;
            if (targetRoadType !== -1 && seg.roadType !== targetRoadType && !targetWayId) continue;

            const n1 = rawData.nodes[String(seg.from)];
            const n2 = rawData.nodes[String(seg.to)];

            if (n1 && n2) {
                const [sx1, sy1] = worldToScreen(n1.x, n1.y);
                const [sx2, sy2] = worldToScreen(n2.x, n2.y);

                ctx.beginPath();
                ctx.moveTo(sx1, sy1);
                ctx.lineTo(sx2, sy2);

                if (targetWayId === String(seg.wayId)) {
                    ctx.strokeStyle = '#4f9';
                    ctx.lineWidth = Math.max(2, 4 * vp.scale);
                } else {
                    ctx.strokeStyle = 'rgba(0, 255, 255, 0.4)';
                    ctx.lineWidth = Math.max(1, 2 * vp.scale);
                }
                ctx.stroke();
            }
        }
    }

    // --- 2. Draw Nodes (Vertices) ---
    if (showNodes || showTL) {
        const baseRadius = Math.max(1.0, 2 * vp.scale);

        for (const id in rawData.nodes) {
            const n = rawData.nodes[id];

            // Check if it's a traffic light (1 = TrafficLight enum in SimulationTypes.hpp)
            const isTrafficLight = (n.type === 1);

            if (!showNodes && !isTrafficLight) continue;
            if (targetWayId) continue; // Hide all raw nodes if focusing on a specific way to prevent clutter

            const [sx, sy] = worldToScreen(n.x, n.y);
            ctx.beginPath();

            if (isTrafficLight && showTL) {
                ctx.arc(sx, sy, baseRadius * 2.5, 0, Math.PI * 2);
                ctx.fillStyle = '#ffaa00'; // Bright Orange for Traffic Lights
                ctx.fill();
                ctx.lineWidth = 1;
                ctx.strokeStyle = '#fff';
                ctx.stroke();
            } else if (showNodes) {
                ctx.arc(sx, sy, baseRadius, 0, Math.PI * 2);
                ctx.fillStyle = 'rgba(255, 0, 0, 0.4)'; // Dim Red for standard nodes
                ctx.fill();
            }
        }
    }

    ctx.restore();
    requestAnimationFrame(render);
}

requestAnimationFrame(render);