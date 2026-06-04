/*
 * Note: Claude (Anthropic) was used under human supervision to implement this
 * performance monitoring overlay as a feature addition.
 */
const UPDATE_INTERVAL_MS = 500;

let panel = null;
let fpsElem = null;
let listElem = null;

let frameCount = 0;
let lastUpdateT = 0;

// Current frame state
let frameStartT = 0;
let lastMarkT = 0;
let metrics = []; // Array of { label, dt }

// Accumulators for averaging
let accumMetrics = new Map(); // label -> totalTime
let accumFrameTime = 0;
let accumSamples = 0;

export function initPerf() {
    panel = document.getElementById('perfPanel');
    fpsElem = document.getElementById('perf-fps');
    listElem = document.getElementById('perf-breakdown');
    lastUpdateT = performance.now();
}

export function perfBegin() {
    frameStartT = performance.now();
    lastMarkT = frameStartT;
    metrics = [];
}

export function perfMark(label) {
    const now = performance.now();
    const dt = now - lastMarkT;
    lastMarkT = now;
    metrics.push({ label, dt });
}

export function perfEnd() {
    const now = performance.now();
    const totalFrameTime = now - frameStartT;

    // 1. Accumulate data
    accumFrameTime += totalFrameTime;
    accumSamples++;
    frameCount++;

    for (const m of metrics) {
        const current = accumMetrics.get(m.label) || 0;
        accumMetrics.set(m.label, current + m.dt);
    }

    // 2. Check if UI update is due
    if (now - lastUpdateT > UPDATE_INTERVAL_MS) {
        updateUI(now);

        // Reset accumulators
        lastUpdateT = now;
        frameCount = 0;
        accumFrameTime = 0;
        accumSamples = 0;
        accumMetrics.clear();
    }
}

function updateUI(now) {
    if (!fpsElem || !listElem) return;

    // Calculate Averages
    const avgFrameTime = accumFrameTime / accumSamples;
    const fps = Math.round((frameCount * 1000) / (now - (lastUpdateT - UPDATE_INTERVAL_MS))); // Approximate FPS based on real time elapsed

    // Colorize status
    let statusColor = '#4f9'; // Green
    if (avgFrameTime > 16) statusColor = '#fe0'; // Yellow (drop below 60)
    if (avgFrameTime > 33) statusColor = '#f44'; // Red (drop below 30)

    fpsElem.innerHTML = `
        <div style="font-size:17px;font-weight:700;color:${statusColor};font-family:monospace;letter-spacing:1px">${fps} FPS</div>
        <div style="font-size:11px;color:#4a6878;margin-top:1px">${avgFrameTime.toFixed(2)} ms/frame</div>
    `;

    // Render Breakdown list
    let html = '';
    // Sort logic? Or keep call order? Call order is usually better for timeline understanding.
    // We iterate over the *keys* of the map to preserve insertion order (mostly),
    // but better to rely on a predefined order or just map iteration.

    accumMetrics.forEach((totalTime, label) => {
        const avg = totalTime / accumSamples;
        const pct = (avg / avgFrameTime) * 100;

        const barColor = label.includes('GL') ? '#00b8e6' : '#6688cc';

        html += `
            <div style="display:flex;justify-content:space-between;margin-top:5px;font-size:10.5px;">
                <span style="color:#5a7888">${label}</span>
                <span style="font-family:monospace;color:#b8d0e0">${avg.toFixed(2)} ms</span>
            </div>
            <div style="width:100%;height:2px;background:#0e1824;margin-top:2px;border-radius:2px;">
                <div style="width:${pct}%;height:100%;background:${barColor};border-radius:2px;"></div>
            </div>
        `;
    });

    listElem.innerHTML = html;
}