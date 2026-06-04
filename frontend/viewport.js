/*
 * Note: The pan/zoom viewport implementation is the author's own work.
 * Claude (Anthropic) was used only to assist with documenting non-obvious
 * coordinate-system decisions.
 */

// Pan/zoom viewport — WebGL version
// makeViewport(interactCanvas, glCanvas)
// Legacy compat: makeViewport(canvas, ctx2d) also works — uses same canvas for both roles.

export function makeViewport(interactCanvas, glCanvasOrCtx) {
    // Detect legacy call: second arg is a 2D context, not a canvas
    const glCanvas = (glCanvasOrCtx instanceof HTMLCanvasElement)
        ? glCanvasOrCtx
        : interactCanvas;
    const vp = {
        cx: 0, cy: 0, scale: 3,
        targetCx: 0, targetCy: 0, targetScale: 3,
        smoothing: 0.18,
        minScale: 0.5,
        maxScale: 200,
        autofit: true,
    };

    function resize() {
        const dpr = Math.max(1, window.devicePixelRatio || 1);
        const w = window.innerWidth, h = window.innerHeight;
        const canvases = glCanvas === interactCanvas
            ? [glCanvas]
            : [glCanvas, interactCanvas];
        for (const c of canvases) {
            c.style.width  = w + 'px';
            c.style.height = h + 'px';
            c.width  = Math.floor(w * dpr);
            c.height = Math.floor(h * dpr);
        }
    }
    window.addEventListener('resize', resize);
    resize();

    function step() {
        const a = vp.smoothing;
        vp.cx    += (vp.targetCx    - vp.cx)    * a;
        vp.cy    += (vp.targetCy    - vp.cy)    * a;
        vp.scale += (vp.targetScale - vp.scale) * a;
    }

    // Returns column-major mat3 for WebGL (world → clip space)
    // World: Y-up.  WebGL NDC: Y-up.  No flip needed.
    function getViewMatrix() {
        const dpr = window.devicePixelRatio || 1;
        const W = glCanvas.width  / dpr;
        const H = glCanvas.height / dpr;
        const sx = 2 * vp.scale / W;
        const sy = 2 * vp.scale / H;
        // Column-major Float32Array for gl.uniformMatrix3fv
        return new Float32Array([
            sx,            0,  0,
            0,            sy,  0,
            -vp.cx * sx,  -vp.cy * sy,  1,
        ]);
    }

    function cssWidth()  { return glCanvas.width  / (window.devicePixelRatio || 1); }
    function cssHeight() { return glCanvas.height / (window.devicePixelRatio || 1); }

    function worldToScreen(x, y) {
        return [
            cssWidth()  * 0.5 + (x - vp.cx) * vp.scale,
            cssHeight() * 0.5 - (y - vp.cy) * vp.scale,
        ];
    }

    function screenToWorld(sx, sy) {
        return [
            (sx - cssWidth()  * 0.5) / vp.scale + vp.cx,
           -(sy - cssHeight() * 0.5) / vp.scale + vp.cy,
        ];
    }

    const drag = { active: false, startScreen: [0, 0], startCenter: [0, 0] };
    interactCanvas.addEventListener('contextmenu', e => e.preventDefault());

    let onUserControl = null;
    function setUserControlCallback(cb) { onUserControl = cb; }

    interactCanvas.addEventListener('mousedown', (e) => {
        if (e.button !== 0) return;
        e.preventDefault();
        if (onUserControl) onUserControl();
        vp.autofit = false;
        drag.active = true;
        drag.startScreen = [e.clientX, e.clientY];
        drag.startCenter = [vp.targetCx, vp.targetCy];
        interactCanvas.style.cursor = 'grabbing';
    });

    window.addEventListener('mousemove', (e) => {
        if (!drag.active) return;
        const dx = e.clientX - drag.startScreen[0];
        const dy = e.clientY - drag.startScreen[1];
        vp.targetCx = drag.startCenter[0] - dx / vp.scale;
        vp.targetCy = drag.startCenter[1] + dy / vp.scale;
    });

    window.addEventListener('mouseup', () => {
        drag.active = false;
        interactCanvas.style.cursor = 'crosshair';
    });

    interactCanvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        if (onUserControl) onUserControl();
        vp.autofit = false;

        const rect = interactCanvas.getBoundingClientRect();
        const sx = e.clientX - rect.left;
        const sy = e.clientY - rect.top;
        const [wx, wy] = screenToWorld(sx, sy);

        const zoomStep   = Math.pow(1.0018, -e.deltaY);
        const newScale   = Math.max(vp.minScale, Math.min(vp.maxScale, vp.targetScale * zoomStep));
        const W = cssWidth(), H = cssHeight();

        vp.targetCx    = wx - (sx - W * 0.5) / newScale;
        vp.targetCy    = wy + (sy - H * 0.5) / newScale;
        vp.targetScale = newScale;
    }, { passive: false });

    return { vp, step, getViewMatrix, worldToScreen, screenToWorld, resize, setUserControlCallback };
}
