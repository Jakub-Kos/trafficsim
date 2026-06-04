/*
 * Note: Claude (Anthropic) was used under human supervision to implement this
 * background tile renderer as a feature addition. The author directed the
 * coordinate-system integration with the existing viewport.
 */
/**
 * scene.js — Background tile/OSM renderer for Scene view mode
 *
 * Uses a 2D canvas (#bgCanvas) positioned behind the WebGL canvas.
 * Supports two background sources:
 *   'satellite' — ESRI World Imagery tiles
 *   'osm'       — OpenStreetMap standard tiles
 *
 * Coordinate system bridge:
 *   Simulation uses equirectangular projection centred on (refLat, refLon):
 *     worldX = (lon - refLon) * DEG2RAD * R * cos(refLat * DEG2RAD)
 *     worldY = (lat - refLat) * DEG2RAD * R
 *   (Y is world-up; canvas Y is screen-down — viewport.js flips it)
 *
 *   Tile coordinates (slippy map):
 *     tileX = floor((lon + 180) / 360 * 2^z)
 *     tileY = floor((1 - ln(tan(lat*PI/180) + 1/cos(lat*PI/180)) / PI) / 2 * 2^z)
 */

const R = 6_378_137; // Earth radius (metres)
const DEG2RAD = Math.PI / 180;

// ── Tile URL factories ────────────────────────────────────────────────────────
const SUBDOMAINS = ['a', 'b', 'c'];
let subIdx = 0;

function tileUrl(source, z, tx, ty) {
    if (source === 'satellite') {
        // ESRI Clarity — true nadir (straight-down) imagery, higher quality than standard WorldImagery
        return `https://clarity.maptiles.arcgis.com/arcgis/rest/services/World_Imagery/MapServer/tile/${z}/${ty}/${tx}`;
    }
    const sub = SUBDOMAINS[subIdx++ % 3];
    return `https://${sub}.tile.openstreetmap.org/${z}/${tx}/${ty}.png`;
}

// ── Coordinate conversions ────────────────────────────────────────────────────
function worldToLatLon(wx, wy, refLat, refLon) {
    const cosRef = Math.cos(refLat * DEG2RAD);
    const lat = wy / (DEG2RAD * R) + refLat;
    const lon = wx / (DEG2RAD * R * cosRef) + refLon;
    return { lat, lon };
}

function latLonToTile(lat, lon, z) {
    const n = Math.pow(2, z);
    const tx = Math.floor((lon + 180) / 360 * n);
    const latR = lat * DEG2RAD;
    const ty = Math.floor((1 - Math.log(Math.tan(latR) + 1 / Math.cos(latR)) / Math.PI) / 2 * n);
    return { tx, ty };
}

function tileToLatLon(tx, ty, z) {
    const n = Math.pow(2, z);
    const lon = tx / n * 360 - 180;
    const latR = Math.atan(Math.sinh(Math.PI * (1 - 2 * ty / n)));
    return { lat: latR / DEG2RAD, lon };
}

// Convert lat/lon → simulation world coordinates
function latLonToWorld(lat, lon, refLat, refLon) {
    const cosRef = Math.cos(refLat * DEG2RAD);
    return {
        x: (lon - refLon) * DEG2RAD * R * cosRef,
        y: (lat - refLat) * DEG2RAD * R,
    };
}

// ── SceneView class ───────────────────────────────────────────────────────────
export class SceneView {
    /**
     * @param {HTMLCanvasElement} canvas  — the bgCanvas element
     * @param {object} vp                 — viewport object from viewport.js
     * @param {Function} worldToScreen    — (x,y) → [sx, sy] from viewport.js
     */
    constructor(canvas, vp, worldToScreen) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.vp = vp;
        this.worldToScreen = worldToScreen;

        this.source = 'satellite'; // 'satellite' | 'osm'
        this.refLat = 0;
        this.refLon = 0;
        this._ready = false;

        // Tile image cache: key = `z/tx/ty` → HTMLImageElement (or null if error)
        this._cache = new Map();
        this._loading = new Set(); // keys currently being fetched
    }

    setOrigin(refLat, refLon) {
        this.refLat = refLat;
        this.refLon = refLon;
        this._ready = true;
    }

    setSource(source) {
        if (this.source === source) return;
        this.source = source;
        this._cache.clear();
        this._loading.clear();
    }

    /** Resize the backing canvas to match CSS size × DPR */
    _resize() {
        const dpr = window.devicePixelRatio || 1;
        const w = Math.round(this.canvas.clientWidth * dpr);
        const h = Math.round(this.canvas.clientHeight * dpr);
        if (this.canvas.width !== w || this.canvas.height !== h) {
            this.canvas.width = w;
            this.canvas.height = h;
        }
    }

    /** Pick a zoom level that keeps tiles reasonably sized on screen */
    _chooseZoom() {
        // vp.scale is world-metres per CSS-pixel (inverted in viewport.js)
        // At zoom z, one tile = 2*PI*R / 2^z metres wide, renders as tilePixels px
        // We want tilePixels ≈ 256 px on screen.
        const z = Math.round(Math.log2(this.vp.scale * 2 * Math.PI * R / 256));
        return Math.max(1, Math.min(19, z));
    }

    /** Fetch (or return cached) tile image.  Returns null while loading. */
    _getTile(z, tx, ty) {
        const key = `${this.source}/${z}/${tx}/${ty}`;
        if (this._cache.has(key)) return this._cache.get(key);
        if (this._loading.has(key)) return null;

        this._loading.add(key);
        const img = new Image();
        img.crossOrigin = 'anonymous';
        const url = tileUrl(this.source, z, tx, ty);
        img.src = url;
        console.log('[Scene] fetching tile:', url);
        img.onload = () => {
            console.log('[Scene] tile loaded:', key);
            this._cache.set(key, img);
            this._loading.delete(key);
            // Re-render when tile arrives
            this.render();
        };
        img.onerror = () => {
            console.warn('[Scene] tile error:', url);
            this._cache.set(key, null); // mark as failed so we don't retry immediately
            this._loading.delete(key);
        };
        return null;
    }

    render() {
        if (!this._ready) {
            console.warn('[Scene] render() called but not ready (no origin set)');
            return;
        }

        this._resize();
        const dpr = window.devicePixelRatio || 1;
        const ctx = this.ctx;
        const W = this.canvas.width;
        const H = this.canvas.height;

        ctx.save();
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, W, H);
        ctx.fillStyle = '#080c12';
        ctx.fillRect(0, 0, W, H);
        ctx.restore();

        const z = this._chooseZoom();
        const n = Math.pow(2, z);

        // ── Figure out which tiles cover the visible area ─────────────────
        // Convert the four canvas corners to lat/lon
        const corners = [
            [0, 0], [W / dpr, 0], [0, H / dpr], [W / dpr, H / dpr],
        ].map(([sx, sy]) => {
            // screenToWorld from viewport.js: x = cx + (sx - W/2/dpr) / scale
            //                                 y = cy - (sy - H/2/dpr) / scale
            const wx = this.vp.cx + (sx - W / (2 * dpr)) / this.vp.scale;
            const wy = this.vp.cy - (sy - H / (2 * dpr)) / this.vp.scale;
            return worldToLatLon(wx, wy, this.refLat, this.refLon);
        });

        const lats = corners.map(c => c.lat);
        const lons = corners.map(c => c.lon);
        const minLat = Math.min(...lats), maxLat = Math.max(...lats);
        const minLon = Math.min(...lons), maxLon = Math.max(...lons);

        // Tile range (note: tileY increases downward, so minLat → maxTy)
        const { tx: txMin, ty: tyMax } = latLonToTile(minLat, minLon, z);
        const { tx: txMax, ty: tyMin } = latLonToTile(maxLat, maxLon, z);

        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        const txLo = clamp(txMin, 0, n - 1);
        const txHi = clamp(txMax, 0, n - 1);
        const tyLo = clamp(tyMin, 0, n - 1);
        const tyHi = clamp(tyMax, 0, n - 1);
        console.log(`[Scene] z=${z} tiles x:${txLo}-${txHi} y:${tyLo}-${tyHi} scale=${this.vp.scale.toFixed(4)}`);

        ctx.save();
        ctx.scale(dpr, dpr); // draw in CSS pixels

        for (let ty = tyLo; ty <= tyHi; ty++) {
            for (let tx = txLo; tx <= txHi; tx++) {
                const img = this._getTile(z, tx, ty);

                // Tile top-left corner (lat/lon) → world metres
                const { lat: lat0, lon: lon0 } = tileToLatLon(tx, ty, z);
                // Tile bottom-right corner → tileToLatLon(tx+1, ty+1, z)
                const { lat: lat1, lon: lon1 } = tileToLatLon(tx + 1, ty + 1, z);

                // Top-left tile corner → screen
                const w0 = latLonToWorld(lat0, lon0, this.refLat, this.refLon);
                const [sx0, sy0] = this.worldToScreen(w0.x, w0.y);

                // Bottom-right tile corner → screen
                const w1 = latLonToWorld(lat1, lon1, this.refLat, this.refLon);
                const [sx1, sy1] = this.worldToScreen(w1.x, w1.y);

                const tileW = sx1 - sx0;
                const tileH = sy1 - sy0;

                if (img) {
                    ctx.drawImage(img, sx0, sy0, tileW, tileH);
                } else {
                    // Placeholder while loading
                    ctx.fillStyle = '#0d1420';
                    ctx.fillRect(sx0, sy0, tileW, tileH);
                }
            }
        }

        ctx.restore();
    }
}
