/*
 * Note: Claude (Anthropic) was used under human supervision to implement the
 * project manager UI in this file. The author directed the feature set and
 * reviewed all resulting code.
 */
import { listProjects, createProject, saveProjectOsm, loadProject, deleteProject, updateProject } from './net.js';

// ── State ─────────────────────────────────────────────────────────────────────
let selectedSource = 'default';
let chosenFile     = null;

// ── Colour palette for project card thumbnails ────────────────────────────────
const CARD_THEMES = [
    { from: '#0ea5e9', to: '#6366f1' },
    { from: '#06b6d4', to: '#0284c7' },
    { from: '#10b981', to: '#059669' },
    { from: '#f59e0b', to: '#d97706' },
    { from: '#8b5cf6', to: '#6d28d9' },
    { from: '#ec4899', to: '#be185d' },
    { from: '#14b8a6', to: '#0f766e' },
    { from: '#f97316', to: '#c2410c' },
];

function cardTheme(name) {
    let hash = 0;
    for (let i = 0; i < name.length; i++) hash = (hash * 31 + name.charCodeAt(i)) | 0;
    return CARD_THEMES[Math.abs(hash) % CARD_THEMES.length];
}

// ── Time helpers ──────────────────────────────────────────────────────────────
function relTime(iso) {
    if (!iso) return 'Never opened';
    const diff = Date.now() - new Date(iso).getTime();
    const mins = Math.floor(diff / 60000);
    if (mins < 2)   return 'Just now';
    if (mins < 60)  return `${mins}m ago`;
    const hrs = Math.floor(mins / 60);
    if (hrs  < 24)  return `${hrs}h ago`;
    const days = Math.floor(hrs / 24);
    if (days < 30)  return `${days}d ago`;
    return new Date(iso).toLocaleDateString();
}

// ── Render project grid ───────────────────────────────────────────────────────
function renderProjects(projects) {
    const grid     = document.getElementById('projectGrid');
    const emptyEl  = document.getElementById('emptyState');
    const loadMsg  = document.getElementById('loadMsg');

    loadMsg.style.display = 'none';
    grid.innerHTML = '';

    if (!projects.length) {
        emptyEl.classList.add('show');
        return;
    }
    emptyEl.classList.remove('show');

    projects.forEach((p, i) => {
        const theme = cardTheme(p.name);
        const card  = document.createElement('div');
        card.className = 'proj-card';
        card.dataset.name = p.name;
        card.innerHTML = `
            <div class="card-thumb" style="background:linear-gradient(135deg,${theme.from}22,${theme.to}11)">
                <div class="card-thumb-grid" style="background-image:
                    linear-gradient(${theme.from}22 1px, transparent 1px),
                    linear-gradient(90deg,${theme.from}22 1px, transparent 1px)"></div>
                <div class="card-thumb-dots">
                    <span class="card-thumb-icon" style="color:${theme.from}88">◈</span>
                </div>
                ${!p.hasOsm ? '<div class="card-no-map-badge">No map data</div>' : ''}
                <div class="card-open-overlay">
                    <div class="card-open-label">Open →</div>
                </div>
            </div>
            <div class="card-body">
                <div class="card-name">${escHtml(p.name)}</div>
                <div class="card-desc">${escHtml(p.description || '—')}</div>
                <div class="card-footer">
                    <span class="card-date">${relTime(p.lastOpenedAt)}</span>
                    <div class="card-actions">
                        <button class="btn-ghost"  title="Edit project"   data-edit="${escHtml(p.name)}" style="font-size:11px;padding:4px 8px">✎</button>
                        <button class="btn-danger" title="Delete project" data-del="${escHtml(p.name)}">✕</button>
                    </div>
                </div>
            </div>
        `;

        // Open on card click (but not on action buttons)
        card.addEventListener('click', (e) => {
            if (e.target.closest('[data-del]') || e.target.closest('[data-edit]')) return;
            openProject(p.name, p.osmSize || 0);
        });

        // Edit
        card.querySelector('[data-edit]').addEventListener('click', (e) => {
            e.stopPropagation();
            openEditModal(p.name, p.description || '');
        });

        // Delete
        card.querySelector('[data-del]').addEventListener('click', (e) => {
            e.stopPropagation();
            confirmDelete(p.name);
        });

        grid.appendChild(card);
    });
}

function escHtml(s) {
    return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

// ── Open project ──────────────────────────────────────────────────────────────
let _loadCancelled = false;
let _loadTimer     = null;

function formatSize(bytes) {
    if (!bytes) return null;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(0)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

function showProjectLoadOverlay(name, osmSize) {
    _loadCancelled = false;
    document.getElementById('projLoadName').textContent  = `"${name}"`;
    document.getElementById('projLoadTimer').textContent = '0s';

    const sizeStr = formatSize(osmSize);
    document.getElementById('projLoadMeta').textContent =
        sizeStr ? `Map file: ${sizeStr}` : '';

    document.getElementById('projLoadOverlay').classList.add('show');

    let elapsed = 0;
    _loadTimer = setInterval(() => {
        elapsed++;
        document.getElementById('projLoadTimer').textContent = `${elapsed}s`;
    }, 1000);
}

function hideProjectLoadOverlay() {
    clearInterval(_loadTimer); _loadTimer = null;
    document.getElementById('projLoadOverlay').classList.remove('show');
}

document.getElementById('projLoadCancel').addEventListener('click', () => {
    _loadCancelled = true;
    hideProjectLoadOverlay();
});

async function openProject(name, osmSize = 0) {
    showProjectLoadOverlay(name, osmSize);
    try {
        await loadProject(name);
        if (_loadCancelled) return;
        sessionStorage.setItem('currentProject', name);
        window.location.href = '/app';
    } catch (err) {
        hideProjectLoadOverlay();
        // Show error in a visible place — create a brief toast above the grid
        const toast = document.createElement('div');
        toast.style.cssText = 'position:fixed;bottom:24px;left:50%;transform:translateX(-50%);background:#1a1020;border:1px solid rgba(255,51,85,0.4);color:#ff6080;padding:10px 20px;border-radius:8px;font-size:12px;z-index:400';
        toast.textContent = `Failed to load "${name}": ${err.message}`;
        document.body.appendChild(toast);
        setTimeout(() => toast.remove(), 5000);
    }
}

// ── Delete confirmation ───────────────────────────────────────────────────────
let _confirmResolve = null;

function confirmDelete(name) {
    document.getElementById('confirmTitle').textContent = `Delete "${name}"?`;
    document.getElementById('confirmMsg').textContent   =
        `This will permanently delete the project and all its map data. This cannot be undone.`;
    const overlay = document.getElementById('confirmOverlay');
    overlay.classList.add('show');

    _confirmResolve = async (ok) => {
        overlay.classList.remove('show');
        if (!ok) return;
        try {
            await deleteProject(name);
            loadAndRender();
        } catch (err) {
            alert('Delete failed: ' + err.message);
        }
    };
}

document.getElementById('confirmOk').addEventListener('click',    () => _confirmResolve?.(true));
document.getElementById('confirmCancel').addEventListener('click', () => _confirmResolve?.(false));
document.getElementById('confirmOverlay').addEventListener('click', (e) => {
    if (e.target === e.currentTarget) _confirmResolve?.(false);
});

// ── Load and render project list ──────────────────────────────────────────────
async function loadAndRender() {
    const loadMsg = document.getElementById('loadMsg');
    loadMsg.style.display = '';
    loadMsg.textContent = 'Loading projects…';
    try {
        const projects = await listProjects();
        renderProjects(projects);
    } catch (err) {
        loadMsg.textContent = 'Could not load projects: ' + err.message;
    }
}

// ── Modal ─────────────────────────────────────────────────────────────────────
const modalOverlay   = document.getElementById('modalOverlay');
const modalTitle     = document.querySelector('.modal-title');
const btnCreate      = document.getElementById('btnCreate');
const mapSourceGroup = document.querySelector('.source-options').closest('.form-group');

let editMode         = false;
let editingName      = null;   // original name when editing

function openModal() {
    editMode = false; editingName = null;
    document.getElementById('projName').value  = '';
    document.getElementById('projDesc').value  = '';
    selectedSource = 'default';
    chosenFile     = null;
    document.getElementById('fileNameDisplay').textContent = 'No file chosen';
    document.getElementById('uploadSection').classList.remove('show');
    document.querySelectorAll('.source-opt').forEach(o => o.classList.remove('selected'));
    document.querySelector('.source-opt[data-source="default"]').classList.add('selected');
    modalTitle.textContent = 'New Project';
    btnCreate.innerHTML    = 'Create Project →';
    mapSourceGroup.style.display = '';
    setStatus('', '');
    modalOverlay.classList.add('show');
    setTimeout(() => document.getElementById('projName').focus(), 150);
}

function openEditModal(name, desc) {
    editMode = true; editingName = name;
    document.getElementById('projName').value = name;
    document.getElementById('projDesc').value = desc;
    modalTitle.textContent       = 'Edit Project';
    btnCreate.innerHTML          = 'Save Changes';
    mapSourceGroup.style.display = 'none';   // not relevant when editing
    document.getElementById('uploadSection').classList.remove('show');
    setStatus('', '');
    modalOverlay.classList.add('show');
    setTimeout(() => document.getElementById('projName').focus(), 150);
}

function closeModal() {
    modalOverlay.classList.remove('show');
}

document.getElementById('btnNewProject').addEventListener('click', openModal);
document.getElementById('btnNewProjectEmpty').addEventListener('click', openModal);
document.getElementById('btnCancelModal').addEventListener('click', closeModal);
document.getElementById('modalClose').addEventListener('click', closeModal);
modalOverlay.addEventListener('click', (e) => { if (e.target === modalOverlay) closeModal(); });

// If ?new=1 in URL, auto-open modal (e.g. from app.js "New Project")
if (new URLSearchParams(location.search).has('new')) openModal();

// Source option selection
document.querySelectorAll('.source-opt').forEach(opt => {
    opt.addEventListener('click', () => {
        document.querySelectorAll('.source-opt').forEach(o => o.classList.remove('selected'));
        opt.classList.add('selected');
        selectedSource = opt.dataset.source;
        const uploadSection = document.getElementById('uploadSection');
        if (selectedSource === 'upload') {
            uploadSection.classList.add('show');
        } else {
            uploadSection.classList.remove('show');
        }
        setStatus('', '');
    });
});

// File picker
document.getElementById('btnChooseFile').addEventListener('click', () => {
    document.getElementById('fileInput').click();
});
document.getElementById('fileInput').addEventListener('change', (e) => {
    const file = e.target.files[0];
    if (!file) return;
    chosenFile = file;
    document.getElementById('fileNameDisplay').textContent = file.name;
    e.target.value = '';
});

// ── Create project ────────────────────────────────────────────────────────────
document.getElementById('btnCreate').addEventListener('click', handleCreate);
document.addEventListener('keydown', (e) => {
    if (e.key === 'Enter' && modalOverlay.classList.contains('show')) handleCreate();
    if (e.key === 'Escape' && modalOverlay.classList.contains('show')) closeModal();
});

async function handleCreate() {
    const name = document.getElementById('projName').value.trim();
    const desc = document.getElementById('projDesc').value.trim();

    if (!name) {
        document.getElementById('projName').focus();
        setStatus('Please enter a project name.', 'err');
        return;
    }

    // ── Edit mode ──────────────────────────────────────────────────────────────
    if (editMode) {
        btnCreate.disabled = true;
        setStatus('<span class="spinner"></span>Saving…', '');
        try {
            await updateProject(editingName, name, desc);
            if (sessionStorage.getItem('currentProject') === editingName) {
                sessionStorage.setItem('currentProject', name);
            }
            closeModal();
            loadAndRender();
        } catch (err) {
            setStatus('Error: ' + err.message, 'err');
            btnCreate.disabled = false;
        }
        return;
    }

    // ── Create mode ────────────────────────────────────────────────────────────
    if (selectedSource === 'upload' && !chosenFile) {
        setStatus('Please choose an OSM file.', 'err');
        return;
    }

    const btn = document.getElementById('btnCreate');
    btn.disabled = true;

    try {
        // 1. Create project directory + meta.json
        setStatus('<span class="spinner"></span>Creating project…', '');
        await createProject(name, desc);

        // 2. Handle map source
        if (selectedSource === 'upload') {
            setStatus('<span class="spinner"></span>Uploading map data…', '');
            const xml = await chosenFile.text();
            if (!xml.includes('<osm')) throw new Error('Not a valid OSM file');
            await saveProjectOsm(name, xml);
            // Also load it into the simulation engine via /map/reload
            const r = await fetch('/map/reload', {
                method: 'POST', headers: { 'Content-Type': 'text/xml' }, body: xml,
            });
            const j = await r.json();
            if (!r.ok) throw new Error(j.error || 'Map load failed');

        } else if (selectedSource === 'select') {
            // Redirect to map selector with project name so it can save back
            sessionStorage.setItem('currentProject', name);
            window.location.href = `/map-selector.html?project=${encodeURIComponent(name)}`;
            return;

        } else {
            // Default map: load via /map/reset
            setStatus('<span class="spinner"></span>Loading default map…', '');
            const r = await fetch('/map/reset', { method: 'POST' });
            const j = await r.json();
            if (!r.ok) throw new Error(j.error || 'Map reset failed');
        }

        sessionStorage.setItem('currentProject', name);
        window.location.href = '/loading.html';

    } catch (err) {
        setStatus('Error: ' + err.message, 'err');
        btn.disabled = false;
    }
}

function setStatus(html, cls) {
    const el = document.getElementById('modalStatus');
    el.innerHTML = html;
    el.className = cls;
}

// ── Initial load ──────────────────────────────────────────────────────────────
loadAndRender();
