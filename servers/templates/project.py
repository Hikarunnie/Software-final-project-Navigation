from .base import render_template

_CONTENT = '''
    <div class="container">
        <div class="video-section">
            <img src="/video" class="stream" id="videoStream">
        </div>

        <div class="controls-section">

            <div class="card">
                <div class="card-header">
                    Status
                    <span id="statusDot" style="width:8px;height:8px;border-radius:50%;
                        background:var(--accent-green);display:inline-block;"></span>
                </div>
                <div id="statusTable" style="font-size:12px;">
                    <div style="color:var(--text-muted);text-align:center;padding:12px 0;">
                        Waiting for data...
                    </div>
                </div>
            </div>

            <div class="card">
                <div class="card-header">Drive</div>
                <div class="key-display">
                    <div class="key-box key-up"    id="key-up">&#9650;</div>
                    <div class="key-box key-left"  id="key-left">&#9664;</div>
                    <div class="key-box key-down"  id="key-down">&#9660;</div>
                    <div class="key-box key-right" id="key-right">&#9654;</div>
                </div>
                <p style="text-align:center;font-size:11px;color:var(--text-muted)">Arrow keys or WASD</p>
            </div>

            <div class="card">
                <div class="card-header">Start Node</div>
                <div style="display:flex;gap:8px;align-items:center;">
                    <select id="startNode" style="flex:1;padding:6px 8px;background:var(--bg-sidebar);
                           border:1px solid var(--border-color);border-radius:4px;color:var(--text-primary);">
                        <option value="1">Node 1</option>
                        <option value="2">Node 2</option>
                        <option value="3">Node 3</option>
                    </select>
                    <button class="button" onclick="setStartNode()">Set</button>
                </div>
                <div id="startStatus" class="status"></div>
            </div>

            <div class="card">
                <div class="card-header">End Node</div>
                <div style="display:flex;gap:8px;align-items:center;">
                    <select id="goalNode" style="flex:1;padding:6px 8px;background:var(--bg-sidebar);
                           border:1px solid var(--border-color);border-radius:4px;color:var(--text-primary);">
                        <option value="1">Node 1</option>
                        <option value="2">Node 2</option>
                        <option value="3">Node 3</option>
                    </select>
                    <button class="button" onclick="setGoalNode()">Set</button>
                </div>
                <div id="goalStatus" class="status"></div>
            </div>

            <div class="card">
                <div class="card-header">Dance Maneuver</div>
                <div style="display:flex;flex-direction:column;gap:8px;">
                    <button class="button" onclick="sendDance()">Dance</button>
                    <div id="danceStatus" class="status"></div>
                </div>
            </div>

        </div>
    </div>
'''

_EXTRA_CSS = '''
#statusTable .row {
    display: flex;
    justify-content: space-between;
    padding: 6px 0;
    border-bottom: 1px solid var(--border-color);
    align-items: baseline;
}
#statusTable .row:last-child { border-bottom: none; }
#statusTable .key  { color: var(--text-secondary); font-size: 12px; }
#statusTable .val  { color: var(--text-primary);   font-weight: 500; font-size: 13px; font-family: monospace; }

.key-display {
    display: grid;
    grid-template-areas: ".    up   ." "left down right";
    grid-template-columns: repeat(3, 48px);
    grid-template-rows: repeat(2, 48px);
    gap: 4px;
    justify-content: center;
    margin: 8px 0;
}
.key-box {
    display: flex;
    align-items: center;
    justify-content: center;
    background: var(--bg-sidebar);
    border: 2px solid var(--border-color);
    border-radius: 8px;
    font-size: 20px;
    font-weight: 600;
    color: var(--text-muted);
    transition: all 0.1s;
    user-select: none;
}
.key-box.active { background: rgba(63,185,80,0.2); border-color: var(--accent-green); color: var(--accent-green); }
.key-up    { grid-area: up; }
.key-down  { grid-area: down; }
.key-left  { grid-area: left; }
.key-right { grid-area: right; }
'''

_EXTRA_JS = '''
const keyState = {up: false, down: false, left: false, right: false};
const keyMap = {
    'ArrowUp': 'up', 'ArrowDown': 'down', 'ArrowLeft': 'left', 'ArrowRight': 'right',
    'w': 'up', 's': 'down', 'a': 'left', 'd': 'right',
    'W': 'up', 'S': 'down', 'A': 'left', 'D': 'right',
};

function updateKeyDisplay() {
    for (const [key, active] of Object.entries(keyState)) {
        const el = document.getElementById('key-' + key);
        if (el) el.classList.toggle('active', active);
    }
}

function sendKeys() {
    fetch('/keys', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(keyState)})
        .catch(() => {});
}

function releaseAll() {
    Object.keys(keyState).forEach(k => keyState[k] = false);
    updateKeyDisplay();
    sendKeys();
}

document.addEventListener('keydown', e => {
    const dir = keyMap[e.key];
    if (dir && !keyState[dir]) { e.preventDefault(); keyState[dir] = true; updateKeyDisplay(); sendKeys(); }
});
document.addEventListener('keyup', e => {
    const dir = keyMap[e.key];
    if (dir) { e.preventDefault(); keyState[dir] = false; updateKeyDisplay(); sendKeys(); }
});
window.addEventListener('blur', releaseAll);
setInterval(() => { if (Object.values(keyState).some(Boolean)) sendKeys(); }, 150);

function refreshStatus() {
    fetch('/status')
        .then(r => r.json())
        .then(data => {
            const table = document.getElementById('statusTable');
            const keys = Object.keys(data);
            if (keys.length === 0) {
                table.innerHTML = '<div style="color:var(--text-muted);text-align:center;padding:12px 0;">get_ui_data() returned {}</div>';
                return;
            }
            table.innerHTML = keys.map(k =>
                `<div class="row">
                    <span class="key">${k}</span>
                    <span class="val">${JSON.stringify(data[k])}</span>
                </div>`
            ).join('');
            document.getElementById('statusDot').style.background = 'var(--accent-green)';
        })
        .catch(() => {
            document.getElementById('statusDot').style.background = 'var(--accent-red)';
        });
}

function sendDance() {
    postJSON('/maneuver', {type: 'dance', value: 3.0})
        .then(r => showStatus('danceStatus', r.status === 'ok' ? 'Dance started' : (r.message || 'Error'), r.status === 'ok' ? 'success' : 'error'))
        .catch(e => showStatus('danceStatus', 'Error: ' + e, 'error'));
}

function setStartNode() {
    const node = parseInt(document.getElementById('startNode').value);
    postJSON('/set_start', {node})
        .then(r => showStatus('startStatus', 'Start node set to ' + r.node, 'success'))
        .catch(e => showStatus('startStatus', 'Error: ' + e, 'error'));
}

function setGoalNode() {
    const node = parseInt(document.getElementById('goalNode').value);
    postJSON('/set_goal', {node})
        .then(r => showStatus('goalStatus', 'End node set to ' + r.node, 'success'))
        .catch(e => showStatus('goalStatus', 'Error: ' + e, 'error'));
}

fetch('/get_start').then(r => r.json()).then(d => {
    document.getElementById('startNode').value = d.node;
}).catch(() => {});

fetch('/get_goal').then(r => r.json()).then(d => {
    document.getElementById('goalNode').value = d.node;
}).catch(() => {});

refreshStatus();
setInterval(refreshStatus, 500);
'''


def get_template(title='Project', subtitle='Real Duckiebot'):
    return render_template(
        title=title,
        subtitle=subtitle,
        content_html=_CONTENT,
        extra_css=_EXTRA_CSS,
        extra_js=_EXTRA_JS,
    )

PROJECT_TEMPLATE = get_template()