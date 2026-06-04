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
    <div class="card-header">HSV Calibration</div>
    <div style="font-size:12px;color:#f1c40f;font-weight:600;margin-bottom:8px;">YELLOW LINE</div>
    <div id="hsv-sliders-yellow"></div>
    <div style="font-size:12px;color:#ecf0f1;font-weight:600;margin:12px 0 8px;">WHITE LINE</div>
    <div id="hsv-sliders-white"></div>
    <div id="hsv-status" class="status"></div>
</div>

            <div class="card">
                <div class="card-header">Mode</div>
                <div style="display:flex;align-items:center;gap:12px;padding:4px 0;">
                    <span style="font-size:13px;color:var(--text-secondary);">Navigation</span>
                    <label style="position:relative;display:inline-block;width:48px;height:26px;">
                        <input type="checkbox" id="driveToggle" onchange="toggleMode(this.checked)"
                            style="opacity:0;width:0;height:0;">
                        <span id="toggleSlider" style="position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;
                            background:var(--bg-sidebar);border:2px solid var(--border-color);border-radius:26px;
                            transition:.3s;">
                            <span style="position:absolute;content:'';height:18px;width:18px;left:2px;bottom:2px;
                                background:var(--text-muted);border-radius:50%;transition:.3s;display:block;"
                                id="toggleKnob"></span>
                        </span>
                    </label>
                    <span style="font-size:13px;color:var(--text-secondary);">Manual Drive</span>
                </div>
                <div id="modeStatus" style="font-size:12px;color:var(--text-muted);margin-top:4px;">Mode: Navigation</div>
            </div>

            <div class="card" id="driveCard" style="display:none;">
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
let manualMode = false;

const keyState = {up: false, down: false, left: false, right: false};
const keyMap = {
    'ArrowUp': 'up', 'ArrowDown': 'down', 'ArrowLeft': 'left', 'ArrowRight': 'right',
    'w': 'up', 's': 'down', 'a': 'left', 'd': 'right',
    'W': 'up', 'S': 'down', 'A': 'left', 'D': 'right',
};
const hsvKeys = {
    yellow: ['yellow_lower_h','yellow_upper_h','yellow_lower_s','yellow_upper_s','yellow_lower_v','yellow_upper_v'],
    white:  ['white_lower_h', 'white_upper_h', 'white_lower_s', 'white_upper_s', 'white_lower_v', 'white_upper_v'],
};
const hsvRanges = {h: 179, s: 255, v: 255};
function postJSON(url, data) {
    return fetch(url, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(data)
    }).then(r => r.json());
}

function showStatus(id, msg, type) {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = msg;
    el.style.color = type === 'success' ? 'var(--accent-green)' : 'var(--accent-red)';
    setTimeout(() => { el.textContent = ''; }, 3000);
}
function buildHsvSliders() {
    ['yellow','white'].forEach(color => {
        const container = document.getElementById('hsv-sliders-' + color);
        hsvKeys[color].forEach(key => {
            const label = key.replace(color + '_', '').replace('_', ' ').toUpperCase();
            const letter = key.slice(-1);
            const max = hsvRanges[letter] || 255;
            
            const wrapper = document.createElement('div');
            wrapper.style.marginBottom = '6px';
            
            const labelRow = document.createElement('div');
            labelRow.style.cssText = 'display:flex;justify-content:space-between;font-size:11px;color:var(--text-muted);';
            labelRow.innerHTML = '<span>' + label + '</span><span id="' + key + '-val">0</span>';
            
            const slider = document.createElement('input');
            slider.type = 'range';
            slider.id = key;
            slider.min = 0;
            slider.max = max;
            slider.value = 0;
            slider.style.width = '100%';
            slider.addEventListener('input', function() {
                document.getElementById(key + '-val').textContent = this.value;
                console.log('Slider moved:', key, this.value);
                sendHsv(key, this.value);
            });
            
            wrapper.appendChild(labelRow);
            wrapper.appendChild(slider);
            container.appendChild(wrapper);
        });
    });
}

function sendHsv(key, val) {
    const payload = {};
    payload[key] = parseInt(val);
    postJSON('/update_hsv', payload)
        .then(r => {
            console.log('HSV update response:', r);
            showStatus('hsv-status', 'Updated', 'success');
        })
        .catch(e => {
            console.log('HSV update error:', e);
            showStatus('hsv-status', 'Error', 'error');
        });
}

buildHsvSliders();

fetch('/get_hsv').then(r => r.json()).then(d => {
    Object.entries(d).forEach(([k, v]) => {
        const el = document.getElementById(k);
        if (el) { el.value = v; document.getElementById(k + '-val').textContent = v; }
    });
}).catch(() => {});
function toggleMode(isManual) {
    manualMode = isManual;
    document.getElementById('driveCard').style.display = isManual ? 'block' : 'none';
    document.getElementById('modeStatus').textContent = 'Mode: ' + (isManual ? 'Manual Drive' : 'Navigation');
    document.getElementById('toggleKnob').style.left = isManual ? '26px' : '2px';
    document.getElementById('toggleSlider').style.background = isManual ? 'rgba(63,185,80,0.3)' : 'var(--bg-sidebar)';
    document.getElementById('toggleSlider').style.borderColor = isManual ? 'var(--accent-green)' : 'var(--border-color)';
    document.getElementById('toggleKnob').style.background = isManual ? 'var(--accent-green)' : 'var(--text-muted)';

    postJSON('/set_mode', {manual: isManual}).catch(() => {});
    if (!isManual) releaseAll();
}

function updateKeyDisplay() {
    for (const [key, active] of Object.entries(keyState)) {
        const el = document.getElementById('key-' + key);
        if (el) el.classList.toggle('active', active);
    }
}

function sendKeys() {
    if (!manualMode) return;
    fetch('/keys', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(keyState)})
        .catch(() => {});
}

function releaseAll() {
    Object.keys(keyState).forEach(k => keyState[k] = false);
    updateKeyDisplay();
    fetch('/keys', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(keyState)})
        .catch(() => {});
}

document.addEventListener('keydown', e => {
    if (!manualMode) return;
    const dir = keyMap[e.key];
    if (dir && !keyState[dir]) { e.preventDefault(); keyState[dir] = true; updateKeyDisplay(); sendKeys(); }
});
document.addEventListener('keyup', e => {
    if (!manualMode) return;
    const dir = keyMap[e.key];
    if (dir) { e.preventDefault(); keyState[dir] = false; updateKeyDisplay(); sendKeys(); }
});
window.addEventListener('blur', releaseAll);
setInterval(() => { if (manualMode && Object.values(keyState).some(Boolean)) sendKeys(); }, 150);

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