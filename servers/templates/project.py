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
'''

_EXTRA_JS = '''
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
    postJSON('/maneuver', {type: 'dance'})
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
});

fetch('/get_goal').then(r => r.json()).then(d => {
    document.getElementById('goalNode').value = d.node;
});

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