import sys
import os
import signal
import threading
import argparse
import time

script_dir   = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, '..', '..')
sys.path.insert(0, project_root)

from flask import Flask, Response, jsonify, request
import numpy as np
import cv2

from duckiebot.camera_driver import CameraDriver
from duckiebot.wheel_driver import DaguWheelsDriver
from duckiebot.wheel_driver.wheels_driver_abs import WheelPWMConfiguration
from duckiebot.led_driver import LEDDriver
from launcher.ports import find_available_port
from servers.common import shutdown_cleanup, suppress_http_logs, make_frame_generator
from servers.templates.project import get_template as HTML_TEMPLATE
from tasks.project.packages.optimal_path import dijkstra

app        = Flask(__name__)
camera     = None
wheels     = None
leds       = None
stop_event = threading.Event()

current_node = 1
goal_node    = 3
_manual_mode = True
_navigation_thread = None
_navigation_stop   = threading.Event()
keys_pressed = {'up': False, 'down': False, 'left': False, 'right': False}
keys_lock    = threading.Lock()
_keys_last_update = time.time()
current_speeds = {'left': 0.0, 'right': 0.0}

# ── Motor trim ────────────────────────────────────────────────────────────────
# Right motor is weaker, so we boost it relative to left.
# Positive = boost right, negative = boost left.  Start at 0.05, tune via
# POST /set_trim {"trim": 0.10} while driving straight in manual mode.
_trim = 0.05


def _set_wheels(left: float, right: float):
    """Apply trim correction then forward to the real wheel driver."""
    global current_speeds
    if wheels is None:
        return
    # Multiplicative trim keeps the effect proportional at all speeds.
    r_trimmed = float(np.clip(right * (1.0 + _trim), -1.0, 1.0))
    current_speeds['left']  = left
    current_speeds['right'] = right   # store un-trimmed for status display
    wheels.set_wheels_speed(left, r_trimmed)


_DANCE_COLORS = [
    [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
    [1.0, 1.0, 0.0], [0.0, 1.0, 1.0], [1.0, 0.0, 1.0],
]
maneuver_thread = None
maneuver_stop   = threading.Event()


def start_maneuver(fn, *args):
    global maneuver_thread, maneuver_stop
    maneuver_stop.set()
    if maneuver_thread and maneuver_thread.is_alive():
        maneuver_thread.join(timeout=1.5)
    maneuver_stop = threading.Event()
    maneuver_thread = threading.Thread(
        target=fn, args=(*args, maneuver_stop), daemon=True
    )
    maneuver_thread.start()


def dance(duration_sec, stop_ev):
    print(f"[Dance] Starting for {duration_sec:.1f}s")
    duration = float(np.clip(duration_sec, 0.5, 10.0))

    if wheels:
        _set_wheels(0.8, -0.8)
        time.sleep(0.6)
        _set_wheels(0.8, 0.8)
        time.sleep(1.0)
        _set_wheels(0.0, 0.0)
        time.sleep(0.1)

    end_time = time.time() + duration
    step = 0
    while not stop_ev.is_set() and time.time() < end_time:
        if step % 2 == 0:
            left, right = 0.8, -0.8
        else:
            left, right = -0.8, 0.8
        if wheels:
            _set_wheels(left, right)
        if leds:
            try:
                color = _DANCE_COLORS[step % len(_DANCE_COLORS)]
                leds.set_led(0, color)
            except Exception:
                pass
        time.sleep(0.1)
        step += 1

    if wheels:
        _set_wheels(0.0, 0.0)
    print("[Dance] Done")


# ── Video stream ──────────────────────────────────────────────────────────────

def _visualize_frame(frame_bgr):
    """
    Called by make_frame_generator for every camera frame (BGR from read()).
    - During navigation: return the agent debug overlay (also BGR).
    - Otherwise:        return the raw BGR frame directly.
    """
    import tasks.project.packages.agent as agent_mod
    if agent_mod.debug_frame is not None:
        return agent_mod.debug_frame
    return frame_bgr


# rgb=False -> make_frame_generator calls cam.read() which returns BGR directly.
generate_frames = make_frame_generator(
    get_camera=lambda: camera,
    visualize=_visualize_frame,
    quality=70,
    rgb=False,
)


# ── Navigation thread helpers ─────────────────────────────────────────────────

def start_navigation():
    global _navigation_thread, _navigation_stop
    import tasks.project.packages.agent as agent

    if _navigation_thread and _navigation_thread.is_alive():
        print("[Navigation] Already running")
        return

    print("[Navigation] Starting navigation loop...")
    _navigation_stop.clear()
    import servers.project.real_server as _self
    _navigation_thread = threading.Thread(
        target=agent.main,
        args=(camera, wheels, leds, _navigation_stop, _self),
        daemon=True,
        name='NavigationThread'
    )
    _navigation_thread.start()


def stop_navigation():
    global _navigation_thread, _navigation_stop

    if not _navigation_thread or not _navigation_thread.is_alive():
        print("[Navigation] Not running")
        return

    print("[Navigation] Stopping navigation loop...")
    _navigation_stop.set()
    _navigation_thread.join(timeout=2.0)
    print("[Navigation] Navigation stopped")


# ── Routes ────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return HTML_TEMPLATE(
        title="Navigation — Project",
        subtitle="Real Duckiebot",
    )


@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/status')
def status():
    return jsonify({
        'current_node': current_node,
        'goal_node':    goal_node,
        'left_speed':   round(current_speeds['left'], 2),
        'right_speed':  round(current_speeds['right'], 2),
        'mode':         'manual' if _manual_mode else 'navigation',
    })


@app.route('/keys', methods=['POST'])
def update_keys():
    global keys_pressed, _keys_last_update
    data = request.json
    with keys_lock:
        keys_pressed = {
            'up':    bool(data.get('up', False)),
            'down':  bool(data.get('down', False)),
            'left':  bool(data.get('left', False)),
            'right': bool(data.get('right', False)),
        }
    _keys_last_update = time.time()

    up    = keys_pressed['up']
    down  = keys_pressed['down']
    left  = keys_pressed['left']
    right = keys_pressed['right']

    if up and not down:
        l, r = 0.4, 0.4
    elif down and not up:
        l, r = -0.4, -0.4
    elif left and not right:
        l, r = -0.3, 0.3
    elif right and not left:
        l, r = 0.3, -0.3
    else:
        l, r = 0.0, 0.0

    current_speeds['left']  = l
    current_speeds['right'] = r

    if _manual_mode and wheels:
        _set_wheels(l, r)

    return jsonify({'status': 'ok', 'left': l, 'right': r})


@app.route('/set_mode', methods=['POST'])
def set_mode():
    global _manual_mode
    _manual_mode = bool(request.json.get('manual', False))
    if _manual_mode:
        print("[Mode] Manual Drive - stopping navigation")
        stop_navigation()
        if wheels:
            _set_wheels(0.0, 0.0)
    else:
        print("[Mode] Autonomous Navigation - starting agent")
        start_navigation()
    return jsonify({'status': 'ok', 'manual': _manual_mode})


@app.route('/set_start', methods=['POST'])
def set_start():
    global current_node
    current_node = int(request.json['node'])
    return jsonify({'status': 'ok', 'node': current_node})


@app.route('/get_start')
def get_start():
    return jsonify({'node': current_node})


@app.route('/get_hsv')
def get_hsv():
    from tasks.visual_lane_servoing.packages.visual_servoing_activity import get_hsv_bounds
    return jsonify(get_hsv_bounds())


@app.route('/update_hsv', methods=['POST'])
def update_hsv():
    from tasks.visual_lane_servoing.packages.visual_servoing_activity import get_hsv_bounds, set_hsv_bounds
    data = request.json
    current = get_hsv_bounds()
    current.update({k: int(v) for k, v in data.items() if k in current})
    set_hsv_bounds(
        [current['yellow_lower_h'], current['yellow_lower_s'], current['yellow_lower_v']],
        [current['yellow_upper_h'], current['yellow_upper_s'], current['yellow_upper_v']],
        [current['white_lower_h'],  current['white_lower_s'],  current['white_lower_v']],
        [current['white_upper_h'],  current['white_upper_s'],  current['white_upper_v']],
    )
    return jsonify({'status': 'ok', 'new_values': get_hsv_bounds()})


# ── Red-line HSV calibration endpoints ───────────────────────────────────────

@app.route('/get_red_hsv')
def get_red_hsv():
    from tasks.project.packages.agent import get_red_hsv as _get
    return jsonify(_get())


@app.route('/update_red_hsv', methods=['POST'])
def update_red_hsv():
    from tasks.project.packages.agent import get_red_hsv as _get, set_red_hsv as _set
    _set(request.json)
    return jsonify({'status': 'ok', 'new_values': _get()})


# ─────────────────────────────────────────────────────────────────────────────

@app.route('/set_goal', methods=['POST'])
def set_goal():
    global goal_node
    goal_node = int(request.json['node'])
    route = dijkstra(current_node, goal_node)

    print("\n===================")
    print("PATH PLANNER")
    print("===================")
    print(f"Start: {current_node}")
    print(f"Goal: {goal_node}")
    print(f"Path: {route['path']}")
    print(f"Edges: {route['edges']}")
    print(f"Distance: {route['distance']}")
    print("===================\n")

    return jsonify({
        'status':   'ok',
        'node':     goal_node,
        'path':     route['path'],
        'distance': route['distance'] if route['path'] else None,
    })


@app.route('/get_goal')
def get_goal():
    return jsonify({'node': goal_node})


@app.route('/maneuver', methods=['POST'])
def run_maneuver():
    data  = request.json
    mtype = data.get('type', '')
    value = float(data.get('value', 0.5))

    if mtype == 'dance':
        distance = float(np.clip(value, 3.0, 10.0))
        start_maneuver(dance, distance)
        return jsonify({'status': 'ok', 'maneuver': 'dance', 'distance': distance})

    return jsonify({'status': 'error', 'message': 'Unknown maneuver'}), 400


@app.route('/get_trim')
def get_trim():
    return jsonify({'trim': _trim})


@app.route('/set_trim', methods=['POST'])
def set_trim():
    global _trim
    val = float(request.json.get('trim', _trim))
    _trim = float(np.clip(val, -0.3, 0.3))
    print(f"[Trim] set to {_trim:+.3f}")
    return jsonify({'status': 'ok', 'trim': _trim})


@app.route('/speeds')
def get_speeds():
    return jsonify(current_speeds)


@app.route('/shutdown')
def shutdown():
    shutdown_cleanup(wheels, camera, stop_event)
    return jsonify({'status': 'ok'})


def main():
    global camera, wheels, leds, stop_event

    ap = argparse.ArgumentParser(description='Project Server — Real Hardware')
    ap.add_argument('--port', type=int, default=5000)
    args = ap.parse_args()

    suppress_http_logs()
    print('=' * 60)
    print('PROJECT SERVER — REAL HARDWARE')
    print('=' * 60)

    print('\n[1/4] Initializing LED driver...')
    try:
        leds = LEDDriver()
        leds.all_off()
        print('  LEDs: ok')
    except Exception as e:
        print(f'  LEDs: not available ({e})')
        leds = None

    print('\n[2/4] Initializing wheels driver...')
    wheels = DaguWheelsDriver(WheelPWMConfiguration(), WheelPWMConfiguration())
    print('  Wheels: ok')

    print('\n[3/4] Initializing camera driver...')
    camera = CameraDriver()
    camera.start()
    print('  Camera: ok')

    print('\n[4/4] Ready — use the UI to start navigation')

    def _shutdown(signum, frame):
        print('\nShutting down...')
        if leds:
            try:
                leds.all_off()
                leds.release()
            except Exception:
                pass
        shutdown_cleanup(wheels, camera, stop_event)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT,  _shutdown)

    web_port = find_available_port(args.port)
    print(f'\nWeb UI: http://10.251.244.26:{web_port}')
    print('Press Ctrl+C to stop\n')

    try:
        _manual_mode = True
        if wheels:
            _set_wheels(0.0, 0.0)
        print("[Mode] Starting in Manual mode")
        app.run(host='0.0.0.0', port=web_port, debug=False, threaded=True)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if leds:
            try:
                leds.all_off()
                leds.release()
            except Exception:
                pass
        shutdown_cleanup(wheels, camera, stop_event)


if __name__ == '__main__':
    sys.exit(main())