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
from servers.common import shutdown_cleanup, suppress_http_logs
from servers.templates.project import get_template as HTML_TEMPLATE
from tasks.project.packages.optimal_path import dijkstra

app        = Flask(__name__)
camera     = None
wheels     = None
leds       = None
stop_event = threading.Event()

current_node = 1
goal_node    = 3
_manual_mode = False
_navigation_thread = None
_navigation_stop   = threading.Event()
keys_pressed = {'up': False, 'down': False, 'left': False, 'right': False}
keys_lock    = threading.Lock()
_keys_last_update = time.time()
current_speeds = {'left': 0.0, 'right': 0.0}

_latest_frame_bgr  = None
_frame_lock        = threading.Lock()
_capture_thread    = None

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
        wheels.set_wheels_speed(0.8, -0.8)
        time.sleep(0.6)
        wheels.set_wheels_speed(0.8, 0.8)
        time.sleep(1.0)
        wheels.set_wheels_speed(0.0, 0.0)
        time.sleep(0.1)

    end_time = time.time() + duration
    step = 0
    while not stop_ev.is_set() and time.time() < end_time:
        if step % 2 == 0:
            left, right = 0.8, -0.8
        else:
            left, right = -0.8, 0.8
        if wheels:
            wheels.set_wheels_speed(left, right)
        if leds:
            try:
                color = _DANCE_COLORS[step % len(_DANCE_COLORS)]
                leds.set_led(0, color)
            except Exception:
                pass
        time.sleep(0.1)
        step += 1

    if wheels:
        wheels.set_wheels_speed(0.0, 0.0)
    print("[Dance] Done")




def generate_frames():
    import tasks.project.packages.agent as agent_mod

    _ensure_capture_thread()

    while True:
        try:
            with _frame_lock:
                frame_bgr = _latest_frame_bgr.copy() if _latest_frame_bgr is not None else None

            if frame_bgr is None:
                display = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(display, "Waiting for camera...", (160, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 80), 2)
            else:
                follower   = agent_mod.lane_follower
                debug_info = getattr(follower, 'last_debug_info', None)
                nav        = agent_mod.agent
                pwm_left   = getattr(nav, '_last_left',  0.0)
                pwm_right  = getattr(nav, '_last_right', 0.0)

                if debug_info is not None:
                    display = create_lane_visualization(
                        frame_bgr, debug_info, pwm_left, pwm_right
                    )
                else:
                    display = (agent_mod.debug_frame
                               if agent_mod.debug_frame is not None
                               else frame_bgr)

            ret, jpeg = cv2.imencode(
                '.jpg', display,
                [cv2.IMWRITE_JPEG_QUALITY, 60],   # slightly lower → smaller payload
            )
            if ret:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + jpeg.tobytes() + b'\r\n')

        except Exception as e:
            print(f"[VideoStream] Error: {e}")

        time.sleep(0.05)          # stream at ~20 Hz — plenty for a debug view

def start_navigation():
    global _navigation_thread, _navigation_stop
    import tasks.project.packages.agent as agent

    if _navigation_thread and _navigation_thread.is_alive():
        print("[Navigation] Already running")
        return

    print("[Navigation] Starting navigation loop...")
    _navigation_stop.clear()
    _navigation_thread = threading.Thread(
        target=agent.main,
        args=(camera, wheels, leds, _navigation_stop),
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
        wheels.set_wheels_speed(l, r)

    return jsonify({'status': 'ok', 'left': l, 'right': r})


@app.route('/set_mode', methods=['POST'])
def set_mode():
    global _manual_mode
    _manual_mode = bool(request.json.get('manual', False))
    if _manual_mode:
        print("[Mode] Manual Drive - stopping navigation")
        stop_navigation()
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
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


import cv2
import numpy as np


def create_lane_visualization(
    image: np.ndarray,
    debug_info: dict,
    pwm_left: float,
    pwm_right: float,
) -> np.ndarray:
    display_w = 320
    h, w = image.shape[:2]
    display_h = int(h * display_w / w)

    # Panel 1 – camera with slice overlay
    cam = cv2.resize(image, (display_w, display_h))
    scale_y = display_h / h
    scale_x = display_w / w

    for sy in debug_info.get('slice_ys', []):
        dy = int(sy * scale_y)
        cv2.line(cam, (0, dy), (display_w, dy), (0, 255, 255), 1)

    for i, x in enumerate(debug_info.get('yellow_xs', [])):
        sy_list = debug_info.get('slice_ys', [])
        if i < len(sy_list):
            dy = int(sy_list[i] * scale_y)
            dx = int(x * scale_x)
            cv2.circle(cam, (dx, dy), 5, (0, 255, 255), -1)   # yellow dot = yellow line

    for i, x in enumerate(debug_info.get('white_xs', [])):
        sy_list = debug_info.get('slice_ys', [])
        if i < len(sy_list):
            dy = int(sy_list[i] * scale_y)
            dx = int(x * scale_x)
            cv2.circle(cam, (dx, dy), 5, (255, 255, 255), -1)  # white dot = white line

    # Panel 2 – combined lane heatmap
    lane_vis  = cv2.resize(cv2.applyColorMap(debug_info['lane_mask'],  cv2.COLORMAP_HOT),  (display_w, display_h))
    white_vis = cv2.resize(cv2.applyColorMap(debug_info['white_mask'], cv2.COLORMAP_BONE), (display_w, display_h))
    # Yellow mask: yellow on black background
    ym = debug_info['yellow_mask']
    yellow_bgr = np.zeros((*ym.shape, 3), dtype=np.uint8)
    yellow_bgr[:, :, 1] = ym  # green channel
    yellow_bgr[:, :, 2] = ym  # red channel  (green + red = yellow in BGR)
    yellow_vis = cv2.resize(yellow_bgr, (display_w, display_h))

    grid = np.vstack([np.hstack([cam, lane_vis]),
                      np.hstack([white_vis, yellow_vis])])

    font = cv2.FONT_HERSHEY_SIMPLEX
    green = (0, 255, 0)
    cv2.putText(grid, "Camera",       (10,              20), font, 0.5, green, 1)
    cv2.putText(grid, "Lane Mask",    (display_w + 10,  20), font, 0.5, green, 1)
    cv2.putText(grid, "White Lines",  (10,              display_h + 20), font, 0.5, green, 1)
    cv2.putText(grid, "Yellow Lines", (display_w + 10,  display_h + 20), font, 0.5, green, 1)

    info = _info_strip(display_w * 2, debug_info, pwm_left, pwm_right)
    return np.vstack([grid, info])

def _capture_loop():
    """Runs in a daemon thread; always keeps _latest_frame_bgr fresh."""
    global _latest_frame_bgr
    while True:
        if camera is not None:
            ok, frame = camera.read()
            if ok and frame is not None:
                with _frame_lock:
                    _latest_frame_bgr = frame
        time.sleep(0.02)          # 50 Hz — matches the agent loop

def _ensure_capture_thread():
    global _capture_thread
    if _capture_thread is None or not _capture_thread.is_alive():
        _capture_thread = threading.Thread(
            target=_capture_loop, daemon=True, name='CaptureThread'
        )
        _capture_thread.start()


def _draw_bar(canvas, label, x0, y, bar_w, bar_h, value, font):
    cv2.putText(canvas, label, (10, y + 13), font, 0.5, (255, 255, 255), 1)
    cv2.rectangle(canvas, (x0, y), (x0 + bar_w, y + bar_h), (50, 50, 50), -1)
    fill = int(bar_w * np.clip(abs(value), 0, 1))
    color = (100, 100, 255) if value >= 0 else (255, 100, 100)
    cv2.rectangle(canvas, (x0, y), (x0 + fill, y + bar_h), color, -1)
    cv2.putText(canvas, f"{value:.3f}", (x0 + bar_w + 10, y + 13), font, 0.4, (200, 200, 200), 1)


def _info_strip(width, debug_info, pwm_left, pwm_right):
    h = 120
    canvas = np.zeros((h, width, 3), dtype=np.uint8)
    font   = cv2.FONT_HERSHEY_SIMPLEX
    bar_x, bar_w, bar_h = 80, 280, 20

    # Lateral error bar
    err = debug_info['lateral_error']
    cv2.putText(canvas, "Error:", (10, 25), font, 0.5, (255, 255, 255), 1)
    cv2.rectangle(canvas, (bar_x, 5), (bar_x + bar_w, 25), (50, 50, 50), -1)
    cx = bar_x + bar_w // 2
    cv2.line(canvas, (cx, 5), (cx, 25), (100, 100, 100), 1)
    ep = int(np.clip(cx + err * bar_w / 2, bar_x, bar_x + bar_w))
    ecol = (0, 255, 0) if abs(err) < 0.1 else (0, 255, 255) if abs(err) < 0.3 else (0, 0, 255)
    cv2.circle(canvas, (ep, 15), 8, ecol, -1)
    cv2.putText(canvas, f"{err:.2f}", (bar_x + bar_w + 10, 20), font, 0.4, (200, 200, 200), 1)

    # PWM bars
    _draw_bar(canvas, "Left:",  bar_x, 40, bar_w, bar_h, pwm_left,  font)
    _draw_bar(canvas, "Right:", bar_x, 70, bar_w, bar_h, pwm_right, font)

    # Status
    detected = debug_info['lane_detected']
    cv2.putText(canvas, "LANE OK" if detected else "NO LANE",
                (20, 105), font, 0.5, (0, 255, 0) if detected else (0, 0, 255), 1)
    cv2.putText(canvas, f"px:{debug_info['total_lane_pixels']}  f:{debug_info.get('frame_count',0)}",
                (300, 105), font, 0.4, (200, 200, 200), 1)

    return canvas

def _draw_steer_matrix(shape, matrix_fn, display_w, display_h):
    mat = matrix_fn(shape)
    # Normalize to 0-255 for display
    pos = np.clip(mat, 0, None)
    neg = np.clip(-mat, 0, None)
    vis = np.zeros((shape[0], shape[1], 3), dtype=np.uint8)
    vis[:, :, 2] = (pos / pos.max() * 255).astype(np.uint8) if pos.max() > 0 else 0  # red = turn right
    vis[:, :, 0] = (neg / neg.max() * 255).astype(np.uint8) if neg.max() > 0 else 0  # blue = turn left
    return cv2.resize(vis, (display_w, display_h))

if __name__ == '__main__':
    sys.exit(main())