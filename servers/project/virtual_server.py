import sys
import os
import threading
import time
import argparse
import queue
import yaml
from tasks.project.packages.optimal_path import dijkstra

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, '..', '..')
sys.path.insert(0, project_root)

from flask import Flask, Response, request, jsonify
import cv2
import numpy as np

from servers.templates.project import get_template as HTML_TEMPLATE
from duckiebot.wheel_driver.godot_wheels_driver import GodotWheelsDriver
from duckiebot.wheel_driver.wheels_driver_abs import WheelPWMConfiguration
from duckiebot.camera_driver.godot_camera_driver import GodotCameraDriver, GodotCameraConfig
from launcher.ports import find_available_port
from servers.common import shutdown_cleanup, suppress_http_logs
from tasks.introduction.packages import manual_drive
from tasks.visual_lane_servoing.packages.agent import LaneServoingAgent
from servers.visual_lane_servoing.visualization import create_lane_visualization
from tasks.project.packages.detection_agent import ObjectDetectionAgent, CLASS_NAMES
from servers.project.detection_visualization import draw_detections

app = Flask(__name__)

camera = None
wheels = None
keys_pressed = {'up': False, 'down': False, 'left': False, 'right': False}
keys_lock = threading.Lock()
_keys_last_update = time.time()
current_speeds = {'left': 0.0, 'right': 0.0}
stop_event = threading.Event()
student_code_works = True
maneuver_thread = None
maneuver_stop = threading.Event()
current_node = 1
goal_node = 3
_manual_mode = True
_navigation_thread = None
_navigation_stop = threading.Event()

LANE_HSV_CONFIG_FILE = os.path.join(project_root, 'config', 'lane_servoing_hsv_config.yaml')

det_agent = None
_frame_queue = queue.Queue(maxsize=1)
_last_detections = []
_detection_lock = threading.Lock()


def detection_loop():
    global _last_detections
    while not stop_event.is_set():
        if det_agent is None or not det_agent.model_loaded:
            time.sleep(0.1)
            continue
        try:
            frame_rgb = _frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        result = det_agent.detect(frame_rgb)
        if result is not None:
            with _detection_lock:
                _last_detections = result

def _get_student_module():
    from tasks.visual_lane_servoing.packages import visual_servoing_activity
    return visual_servoing_activity

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


def control_loop():
    global keys_pressed, current_speeds, student_code_works, _keys_last_update

    print("[ControlLoop] Starting...")

    while not stop_event.is_set():
        try:
            if time.time() - _keys_last_update > 0.5:
                with keys_lock:
                    keys_pressed = {'up': False, 'down': False, 'left': False, 'right': False}

            with keys_lock:
                keys_copy = keys_pressed.copy()

            try:
                left, right = manual_drive.get_motor_speeds(keys_copy)
                student_code_works = True
            except Exception as e:
                print(f"[ControlLoop] Student code error: {e}")
                left, right = 0.0, 0.0
                student_code_works = False

            current_speeds['left'] = left
            current_speeds['right'] = right

            if _manual_mode and wheels:
                wheels.set_wheels_speed(left, right)

            time.sleep(0.05)

        except Exception as e:
            print(f"[ControlLoop] Error: {e}")
            time.sleep(0.1)

    print("[ControlLoop] Stopped")


def start_navigation():
    """Start the autonomous navigation thread."""
    global _navigation_thread, _navigation_stop
    import tasks.project.packages.agent as agent

    if _navigation_thread and _navigation_thread.is_alive():
        print("[Navigation] Already running")
        return

    print("[Navigation] Starting navigation loop...")
    _navigation_stop.clear()
    import servers.project.virtual_server as _self
    _navigation_thread = threading.Thread(
        target=agent.main,
        args=(camera, wheels, None, _navigation_stop, _self),
        daemon=True,
        name='NavigationThread'
    )
    _navigation_thread.start()


def stop_navigation():
    """Stop the autonomous navigation thread."""
    global _navigation_thread, _navigation_stop

    if not _navigation_thread or not _navigation_thread.is_alive():
        print("[Navigation] Not running")
        return

    print("[Navigation] Stopping navigation loop...")
    _navigation_stop.set()
    _navigation_thread.join(timeout=2.0)
    if _navigation_thread.is_alive():
        print("[Navigation] Warning: Navigation thread still alive after timeout")
    else:
        print("[Navigation] Navigation stopped")


_DANCE_COLORS = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 1.0],
    [1.0, 0.0, 1.0],
]


def _set_leds(colors_by_index: dict):
    for idx, color in colors_by_index.items():
        if idx in _virtual_led_states:
            _virtual_led_states[idx] = color


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
    led_indices = [0, 2, 3, 4]

    while not stop_ev.is_set() and time.time() < end_time:
        if step % 2 == 0:
            left, right = 0.8, -0.8
        else:
            left, right = -0.8, 0.8

        if wheels:
            wheels.set_wheels_speed(left, right)

        new_states = {}
        for i, led_idx in enumerate(led_indices):
            color_idx = (step + i) % len(_DANCE_COLORS)
            new_states[led_idx] = _DANCE_COLORS[color_idx]
        _set_leds(new_states)

        time.sleep(0.1)
        step += 1

    if wheels:
        wheels.set_wheels_speed(0.0, 0.0)
    _set_leds({idx: [0, 0, 0] for idx in led_indices})
    print("[Dance] Done")

_vls_agent = LaneServoingAgent()

def create_visualization(frame):
    global current_speeds, keys_pressed, student_code_works

    if frame is None:
        placeholder = np.zeros((240, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Waiting for Godot...", (200, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
        return placeholder

    display = frame.copy()
    h, w = display.shape[:2]
    display_w = 640
    display_h = int(h * display_w / w)
    display = cv2.resize(display, (display_w, display_h))

    font = cv2.FONT_HERSHEY_SIMPLEX
    speed_text = f"L: {current_speeds['left']:+.2f}  R: {current_speeds['right']:+.2f}"
    cv2.putText(display, speed_text, (10, display_h - 10), font, 0.6, (0, 255, 0), 2)

    mode_text = "MANUAL" if _manual_mode else "NAV"
    color = (0, 255, 0) if _manual_mode else (255, 165, 0)
    cv2.putText(display, mode_text, (10, 25), font, 0.7, color, 2)

    with keys_lock:
        kc = keys_pressed.copy()

    key_size = 30
    gap = 4
    base_x = display_w - 3 * (key_size + gap) - 10
    base_y = display_h - 2 * (key_size + gap) - 10

    key_positions = {
        'up': (base_x + key_size + gap, base_y),
        'left': (base_x, base_y + key_size + gap),
        'down': (base_x + key_size + gap, base_y + key_size + gap),
        'right': (base_x + 2 * (key_size + gap), base_y + key_size + gap),
    }
    key_labels = {'up': '^', 'down': 'v', 'left': '<', 'right': '>'}

    for key, (kx, ky) in key_positions.items():
        color = (0, 200, 0) if kc.get(key, False) else (60, 60, 60)
        cv2.rectangle(display, (kx, ky), (kx + key_size, ky + key_size), color, -1)
        cv2.rectangle(display, (kx, ky), (kx + key_size, ky + key_size), (100, 100, 100), 1)
        cv2.putText(display, key_labels[key], (kx + 8, ky + 22), font, 0.6, (255, 255, 255), 2)

    return display

def generate_frames():
    while True:
        try:
            display = None
            if camera is not None:
                ok, raw_rgb = camera.read_rgb()
                if ok and raw_rgb is not None:
                    raw_bgr = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR)

                    if not _manual_mode:
                        import tasks.project.packages.agent as agent_mod
                        if agent_mod.debug_frame is not None:
                            display = agent_mod.debug_frame
                    else:
                        if det_agent is not None and det_agent.model_loaded:
                            try:
                                _frame_queue.put_nowait(raw_rgb)
                            except queue.Full:
                                pass

                    if display is None:
                        with _detection_lock:
                            dets = list(_last_detections)
                        frame_for_viz = raw_bgr.copy()
                        if dets:
                            draw_detections(frame_for_viz, dets)
                        _vls_agent.compute_commands(raw_rgb)
                        vis = create_lane_visualization(
                            frame_for_viz,
                            _vls_agent.last_debug_info,
                            current_speeds['left'],
                            current_speeds['right']
                        )
                        display = create_visualization(vis)

            if display is None:
                placeholder = np.zeros((240, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for Godot...", (200, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
                display = placeholder

            ret, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ret:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + jpeg.tobytes() + b'\r\n')
        except Exception as e:
            print(f"[VideoStream] Error: {e}")
        time.sleep(0.033)


@app.route('/')
def index():
    return HTML_TEMPLATE(
        title="Navigation — Project",
        subtitle="Duckiebot navigation task",
    )

@app.route('/get_hsv')
def get_hsv():
    return jsonify(_get_student_module().get_hsv_bounds())

@app.route('/update_hsv', methods=['POST'])
def update_hsv():
    data = request.json
    mod = _get_student_module()
    current = mod.get_hsv_bounds()
    current.update({k: int(v) for k, v in data.items()})
    mod.set_hsv_bounds(
        [current['yellow_lower_h'], current['yellow_lower_s'], current['yellow_lower_v']],
        [current['yellow_upper_h'], current['yellow_upper_s'], current['yellow_upper_v']],
        [current['white_lower_h'],  current['white_lower_s'],  current['white_lower_v']],
        [current['white_upper_h'],  current['white_upper_s'],  current['white_upper_v']],
    )
    try:
        with open(LANE_HSV_CONFIG_FILE, 'w') as f:
            yaml.dump(current, f, default_flow_style=False)
    except Exception as e:
        print(f"[Project] Could not save HSV config: {e}")
    return jsonify({'status': 'ok'})

@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/keys', methods=['POST'])
def update_keys():
    global keys_pressed, _keys_last_update
    data = request.json
    with keys_lock:
        keys_pressed = {
            'up': bool(data.get('up', False)),
            'down': bool(data.get('down', False)),
            'left': bool(data.get('left', False)),
            'right': bool(data.get('right', False)),
        }
    _keys_last_update = time.time()
    return jsonify({'status': 'ok',
                    'left': current_speeds['left'],
                    'right': current_speeds['right']})


@app.route('/speeds')
def get_speeds():
    return jsonify(current_speeds)


@app.route('/set_mode', methods=['POST'])
def set_mode():
    global _manual_mode
    _manual_mode = bool(request.json.get('manual', False))
    if not _manual_mode and wheels:
        wheels.set_wheels_speed(0.0, 0.0)

    if _manual_mode:
        print(f"[Mode] Manual Drive - stopping navigation")
        stop_navigation()
    else:
        print(f"[Mode] Autonomous Navigation - starting agent")
        start_navigation()

    return jsonify({'status': 'ok', 'manual': _manual_mode})


@app.route('/wheels', methods=['POST'])
def set_wheels():
    data = request.json
    left = max(-1.0, min(1.0, float(data.get('left', 0.0))))
    right = max(-1.0, min(1.0, float(data.get('right', 0.0))))
    if wheels:
        wheels.set_wheels_speed(left, right)
    return jsonify({'status': 'ok', 'left': left, 'right': right})


@app.route('/snapshot')
def snapshot():
    if camera is None:
        return jsonify({'status': 'error', 'message': 'Camera not ready'}), 503

    success, frame = camera.read_rgb()
    if not success or frame is None:
        return jsonify({'status': 'error', 'message': 'No frame available'}), 503

    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    ret, jpeg = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ret:
        return jsonify({'status': 'error', 'message': 'Encode failed'}), 500

    return Response(jpeg.tobytes(), mimetype='image/jpeg')


_virtual_led_states = {0: [0,0,0], 2: [0,0,0], 3: [0,0,0], 4: [0,0,0]}

@app.route('/leds', methods=['POST'])
def set_led():
    data = request.json
    led_index = int(data.get('led', 0))
    color = [max(0.0, min(1.0, float(c))) for c in data.get('color', [0,0,0])]
    if led_index in _virtual_led_states:
        _virtual_led_states[led_index] = color
    return jsonify({'status': 'ok', 'led': led_index, 'color': color})

@app.route('/leds/all', methods=['POST'])
def set_all_leds():
    color = [max(0.0, min(1.0, float(c))) for c in request.json.get('color', [0,0,0])]
    for idx in (0, 2, 3, 4):
        _virtual_led_states[idx] = color[:]
    return jsonify({'status': 'ok', 'color': color})

@app.route('/leds/off', methods=['POST'])
def leds_off():
    for idx in (0, 2, 3, 4):
        _virtual_led_states[idx] = [0, 0, 0]
    return jsonify({'status': 'ok'})

@app.route('/leds/state')
def get_led_state():
    return jsonify(_virtual_led_states)


@app.route('/maneuver', methods=['POST'])
def run_maneuver():
    data = request.json
    mtype = data.get('type', '')
    value = float(data.get('value', 0.5))

    if mtype == 'dance':
        distance = float(np.clip(value, 3.0, 10.0))
        start_maneuver(dance, distance)
        return jsonify({'status': 'ok', 'maneuver': 'dance', 'distance': distance})

    return jsonify({'status': 'error', 'message': 'Unknown maneuver'}), 400


@app.route('/set_start', methods=['POST'])
def set_start():
    global current_node
    current_node = int(request.json['node'])
    return jsonify({'status': 'ok', 'node': current_node})

@app.route('/get_start')
def get_start():
    return jsonify({'node': current_node})

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
        'status': 'ok',
        'node': goal_node,
        'path': route['path'],
        'distance': route['distance'] if route['path'] else None
    })


@app.route('/route')
def route():
    result = dijkstra(current_node, goal_node)
    if not result['path']:
        result['distance'] = None
    return jsonify(result)


@app.route('/get_goal')
def get_goal():
    return jsonify({'node': goal_node})


@app.route('/set_threshold', methods=['POST'])
def set_threshold():
    value = request.json.get('value') if request.json else None
    if det_agent and value is not None:
        det_agent.conf_threshold = float(value)
    return jsonify({'conf_threshold': det_agent.conf_threshold if det_agent else None})


@app.route('/status')
def status():
    with _detection_lock:
        dets = list(_last_detections)
    return jsonify({
        'current_node': current_node,
        'goal_node': goal_node,
        'left_speed': round(current_speeds['left'], 2),
        'right_speed': round(current_speeds['right'], 2),
        'mode': 'manual' if _manual_mode else 'navigation',
        'model_loaded': det_agent.model_loaded if det_agent else False,
        'conf_threshold': det_agent.conf_threshold if det_agent else 0.5,
        'detections': [
            {'class': CLASS_NAMES.get(c, str(c)), 'score': round(s, 3), 'bbox': list(b)}
            for b, s, c in dets
        ],
    })


def main():
    global camera, wheels, stop_event

    ap = argparse.ArgumentParser(description="Navigation Project Server")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--frame-port", type=int, default=5001)
    ap.add_argument("--wheel-port", type=int, default=5002)
    ap.add_argument("--godot-host", type=str, default="localhost")
    args = ap.parse_args()

    suppress_http_logs()
    print("=" * 60)
    print("NAVIGATION PROJECT (SIMULATION)")
    print("=" * 60)

    print("\n[1/3] Initializing wheels driver...")
    left_cfg = WheelPWMConfiguration()
    right_cfg = WheelPWMConfiguration()
    wheels = GodotWheelsDriver(
        left_cfg,
        right_cfg,
        godot_host=args.godot_host,
        godot_port=args.wheel_port,
    )
    wheels.trim = 0
    print(f"  Wheels: {args.godot_host}:{args.wheel_port}")

    print("\n[2/3] Initializing camera driver...")
    camera_cfg = GodotCameraConfig(host="0.0.0.0", port=args.frame_port)
    camera = GodotCameraDriver(godot_config=camera_cfg)
    camera.start()
    print(f"  Camera: connected!")

    stop_event.clear()
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    global det_agent
    print("\n[3/3] Loading detection model...")
    det_agent = ObjectDetectionAgent()
    if det_agent.model_loaded:
        print(f"  Detection model ready ({det_agent.img_size}px)")
    else:
        print(f"  Detection model: {det_agent.load_error or 'not available'}")
    threading.Thread(target=detection_loop, daemon=True).start()

    web_port = find_available_port(args.port)
    if web_port != args.port:
        print(f"  Port {args.port} busy, using {web_port}")

    print("\n" + "=" * 60)
    print(f"Web Interface: http://localhost:{web_port}")
    print("=" * 60 + "\n")

    try:
        _manual_mode = True
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
        print("[Mode] Starting in Manual mode")
        app.run(host='127.0.0.1', port=web_port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        shutdown_cleanup(wheels, camera, stop_event)


if __name__ == "__main__":
    sys.exit(main())