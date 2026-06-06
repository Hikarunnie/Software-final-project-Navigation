import time
import cv2
import numpy as np
from collections import deque

from tasks.visual_lane_servoing.packages.agent import LaneServoingAgent
from tasks.project.packages.road_map import road_map
from tasks.project.packages.optimal_path import dijkstra

import sys

server      = None
debug_frame = None
agent       = None

def _log(msg):
    print(msg, flush=True)
    sys.stdout.flush()

# ============================================================================
# TUNING CONSTANTS — edit these to tune the robot
# ============================================================================

CREEP_SPEED        = 0.3   # speed while creeping over red line
FORWARD_CLEAR_TIME = 3   # seconds to creep forward before turning
TURN_SPEED         = 0.20   # rotation speed during turn
EXIT_SPEED         = 0.2  # speed after turn while searching for lane
EXIT_TIME          = 3 # seconds to drive forward after turn

TURN_TIME_FORWARD  = 3   # seconds for forward crossing
TURN_TIME_LEFT     = 0.8   # seconds to rotate left — tune this
TURN_TIME_RIGHT    = 1    # seconds to rotate right — tune this

TURN_TIMES = {
    "forward": TURN_TIME_FORWARD,
    "left":    TURN_TIME_LEFT,
    "right":   TURN_TIME_RIGHT,
}

RED_WINDOW_SIZE = 12
RED_VOTE_THRESH = 0.65
RED_ARM_FRAMES  = 20

# ============================================================================
# RED LINE DETECTION
# ============================================================================

def detect_red_line(image):
    if image is None or len(image.shape) != 3:
        return False, None
    if image.dtype != np.uint8:
        image = (np.clip(image, 0, 1) * 255).astype(np.uint8) \
                if image.max() <= 1.0 else np.clip(image, 0, 255).astype(np.uint8)

    h, w = image.shape[:2]
    roi_top = int(h * 0.55)
    roi = image[roi_top:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    roi_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0,   100, 100]), np.array([10,  255, 255])),
        cv2.inRange(hsv, np.array([165, 100, 100]), np.array([180, 255, 255])),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, kernel)

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[roi_top:, :] = roi_mask

    if int(np.count_nonzero(roi_mask)) < 150:
        return False, mask

    cols = np.where(roi_mask > 0)[1]
    rows = np.where(roi_mask > 0)[0]
    if len(cols) == 0:
        return False, mask

    span_x = int(cols.max() - cols.min()) + 1
    span_y = int(rows.max() - rows.min()) + 1
    aspect = span_x / max(span_y, 1)

    if aspect < 2.5 or span_x < int(w * 0.15):
        return False, mask

    return True, mask

# ============================================================================
# DEBUG FRAME
# ============================================================================

def build_debug_frame(raw_bgr, mask_yellow, mask_white, mask_red, state, sub, error):
    if raw_bgr is None:
        return None
    h, w = raw_bgr.shape[:2]
    panel_w, panel_h = w // 2, h // 3
    raw = raw_bgr.copy()
    label = state.upper() + (f"/{sub}" if sub else "")
    cv2.putText(raw, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    try:
        err_val = float(error) if not isinstance(error, tuple) else 0.0
    except Exception:
        err_val = 0.0
    cv2.putText(raw, f"err:{err_val:+.3f}", (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

    def _make_panel(mask, tint, label):
        panel = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
        if mask is not None:
            m = mask if mask.dtype == np.uint8 else (mask * 255).astype(np.uint8)
            r = cv2.resize(m, (panel_w, panel_h), interpolation=cv2.INTER_NEAREST)
            panel[r > 0] = tint
            cv2.putText(panel, f"{label} ({int(np.count_nonzero(m))}px)", (4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        else:
            cv2.putText(panel, f"{label} (no mask)", (4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 80, 80), 1)
        return panel

    right_col = cv2.resize(np.vstack([
        _make_panel(mask_yellow, [0, 220, 220], "YELLOW LANE"),
        _make_panel(mask_white,  [0, 220, 0],   "WHITE LANE"),
        _make_panel(mask_red,    [0, 0, 220],   "RED LINE"),
    ]), (panel_w, h))
    return np.hstack([raw, right_col])

# ============================================================================
# HEADING + DIRECTION
# ============================================================================

_heading = [1.0, 0.0]

def _reset_heading():
    global _heading
    _heading = [1.0, 0.0]

def update_heading(direction):
    global _heading
    hx, hy = _heading
    if direction == "right":
        _heading = [hy, -hx]
    elif direction == "left":
        _heading = [-hy, hx]
    print(f"[Heading] after '{direction}': {_heading}")

def get_direction_from_route(current_node, goal_node, route):
    if not route:
        return "forward"
    path  = route.get('path', [])
    edges = route.get('edges', [])
    if len(path) < 2:
        return "forward"
    try:
        idx = path.index(current_node)
    except ValueError:
        return "forward"
    if idx >= len(path) - 1:
        return "forward"
    next_node = path[idx + 1]
    if idx >= len(edges):
        return "forward"

    edge_id   = edges[idx]
    edge_data = road_map.get_edge(edge_id)
    if edge_data:
        if edge_data.get('from') == current_node:
            d = edge_data.get('direction', 'forward')
        else:
            d = edge_data.get('reverse_direction', 'forward')
        _log(f"[Direction] edge {edge_id} {current_node}->{next_node}: '{d}'")
        return d

    print(f"[Direction] edge {edge_id} not found, defaulting forward")
    return "forward"

# ============================================================================
# INTERSECTION FSM
# ============================================================================

class IntersectionFSM:
    def __init__(self):
        self._phase     = "done"
        self._direction = "forward"
        self._phase_end = 0.0

    def reset(self):
        self._phase     = "done"
        self._direction = "forward"
        self._phase_end = 0.0

    @property
    def running(self):
        return self._phase != "done"

    def start(self, direction):
        self._direction = direction
        self._enter_phase("clear")
        _log(f"[Intersection] Starting — direction='{direction}'")

    def _enter_phase(self, phase):
        self._phase = phase
        now = time.time()
        if phase == "clear":
            self._phase_end = now + FORWARD_CLEAR_TIME
        elif phase == "turn":
            if self._direction == "forward":
                self._enter_phase("exit")
                return
            self._phase_end = now + TURN_TIMES[self._direction]
        elif phase == "exit":
            self._phase_end = now + EXIT_TIME
        elif phase == "done":
            self._phase_end = 0.0

    def update(self, wheels):
        if self._phase == "done":
            return False
        now      = time.time()
        finished = now >= self._phase_end

        if self._phase == "clear":
            wheels.set_wheels_speed(CREEP_SPEED, CREEP_SPEED)
            if finished:
                self._enter_phase("turn")

        elif self._phase == "turn":
            if self._direction == "forward":
                wheels.set_wheels_speed(CREEP_SPEED, CREEP_SPEED)
            elif self._direction == "left":
                wheels.set_wheels_speed(-TURN_SPEED, TURN_SPEED)
            else:
                wheels.set_wheels_speed(TURN_SPEED, -TURN_SPEED)
            if finished:
                self._enter_phase("exit")

        elif self._phase == "exit":
            # Drive forward — lane follower picks up when it sees lines
            wheels.set_wheels_speed(EXIT_SPEED, EXIT_SPEED)
            if finished:
                self._enter_phase("done")
                return False

        return True

# ============================================================================
# NAVIGATION AGENT
# ============================================================================

class NavigationAgent:
    def __init__(self):
        self.lane_follower    = LaneServoingAgent()
        self.lane_follower._YELLOW_TARGET = 0.30
        self.lane_follower._WHITE_TARGET  = 0.72
        self.intersection_fsm = IntersectionFSM()
        self.state            = "driving"
        self.current_route = None
        self._red_window      = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames  = 0
        self._route_initialized = False
        _reset_heading()

    def reset(self):
        _log("[Agent] Resetting")
        self.lane_follower._prev_error     = 0.0
        self.lane_follower._filtered_error = 0.0
        self.intersection_fsm.reset()
        self.state              = "driving"
        self.current_route      = None
        self._red_window        = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames    = 0
        self._route_initialized = False
        _reset_heading()

    def _transition(self, new_state):
        _log(f"[Agent] {self.state} → {new_state}")
        self.state = new_state

    def _advance_node(self):
        if self.current_route is None:
            return
        path = self.current_route.get('path', [])
        current = server.current_node
        try:
            idx = path.index(current)
        except ValueError:
            return
        if idx + 1 < len(path):
            server.current_node = path[idx + 1]
            _log(f"[Agent] Node advanced: {current} → {server.current_node}")

    def _red_vote(self, detected):
        self._red_window.append(1 if detected else 0)
        if len(self._red_window) < RED_WINDOW_SIZE:
            return False
        return (sum(self._red_window) / RED_WINDOW_SIZE) >= RED_VOTE_THRESH

    def update(self, frame_bgr, wheels, leds, current_node, goal_node):
        global debug_frame
        red_mask  = None
        fsm_phase = None

        # ── COMPLETED ──────────────────────────────────────────────────────
        if self.state == "completed":
            wheels.set_wheels_speed(0.0, 0.0)
            self._transition("celebrating")
            return True

        if self.state == "celebrating":
            wheels.set_wheels_speed(0.0, 0.0)
            return False

        # ── CROSSING ───────────────────────────────────────────────────────
        if self.state == "crossing":
            fsm_phase     = self.intersection_fsm._phase
            still_running = self.intersection_fsm.update(wheels)
            if not still_running:
                update_heading(self.intersection_fsm._direction)
                self._advance_node()
                self.current_route      = None
                self._route_initialized = False
                self._driving_frames    = 0
                self._red_window.clear()
                self.lane_follower._prev_error     = 0.0
                self.lane_follower._filtered_error = 0.0
                self._transition("driving")
            if frame_bgr is not None:
                debug_frame = build_debug_frame(
                    frame_bgr, None, None, None,
                    self.state, fsm_phase, 0.0)
            return True

        # ── DRIVING ────────────────────────────────────────────────────────
        if self.state == "driving":
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            left, right = self.lane_follower.compute_commands(frame_rgb)

            self._driving_frames += 1
            armed = self._driving_frames >= RED_ARM_FRAMES

            if not armed:
                wheels.set_wheels_speed(left, right)
            else:
                red_detected, red_mask = detect_red_line(frame_bgr)
                confirmed = self._red_vote(red_detected)

                vote_fraction = sum(self._red_window) / max(len(self._red_window), 1)
                if vote_fraction > 0.3:
                    speed_scale = max(0.0, 1.0 - vote_fraction)
                    wheels.set_wheels_speed(left * speed_scale, right * speed_scale)
                else:
                    wheels.set_wheels_speed(left, right)

                if confirmed:
                    wheels.set_wheels_speed(0.0, 0.0)
                    self._red_window.clear()
                    self._driving_frames = 0

                    if not self._route_initialized:
                        _log(f"[Agent] First red line at node {server.current_node}")
                        self.current_route = dijkstra(server.current_node, goal_node)
                        self.current_route['start'] = server.current_node
                        _log(f"[Agent] Route: {self.current_route['path']}")
                        _log(f"[Agent] Edges: {self.current_route['edges']}")
                        self._route_initialized = True

                    if server.current_node == goal_node:
                        self._transition("completed")
                        return False

                    print(
                        f"[Agent] Red line confirmed — crossing | node={server.current_node} route={self.current_route.get('path') if self.current_route else None}")
                    direction = get_direction_from_route(server.current_node, goal_node, self.current_route)
                    self.intersection_fsm.start(direction)
                    self._transition("crossing")

        if frame_bgr is not None:
            di = self.lane_follower.last_debug_info
            debug_frame = build_debug_frame(
                raw_bgr     = frame_bgr,
                mask_yellow = di.get('yellow_mask'),
                mask_white  = di.get('white_mask'),
                mask_red    = red_mask,
                state       = self.state,
                sub         = fsm_phase,
                error       = self.lane_follower._prev_error,
            )
        return True


# Module-level instance — servers reference agent.debug_frame
agent = NavigationAgent()


# ============================================================================
# MAIN LOOP
# ============================================================================

def main(camera, wheels, leds, stop_event, server_module=None):
    global server, debug_frame

    if server_module is not None:
        server = server_module

    agent.reset()
    debug_frame = None

    _log(f"[Agent] Started — Start: {server.current_node}  Goal: {server.goal_node}")
    _log(f"[Agent] (server id={id(server)}, current_node attr={server.current_node})")

    try:
        while not stop_event.is_set():
            start = server.current_node
            goal  = server.goal_node

            if hasattr(camera, 'read_rgb'):
                ok, frame_rgb = camera.read_rgb()
                if not ok or frame_rgb is None:
                    time.sleep(0.02)
                    continue
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            else:
                ok, frame_bgr = camera.read()
                if not ok or frame_bgr is None:
                    time.sleep(0.02)
                    continue
                if len(frame_bgr.shape) == 3 and frame_bgr.shape[2] == 4:
                    frame_bgr = frame_bgr[:, :, :3]

            should_continue = agent.update(frame_bgr, wheels, leds, start, goal)

            if not should_continue:
                _log("[Agent] Route complete — dancing")
                if wheels:
                    wheels.set_wheels_speed(0.0, 0.0)
                time.sleep(2.0)

                end_time = time.time() + 4.0
                step = 0
                dance_colors = [
                    [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
                    [1.0, 1.0, 0.0], [0.0, 1.0, 1.0], [1.0, 0.0, 1.0],
                ]
                while time.time() < end_time and not stop_event.is_set():
                    l, r = (0.8, -0.8) if step % 2 == 0 else (-0.8, 0.8)
                    if wheels:
                        wheels.set_wheels_speed(l, r)
                    if leds:
                        try:
                            leds.set_led(0, dance_colors[step % len(dance_colors)])
                        except Exception:
                            pass
                    time.sleep(0.1)
                    step += 1

                if wheels:
                    wheels.set_wheels_speed(0.0, 0.0)
                if leds:
                    try:
                        leds.all_off()
                    except Exception:
                        pass
                print("[Agent] Dance done")
                break

            time.sleep(0.02)

    finally:
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            try:
                leds.all_off()
            except Exception:
                pass
        _log("[Agent] Stopped")