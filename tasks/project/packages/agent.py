import time

from tasks.project.packages.cuvrve_behavior import detect_curve
from tasks.project.packages.road_map import road_map
from tasks.project.packages.optimal_path import dijkstra
from tasks.project.packages import visual_servoing_activity as student
import servers.project.virtual_server as server
import os
import yaml
import numpy as np
import cv2
from collections import deque
from typing import Tuple



# ============================================================================
# TUNING CONSTANTS  — adjust these first if the bot misbehaves
# ============================================================================
#
# All times are in seconds.  The bot runs at 50 Hz (0.02 s / frame).
#
# APPROACH_SPEED       speed while closing on the red line
# CREEP_SPEED          slow final approach once the red line is confirmed
# FORWARD_CLEAR_TIME   time to drive straight *before* turning so the bot
#                      clears the stop line and centres itself in the box
# TURN_SPEED           wheel speed during a pivot turn
# TURN_TIME_FORWARD    time for a straight crossing at TURN_SPEED
# TURN_TIME_LEFT       time for a 90° left pivot at TURN_SPEED
# TURN_TIME_RIGHT      time for a 90° right pivot at TURN_SPEED
# EXIT_SPEED           speed while driving out of the intersection box
# EXIT_TIME            time to drive straight after turning before re-arming
#                      the lane follower  ← tune second if bot misses the lane

APPROACH_SPEED     = 0.10   # m/s (Godot PWM units)
CREEP_SPEED        = 0.06
FORWARD_CLEAR_TIME = 0.55   # s — enough to pass the stop line fully
TURN_SPEED         = 0.20
TURN_TIME_FORWARD  = 0.22   # s for forward crossing
TURN_TIME_LEFT     = 0.14   # s for 90° left pivot
TURN_TIME_RIGHT    = 0.15   # s for 90° right pivot
EXIT_SPEED         = 0.10
EXIT_TIME          = 0.70   # s before lane follower re-arms

# Convenience lookup used by IntersectionFSM
TURN_TIMES = {
    "forward": TURN_TIME_FORWARD,
    "left":    TURN_TIME_LEFT,
    "right":   TURN_TIME_RIGHT,
}

# Red-line detection voting
RED_WINDOW_SIZE  = 12        # sliding-window length (frames)
RED_VOTE_THRESH  = 0.65      # fraction of window that must be positive
RED_ARM_FRAMES   = 32        # frames after state entry before red is checked
                             # (prevents instant re-trigger after an intersection)


# ============================================================================
# DEBUG VISUALISATION
# ============================================================================

debug_frame = None


def _mask_to_uint8(mask: np.ndarray) -> np.ndarray:
    if mask is None:
        return None
    if mask.dtype == np.uint8:
        return mask
    if mask.max() <= 1.0:
        return (mask * 255).astype(np.uint8)
    return np.clip(mask, 0, 255).astype(np.uint8)


def _make_panel(mask_u8, h, w, tint_bgr, label):
    panel = np.zeros((h, w, 3), dtype=np.uint8)
    if mask_u8 is not None:
        resized = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_NEAREST)
        panel[resized > 0] = tint_bgr
        count = int(np.count_nonzero(mask_u8))
        cv2.putText(panel, f"{label} ({count}px)", (4, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    else:
        cv2.putText(panel, f"{label} (no mask)", (4, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 80, 80), 1)
    return panel


def build_debug_frame(raw_bgr, mask_yellow, mask_white, mask_red, state, sub, error):
    if raw_bgr is None:
        return None

    h, w = raw_bgr.shape[:2]
    panel_w = w // 2
    panel_h = h // 3

    raw = raw_bgr.copy()
    label = f"{state.upper()}" + (f"/{sub}" if sub else "")
    cv2.putText(raw, label, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(raw, f"err:{error:+.3f}", (8, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

    p_yellow = _make_panel(_mask_to_uint8(mask_yellow), panel_h, panel_w,
                           tint_bgr=[0, 220, 220], label="YELLOW LANE")
    p_white  = _make_panel(_mask_to_uint8(mask_white),  panel_h, panel_w,
                           tint_bgr=[0, 220, 0],   label="WHITE LANE")
    p_red    = _make_panel(_mask_to_uint8(mask_red),    panel_h, panel_w,
                           tint_bgr=[0, 0, 220],   label="RED LINE")

    right_col = np.vstack([p_yellow, p_white, p_red])
    right_col = cv2.resize(right_col, (panel_w, h))

    return np.hstack([raw, right_col])


# ============================================================================
# RED LINE DETECTION
# ============================================================================

def detect_red_line(image: np.ndarray) -> tuple:
    """
    Returns (detected: bool, mask: np.ndarray).
    Only looks in the bottom 45% of the frame.
    Rejects blobs that are too square (stop signs) or too narrow.
    """
    if image is None or len(image.shape) != 3:
        return False, None

    if image.dtype != np.uint8:
        image = (np.clip(image, 0, 1) * 255).astype(np.uint8) \
                if image.max() <= 1.0 \
                else np.clip(image, 0, 255).astype(np.uint8)

    h, w = image.shape[:2]
    roi_top = int(h * 0.55)
    roi     = image[roi_top:, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0,   100, 100])
    upper_red1 = np.array([10,  255, 255])
    lower_red2 = np.array([165, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    roi_mask = cv2.bitwise_or(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2),
    )

    kernel   = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_OPEN, kernel)

    mask = np.zeros((h, w), dtype=np.uint8)
    mask[roi_top:, :] = roi_mask

    if int(np.count_nonzero(roi_mask)) < 150:
        return False, mask

    rows, cols = np.where(roi_mask > 0)
    if len(cols) == 0:
        return False, mask

    span_x = int(cols.max() - cols.min()) + 1
    span_y = int(rows.max() - rows.min()) + 1
    aspect = span_x / max(span_y, 1)

    if aspect < 2.5 or span_x < int(w * 0.15):
        return False, mask   # sign blob, not a line

    return True, mask


# ============================================================================
# LANE FOLLOWING CONTROLLER
# ============================================================================

_CONFIG_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'config', 'lane_servoing_config.yaml'
))

_LINE_OFFSET = 160
_ROI_START = 0.47
_NUM_SLICES = 3
_SLICE_TOL = 5


def detect_lines_in_slices(
        mask_yellow: np.ndarray,
        mask_white: np.ndarray,
        h: int,
) -> Tuple[list, list]:
    slice_height = int(h * 0.35 / _NUM_SLICES)
    start_y = int(h * _ROI_START)
    yellow_xs, white_xs = [], []

    for i in range(_NUM_SLICES):
        y = start_y + i * slice_height + slice_height // 2

        strip_y = mask_yellow[y - _SLICE_TOL: y + _SLICE_TOL, :]
        idx = np.where(strip_y > 0)[1]
        if len(idx) > 0:
            yellow_xs.append(int(np.mean(idx)))

        strip_w = mask_white[y - _SLICE_TOL: y + _SLICE_TOL, :]
        idx = np.where(strip_w > 0)[1]
        if len(idx) > 0:
            white_xs.append(int(np.mean(idx)))

    return yellow_xs, white_xs


class LaneServoingAgent:

    def __init__(self, config_path: str = None):
        path = config_path or _CONFIG_FILE
        try:
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception:
            cfg = {}

        self.p_gain = cfg.get('p_gain', 0.1)
        self.d_gain = cfg.get('d_gain', 0.35)
        self.max_steer = cfg.get('max_steer', 0.4)
        self.base_speed = cfg.get('base_speed', 0.2)
        self.curve_speed = cfg.get('curve_speed', 0.2)
        self.curve_threshold = cfg.get('curve_threshold', 350)
        self.steering_threshold = cfg.get('steering_threshold', 0.2)
        self.curve_boost = cfg.get('curve_boost', 1.3)
        self.detection_threshold = cfg.get('detection_threshold', 500)

        self.frame_count = 0
        self._prev_error = 0.0
        self._filtered_error = 0.0
        self._lane_half_width = float(_LINE_OFFSET)
        self._left_history = deque(maxlen=3)
        self._right_history = deque(maxlen=3)
        self.last_debug_info = self._empty_debug_info(480, 640)

    def _calculate_error(self, yellow_xs, white_xs, left_det, right_det, w):
        if left_det and right_det and yellow_xs and white_xs:
            y_mean = float(np.mean(yellow_xs))
            w_mean = float(np.mean(white_xs))
            measured = (w_mean - y_mean) / 2.0
            if measured > 20:
                self._lane_half_width = 0.9 * self._lane_half_width + 0.1 * measured
            error = w / 2.0 - (y_mean + w_mean) / 2.0
        elif left_det and yellow_xs:
            error = w / 2.0 - (float(np.mean(yellow_xs)) + self._lane_half_width)
        elif right_det and white_xs:
            error = w / 2.0 - (float(np.mean(white_xs)) - self._lane_half_width)
        else:
            error = self._prev_error

        return float(np.clip(error / (w / 2.0), -1.0, 1.0))

    def _calculate_steering(self, error: float) -> float:
        error_diff = error - self._prev_error
        self._prev_error = error
        steering = self.p_gain * error + self.d_gain * error_diff
        return float(np.clip(steering, -self.max_steer, self.max_steer))

    def _motor_commands(self, steering: float, recovery: bool, is_curve: bool, both_visible: bool):
        if recovery:
            return 0.0, 0.0

        speed = self.curve_speed if is_curve else self.base_speed

        if not both_visible:
            speed *= 0.8

        left = speed - steering
        right = speed + steering

        if is_curve and abs(steering) > self.steering_threshold:
            if steering > 0:
                right *= 5
            else:
                left *= self.curve_boost

        return float(np.clip(left, 0.0, 1.0)), float(np.clip(right, 0.0, 1.0))

    def _smooth(self, left, right, both_visible):
        buf = 2 if both_visible else 1
        if self._left_history.maxlen != buf:
            self._left_history = deque(maxlen=buf)
            self._right_history = deque(maxlen=buf)
        self._left_history.append(left)
        self._right_history.append(right)
        return (sum(self._left_history) / len(self._left_history),
                sum(self._right_history) / len(self._right_history))

    def compute_commands(self, image: np.ndarray) -> Tuple[float, float]:
        self.frame_count += 1
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        try:
            mask_left, mask_right = student.detect_lane_markings(bgr)
        except Exception as e:
            print(f"[Agent] detect_lane_markings error: {e}")
            return 0.0, 0.0

        mask_y = (mask_left * 255).astype(np.uint8)
        mask_w = (mask_right * 255).astype(np.uint8)

        yellow_pixels = int(np.count_nonzero(mask_y))
        white_pixels = int(np.count_nonzero(mask_w))
        total_pixels = yellow_pixels + white_pixels

        combined = np.clip(mask_left + mask_right, 0, 1)
        self.last_debug_info = {
            'roi': image,
            'lane_mask': (combined * 255).astype(np.uint8),
            'white_mask': mask_w,
            'yellow_mask': mask_y,
            'total_lane_pixels': total_pixels,
            'lateral_error': float(np.clip(self._prev_error, -1.0, 1.0)),
            'lane_detected': total_pixels >= self.detection_threshold,
            'frame_count': self.frame_count,
        }

        h, w = mask_y.shape
        left_det = yellow_pixels > 0
        right_det = white_pixels > 0
        recovery = total_pixels < self.detection_threshold

        yellow_xs, white_xs = detect_lines_in_slices(mask_y, mask_w, h)
        both_visible = left_det and right_det and not recovery
        is_curve, curve_dir = detect_curve(yellow_xs, white_xs, self.curve_threshold)

        raw_error = self._calculate_error(yellow_xs, white_xs, left_det, right_det, w)
        self._filtered_error = 0.7 * self._filtered_error + 0.3 * raw_error
        steering = self._calculate_steering(self._filtered_error)
        left, right = self._motor_commands(steering, recovery, is_curve, both_visible)
        left, right = self._smooth(left, right, both_visible)

        slice_height = int(h * 0.35 / _NUM_SLICES)
        start_y = int(h * _ROI_START)
        self.last_debug_info.update({
            'yellow_xs': yellow_xs,
            'white_xs': white_xs,
            'slice_ys': [start_y + i * slice_height + slice_height // 2 for i in range(_NUM_SLICES)],
            'is_curve': is_curve,
            'curve_dir': curve_dir,
        })

        return left, right

    def step(self, image: np.ndarray, wheels_driver) -> Tuple[float, float]:
        left, right = self.compute_commands(image)
        wheels_driver.set_wheels_speed(left, right)
        return left, right

    def get_debug_info(self, image: np.ndarray) -> dict:
        return self.last_debug_info

    def _empty_debug_info(self, h, w):
        return {
            'roi': np.zeros((h, w, 3), dtype=np.uint8),
            'lane_mask': np.zeros((h, w), dtype=np.uint8),
            'white_mask': np.zeros((h, w), dtype=np.uint8),
            'yellow_mask': np.zeros((h, w), dtype=np.uint8),
            'total_lane_pixels': 0,
            'lateral_error': 0.0,
            'lane_detected': False,
            'frame_count': 0,
        }

lane_follower = LaneServoingAgent()
# ============================================================================
# HEADING TRACKER  — heading-aware direction lookup
# ============================================================================
#
# The bot starts facing +X (east).  After each turn we rotate the heading
# vector 90° in the appropriate direction.  This lets get_direction_from_route
# express the world-space delta to the next node in the bot's *local* frame,
# which is the only way left/right stays correct after the first turn.

_heading = [1.0, 0.0]   # (hx, hy), starts facing +X


def _reset_heading():
    global _heading
    _heading = [1.0, 0.0]


def update_heading(direction: str):
    global _heading
    hx, hy = _heading
    if direction == "right":
        _heading = [hy, -hx]
    elif direction == "left":
        _heading = [-hy, hx]
    print(f"[Heading] after '{direction}': {_heading}")


def get_direction_from_route(current_node: int, goal_node: int, route_path: list) -> str:
    """
    Determine turn direction at the current node.
    Returns 'forward', 'left', or 'right'.
    """
    if not route_path or len(route_path) < 2:
        return "forward"

    try:
        idx = route_path.index(current_node)
    except ValueError:
        print(f"[Direction] Node {current_node} not in path {route_path}")
        return "forward"

    if idx >= len(route_path) - 1:
        return "forward"

    next_node = route_path[idx + 1]

    # ── Try edge attribute first ──────────────────────────────────────────────
    try:
        for neighbor_id, length, edge_id in road_map.neighbors(current_node):
            if neighbor_id == next_node:
                edge_data = road_map.get_edge(edge_id)
                if edge_data and 'direction' in edge_data:
                    d = edge_data['direction']
                    print(f"[Direction] edge attr {current_node}→{next_node}: '{d}'")
                    return d
    except Exception as e:
        print(f"[Direction] edge lookup error: {e}")

    # ── Heading-aware coordinate fallback ─────────────────────────────────────
    try:
        cn = road_map.get_node(current_node)
        nn = road_map.get_node(next_node)
        if cn and nn:
            dx  = nn['x'] - cn['x']
            dy  = nn['y'] - cn['y']
            mag = (dx ** 2 + dy ** 2) ** 0.5
            if mag < 1e-6:
                return "forward"

            nx, ny = dx / mag, dy / mag
            hx, hy = _heading

            forward_comp =  nx * hx + ny * hy   # dot  (+ve = ahead)
            lateral_comp =  nx * hy - ny * hx   # cross (+ve = right)

            if abs(lateral_comp) < abs(forward_comp) * 0.58:  # within ±30°
                d = "forward"
            elif lateral_comp > 0:
                d = "right"
            else:
                d = "left"

            print(f"[Direction] heading-aware {current_node}→{next_node} "
                  f"heading={_heading} delta=({dx:+.2f},{dy:+.2f}) "
                  f"fwd={forward_comp:+.2f} lat={lateral_comp:+.2f} → '{d}'")
            return d
    except Exception as e:
        print(f"[Direction] coord fallback error: {e}")

    print("[Direction] all lookups failed → 'forward'")
    return "forward"


# ============================================================================
# INTERSECTION FSM
# ============================================================================

class IntersectionFSM:
    """
    Encapsulates the full intersection crossing sequence.
    Call .start(direction) when the bot stops at the red line.
    Call .update(wheels) every tick; it returns True while still running.
    """

    # Sub-states executed in order
    _PHASES = ("clear", "turn", "exit", "done")

    def __init__(self):
        self._phase     = "done"
        self._direction = "forward"
        self._phase_end = 0.0

    @property
    def running(self) -> bool:
        return self._phase != "done"

    def start(self, direction: str):
        self._direction = direction
        self._enter_phase("clear")
        print(f"[Intersection] Starting crossing — direction='{direction}'")

    def _enter_phase(self, phase: str):
        self._phase = phase
        now         = time.time()

        if phase == "clear":
            # Drive straight past the stop line
            self._phase_end = now + FORWARD_CLEAR_TIME
            print(f"[Intersection] Phase CLEAR for {FORWARD_CLEAR_TIME:.2f}s")

        elif phase == "turn":
            if self._direction == "forward":
                # No turn needed — jump straight to exit
                self._enter_phase("exit")
                return
            turn_time = TURN_TIMES[self._direction]
            self._phase_end = now + turn_time
            print(f"[Intersection] Phase TURN '{self._direction}' for {turn_time:.2f}s")

        elif phase == "exit":
            self._phase_end = now + EXIT_TIME
            print(f"[Intersection] Phase EXIT for {EXIT_TIME:.2f}s")

        elif phase == "done":
            self._phase_end = 0.0

    def update(self, wheels) -> bool:
        """
        Apply wheel commands for the current phase.
        Returns True while the manoeuvre is in progress.
        """
        if self._phase == "done":
            return False

        now      = time.time()
        finished = now >= self._phase_end

        if self._phase == "clear":
            wheels.set_wheels_speed(CREEP_SPEED, CREEP_SPEED)
            if finished:
                self._enter_phase("turn")

        elif self._phase == "turn":
            if self._direction == "left":
                # Left pivot: left wheel back, right wheel forward
                wheels.set_wheels_speed(-TURN_SPEED,  TURN_SPEED)
            else:
                # Right pivot: left wheel forward, right wheel back
                wheels.set_wheels_speed( TURN_SPEED, -TURN_SPEED)
            if finished:
                wheels.set_wheels_speed(0.0, 0.0)   # brief stop before exit
                self._enter_phase("exit")

        elif self._phase == "exit":
            wheels.set_wheels_speed(EXIT_SPEED, EXIT_SPEED)
            if finished:
                wheels.set_wheels_speed(0.0, 0.0)
                self._enter_phase("done")
                return False

        return True


intersection_fsm = IntersectionFSM()


# ============================================================================
# MAIN NAVIGATION AGENT
# ============================================================================

class NavigationAgent:
    """
    Top-level state machine.

    States
    ------
    driving              — lane-follow toward next intersection
    crossing             — open-loop intersection traversal (IntersectionFSM)
    completed            — destination reached
    """

    def __init__(self):
        self.state         = "driving"
        self.current_route = None
        self._red_window: deque = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames    = 0
        self.last_state         = None
        self._route_initialized = False
        _reset_heading()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: str):
        print(f"[Agent] {self.state} → {new_state}")
        self.last_state = self.state
        self.state      = new_state

    def _advance_node(self):
        """Move current_node one step along the route."""
        if self.current_route is None:
            return
        path    = self.current_route.get('path', [])
        current = server.current_node
        try:
            idx = path.index(current)
        except ValueError:
            return
        if idx + 1 < len(path):
            server.current_node = path[idx + 1]
            print(f"[Agent] Node advanced: {current} → {server.current_node}")

    def _red_vote(self, detected: bool) -> bool:
        self._red_window.append(1 if detected else 0)
        if len(self._red_window) < RED_WINDOW_SIZE:
            return False
        return (sum(self._red_window) / RED_WINDOW_SIZE) >= RED_VOTE_THRESH

    def _get_route(self, start: int, goal: int):
        try:
            route = dijkstra(start, goal)
            route['start'] = start
            print(f"[Agent] Route: {route['path']}")
            return route
        except Exception as e:
            print(f"[Agent] Dijkstra error: {e}")
            return None

    # ------------------------------------------------------------------
    # Main update  (called at ~50 Hz)
    # ------------------------------------------------------------------

    def update(self, image: np.ndarray, camera, wheels, leds,
               current_node: int, goal_node: int) -> bool:
        global debug_frame

        frame_bgr  = image
        red_mask   = None
        fsm_phase  = None

        # ── COMPLETED ──────────────────────────────────────────────────────────
        if self.state == "completed":
            wheels.set_wheels_speed(0.0, 0.0)
            return False

        # ── CROSSING (open-loop intersection manoeuvre) ─────────────────────
        if self.state == "crossing":
            fsm_phase = intersection_fsm._phase
            still_running = intersection_fsm.update(wheels)

            if not still_running:
                # Manoeuvre finished — update heading and advance node
                direction = intersection_fsm._direction
                update_heading(direction)
                self._advance_node()
                self.current_route = None        # force route refresh
                self._driving_frames = 0
                self._red_window.clear()
                lane_follower.reset_error()
                self._transition("driving")

            # Build debug frame (no lane masks during crossing)
            if frame_bgr is not None:
                debug_frame = build_debug_frame(
                    frame_bgr,
                    mask_yellow = None,
                    mask_white  = None,
                    mask_red    = None,
                    state       = self.state,
                    sub         = fsm_phase,
                    error       = 0.0,
                )
            return True

        # ── DRIVING ────────────────────────────────────────────────────────────
        if self.state == "driving":


            # Lane-follow
            left, right = lane_follower.compute_commands(frame_bgr)

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
                        print(f"[Agent] First red line — initializing at node {current_node}")
                        server.current_node = current_node
                        self.current_route = self._get_route(current_node, goal_node)
                        self._route_initialized = True

                    # Check if we've physically arrived at the goal
                    if current_node == goal_node:
                        self._transition("completed")
                        return False

                    print("[Agent] Red line CONFIRMED — entering intersection")
                    route_path = self.current_route.get('path', [])
                    direction = get_direction_from_route(current_node, goal_node, route_path)
                    intersection_fsm.start(direction)
                    self._transition("crossing")

        if frame_bgr is not None:
            debug_frame = build_debug_frame(
                raw_bgr     = frame_bgr,
                mask_yellow = lane_follower.last_mask_yellow,
                mask_white  = lane_follower.last_mask_white,
                mask_red    = red_mask,
                state       = self.state,
                sub         = fsm_phase,
                error       = lane_follower.last_error,
            )

        return True


agent = NavigationAgent()


# ============================================================================
# MAIN LOOP
# ============================================================================

def main(camera, wheels, leds, stop_event):
    print("[Agent] Started")
    print(f"[Agent] Start: {server.current_node}  Goal: {server.goal_node}")

    try:
        while not stop_event.is_set():
            start = server.current_node
            goal  = server.goal_node
            # NEW
            ok, frame_bgr = camera.read()
            if not ok or frame_bgr is None:
                time.sleep(0.02)
                continue

            should_continue = agent.update(
                frame_bgr, camera, wheels, leds, start, goal
            )

            if not should_continue:
                print("[Agent] Route complete — exiting")
                break

            time.sleep(0.02)   # 50 Hz

    finally:
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            try:
                leds.all_off()
            except Exception:
                pass
        print("[Agent] Stopped")