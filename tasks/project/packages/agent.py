import time
import numpy as np
import cv2
from collections import deque

from tasks.project.packages.road_map import road_map
from tasks.project.packages.optimal_path import dijkstra
from tasks.visual_lane_servoing.packages.visual_servoing_activity import detect_lane_markings

# Object detection
try:
    from tasks.object_detection.packages.agent import ObjectDetectionAgent
    from tasks.object_detection.packages.stop_activity import should_stop
    _DETECTION_AVAILABLE = True
except ImportError as e:
    print(f"[Agent] Object detection not available: {e}")
    _DETECTION_AVAILABLE = False

# Server module — injected by main()
server = None


# ============================================================================
# TUNING CONSTANTS
# ============================================================================

APPROACH_SPEED     = 0.10
CREEP_SPEED        = 0.06
FORWARD_CLEAR_TIME = 0.55
TURN_SPEED         = 0.20
TURN_TIME_FORWARD  = 0.22
TURN_TIME_LEFT     = 0.04
TURN_TIME_RIGHT    = 0.15
EXIT_SPEED         = 0.10
EXIT_TIME          = 0.70

TURN_TIMES = {
    "forward": TURN_TIME_FORWARD,
    "left":    TURN_TIME_LEFT,
    "right":   TURN_TIME_RIGHT,
}

RED_WINDOW_SIZE  = 12
RED_VOTE_THRESH  = 0.65
RED_ARM_FRAMES   = 32

CLASS_NAMES  = {0: 'duckie', 1: 'truck', 2: 'sign'}
CLASS_COLORS = {0: (0, 215, 255), 1: (180, 100, 220), 2: (50, 205, 50)}


# ============================================================================
# DEBUG VISUALISATION
# ============================================================================

debug_frame = None


def _mask_to_uint8(mask):
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


def build_debug_frame(raw_bgr, mask_yellow, mask_white, mask_red, state, sub, error, detections=None):
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

    # Draw detection boxes
    if detections:
        for (x1, y1, x2, y2), score, cls_id in detections:
            color = CLASS_COLORS.get(cls_id, (255, 255, 255))
            cv2.rectangle(raw, (x1, y1), (x2, y2), color, 2)
            label_txt = f"{CLASS_NAMES.get(cls_id, '?')} {score:.2f}"
            cv2.putText(raw, label_txt, (x1, max(y1 - 6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

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

def detect_red_line(image):
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
        return False, mask

    return True, mask


# ============================================================================
# LANE FOLLOWING CONTROLLER
# ============================================================================

class LaneFollowingController:

    def __init__(self):
        self.p_gain           = 0.12
        self.d_gain           = 0.10
        self.max_steer        = 0.40
        self.base_speed       = 0.10
        self.prev_error       = 0.0
        self._lane_half_width = 160.0
        self.last_mask_white  = None
        self.last_mask_yellow = None
        self.last_error       = 0.0 ,

    def reset(self):
        self.prev_error       = 0.0
        self.last_error       = 0.0
        self.last_mask_white  = None
        self.last_mask_yellow = None

    def reset_error(self):
        self.reset()

    def compute_commands(self, image):
        if image is None:
            return 0.0, 0.0

        h, w = image.shape[:2]

        try:
            mask_yellow, mask_white = detect_lane_markings(image)
            self.last_mask_white  = mask_white
            self.last_mask_yellow = mask_yellow
        except Exception as e:
            print(f"[LaneFollowing] detect_lane_markings error: {e}")
            self.last_mask_white  = None
            self.last_mask_yellow = None
            return 0.0, 0.0

        yellow_pixels = np.where(mask_yellow > 0)
        white_pixels  = np.where(mask_white  > 0)

        has_yellow = len(yellow_pixels[1]) > 0
        has_white  = len(white_pixels[1])  > 0

        if has_yellow and has_white:
            white_mean_x  = np.mean(white_pixels[1])
            yellow_mean_x = np.mean(yellow_pixels[1])
            white_span_x  = int(white_pixels[1].max() - white_pixels[1].min())
            yellow_count  = len(yellow_pixels[1])
            white_count   = len(white_pixels[1])

            dynamic_ratio_threshold = min(1.3 + (yellow_count / 2000.0), 1.8)
            ratio_bad    = white_count > yellow_count * dynamic_ratio_threshold
            span_bad     = white_span_x > w * 0.25
            position_bad = white_mean_x < yellow_mean_x + (w * 0.15)
            apex_bad     = int(white_pixels[1].min()) > w * 0.55

            if ratio_bad or span_bad or position_bad or apex_bad:
                has_white = False

        error = self.prev_error

        if has_yellow and has_white:
            lane_center = (np.mean(yellow_pixels[1]) + np.mean(white_pixels[1])) / 2.0
            error       = (w / 2.0 - lane_center) / (w / 2.0)
        elif has_yellow:
            yellow_x = np.mean(yellow_pixels[1])
            yellow_y = np.mean(yellow_pixels[0])
            y_scale  = float(np.clip((yellow_y - h * 0.4) / (h * 0.6), 0.0, 1.0))
            scaled_half_width = 60.0 + y_scale * 100.0
            error = (w / 2.0 - (yellow_x + scaled_half_width)) / (w / 2.0)
        elif has_white:
            white_x = np.mean(white_pixels[1])
            error   = (w / 2.0 - (white_x - self._lane_half_width)) / (w / 2.0)

        error           = float(np.clip(error, -1.0, 1.0))
        self.last_error = error
        error_diff      = error - self.prev_error
        self.prev_error = error

        steering = float(np.clip(
            self.p_gain * error + self.d_gain * error_diff,
            -self.max_steer, self.max_steer,
        ))

        left  = float(np.clip(self.base_speed - steering, -0.4, 0.4))
        right = float(np.clip(self.base_speed + steering, -0.4, 0.4))
        return left, right


# ============================================================================
# HEADING TRACKER
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


def get_direction_from_route(current_node, goal_node, route_path):
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

            forward_comp =  nx * hx + ny * hy
            lateral_comp =  nx * hy - ny * hx

            if abs(lateral_comp) < abs(forward_comp) * 0.58:
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
        print(f"[Intersection] Starting crossing — direction='{direction}'")

    def _enter_phase(self, phase):
        self._phase = phase
        now         = time.time()

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
            if self._direction == "left":
                wheels.set_wheels_speed(-TURN_SPEED,  TURN_SPEED)
            else:
                wheels.set_wheels_speed( TURN_SPEED, -TURN_SPEED)
            if finished:
                wheels.set_wheels_speed(0.0, 0.0)
                self._enter_phase("exit")

        elif self._phase == "exit":
            wheels.set_wheels_speed(EXIT_SPEED, EXIT_SPEED)
            if finished:
                wheels.set_wheels_speed(0.0, 0.0)
                self._enter_phase("done")
                return False

        return True


# ============================================================================
# NAVIGATION AGENT
# ============================================================================

class NavigationAgent:

    def __init__(self):
        self.lane_follower    = LaneFollowingController()
        self.intersection_fsm = IntersectionFSM()
        self.state            = "driving"
        self.current_route    = None
        self._red_window      = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames  = 0
        self.last_state       = None
        self._route_initialized = False
        self.last_detections  = []

        # Object detector — shared across resets, loaded once
        if _DETECTION_AVAILABLE:
            try:
                self.detector = ObjectDetectionAgent()
                print("[Agent] Object detector loaded")
            except Exception as e:
                print(f"[Agent] Object detector failed to load: {e}")
                self.detector = None
        else:
            self.detector = None

        _reset_heading()

    def reset(self):
        """Full reset — call this every time navigation is (re)started."""
        print("[Agent] Resetting for new run")
        self.lane_follower.reset()
        self.intersection_fsm.reset()
        self.state              = "driving"
        self.current_route      = None
        self._red_window        = deque(maxlen=RED_WINDOW_SIZE)
        self._driving_frames    = 0
        self.last_state         = None
        self._route_initialized = False
        self.last_detections    = []
        _reset_heading()

    def _transition(self, new_state):
        print(f"[Agent] {self.state} → {new_state}")
        self.last_state = self.state
        self.state      = new_state

    def _advance_node(self):
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

    def _red_vote(self, detected):
        self._red_window.append(1 if detected else 0)
        if len(self._red_window) < RED_WINDOW_SIZE:
            return False
        return (sum(self._red_window) / RED_WINDOW_SIZE) >= RED_VOTE_THRESH

    def _get_route(self, start, goal):
        try:
            route = dijkstra(start, goal)
            route['start'] = start
            print(f"[Agent] Route: {route['path']}")
            return route
        except Exception as e:
            print(f"[Agent] Dijkstra error: {e}")
            return None

    def _run_detection(self, frame_bgr):
        """Run object detection and update self.last_detections. Returns (stop, reason)."""
        if self.detector is None:
            return False, ''
        try:
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            detections = self.detector.detect(frame_rgb)
            if detections is not None:
                self.last_detections = detections
            if self.last_detections:
                h, w = frame_bgr.shape[:2]
                return should_stop(self.last_detections, w)
        except Exception as e:
            print(f"[Agent] Detection error: {e}")
        return False, ''

    def update(self, image, camera, wheels, leds, current_node, goal_node):
        global debug_frame

        frame_bgr = image
        red_mask  = None
        fsm_phase = None

        # ── COMPLETED ─────────────────────────────────────────────────────────
        if self.state == "completed":
            wheels.set_wheels_speed(0.0, 0.0)
            self._transition("celebrating")
            return True

        if self.state == "celebrating":
            wheels.set_wheels_speed(0.0, 0.0)
            return False

        # ── CROSSING ──────────────────────────────────────────────────────────
        if self.state == "crossing":
            fsm_phase     = self.intersection_fsm._phase
            still_running = self.intersection_fsm.update(wheels)

            if not still_running:
                direction = self.intersection_fsm._direction
                update_heading(direction)
                self._advance_node()
                self.current_route   = None
                self._driving_frames = 0
                self._red_window.clear()
                self.lane_follower.reset()
                self.last_detections = []
                self._transition("driving")

            if frame_bgr is not None:
                debug_frame = build_debug_frame(
                    frame_bgr,
                    mask_yellow=None, mask_white=None, mask_red=None,
                    state=self.state, sub=fsm_phase, error=0.0,
                    detections=self.last_detections,
                )
            return True

        # ── DRIVING ───────────────────────────────────────────────────────────
        if self.state == "driving":
            left, right = self.lane_follower.compute_commands(frame_bgr)

            self._driving_frames += 1
            armed = self._driving_frames >= RED_ARM_FRAMES

            # ── Object detection — stop if something is in the way ────────────
            obj_stop, obj_reason = self._run_detection(frame_bgr)
            if obj_stop:
                print(f"[Agent] Object detected — stopping: {obj_reason}")
                wheels.set_wheels_speed(0.0, 0.0)
                # Update debug frame so the box is visible while stopped
                if frame_bgr is not None:
                    debug_frame = build_debug_frame(
                        raw_bgr     = frame_bgr,
                        mask_yellow = self.lane_follower.last_mask_yellow,
                        mask_white  = self.lane_follower.last_mask_white,
                        mask_red    = red_mask,
                        state       = self.state,
                        sub         = "BLOCKED",
                        error       = self.lane_follower.last_error,
                        detections  = self.last_detections,
                    )
                return True  # keep running — will re-check next frame

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
                        self.current_route  = self._get_route(current_node, goal_node)
                        self._route_initialized = True

                    if current_node == goal_node:
                        self._transition("completed")
                        return False

                    print("[Agent] Red line CONFIRMED — entering intersection")
                    route_path = self.current_route.get('path', []) if self.current_route else []
                    direction  = get_direction_from_route(current_node, goal_node, route_path)
                    self.intersection_fsm.start(direction)
                    self._transition("crossing")

        if frame_bgr is not None:
            debug_frame = build_debug_frame(
                raw_bgr     = frame_bgr,
                mask_yellow = self.lane_follower.last_mask_yellow,
                mask_white  = self.lane_follower.last_mask_white,
                mask_red    = red_mask,
                state       = self.state,
                sub         = fsm_phase,
                error       = self.lane_follower.last_error,
                detections  = self.last_detections,
            )

        return True


# Module-level agent instance — always reset before use
agent = NavigationAgent()


# ============================================================================
# MAIN LOOP  — called by the server in a thread
# ============================================================================

def main(camera, wheels, leds, stop_event, server_module=None):
    global server
    if server_module is not None:
        server = server_module

    global debug_frame

    # Full reset so every navigation start is clean
    agent.reset()
    debug_frame = None

    print("[Agent] Started")
    print(f"[Agent] Start: {server.current_node}  Goal: {server.goal_node}")

    try:
        while not stop_event.is_set():
            start = server.current_node
            goal  = server.goal_node

            if hasattr(camera, 'read_rgb'):
                ok, frame_rgb = camera.read_rgb()
            else:
                ok, frame_rgb = camera.read()
                if ok and frame_rgb is not None:
                    frame_rgb = cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2RGB)

            if not ok or frame_rgb is None:
                time.sleep(0.02)
                continue

            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            should_continue = agent.update(
                frame_bgr, camera, wheels, leds, start, goal
            )

            if not should_continue:
                print("[Agent] Route complete — waiting then dancing")
                if wheels:
                    wheels.set_wheels_speed(0.0, 0.0)
                time.sleep(2.0)

                # Dance
                end_time = time.time() + 4.0
                step = 0
                dance_colors = [
                    [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
                    [1.0, 1.0, 0.0], [0.0, 1.0, 1.0], [1.0, 0.0, 1.0],
                ]
                while time.time() < end_time and not stop_event.is_set():
                    if step % 2 == 0:
                        l, r = 0.8, -0.8
                    else:
                        l, r = -0.8, 0.8
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
                print("[Agent] Dance done — stopping")
                break

            time.sleep(0.02)  # ~50 Hz

    finally:
        if wheels:
            wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            try:
                leds.all_off()
            except Exception:
                pass
        print("[Agent] Stopped")