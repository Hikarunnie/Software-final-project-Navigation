import time
import numpy as np
import cv2
from collections import deque

from tasks.project.packages.road_map import road_map
from tasks.project.packages.optimal_path import dijkstra
from tasks.visual_lane_servoing.packages.visual_servoing_activity import detect_lane_markings
import servers.project.virtual_server as server


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

debug_frame: np.ndarray | None = None


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

class LaneFollowingController:
    """PD controller for lane-centre following."""

    def __init__(self):
        self.p_gain           = 0.12
        self.d_gain           = 0.10
        self.max_steer        = 0.40
        self.base_speed       = 0.10
        self.prev_error       = 0.0
        self._lane_half_width = 160.0
        self.last_mask_white: np.ndarray | None  = None
        self.last_mask_yellow: np.ndarray | None = None
        self.last_error: float = 0.0

    def reset_error(self):
        self.prev_error = 0.0
        self.last_error = 0.0

    def compute_commands(self, image: np.ndarray) -> tuple:
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

        # Sanity-check: white must be meaningfully to the RIGHT of yellow.
        # Chevrons/arrows sit at or left of the yellow line in the problem
        # corner, so this one check kills the bad detection without touching
        # detect_lane_markings at all.
        if has_yellow and has_white:
            white_mean_x  = np.mean(white_pixels[1])
            yellow_mean_x = np.mean(yellow_pixels[1])
            white_span_x  = int(white_pixels[1].max() - white_pixels[1].min())
            yellow_count  = len(yellow_pixels[1])
            white_count   = len(white_pixels[1])
            apex_x        = int(white_pixels[1].min())

            # Dynamic threshold — tighter when yellow is sparse (bot drifting)
            # scales from 1.3x at low yellow up to 2.5x at abundant yellow
            dynamic_ratio_threshold = min(1.3 + (yellow_count / 2000.0), 1.8)
            ratio_bad    = white_count > yellow_count * dynamic_ratio_threshold
            span_bad     = white_span_x > w * 0.25   # tightened from 0.40 to 0.25
            position_bad = white_mean_x < yellow_mean_x + (w * 0.15)
            apex_bad = int(white_pixels[1].min()) > w * 0.55

            if ratio_bad or span_bad or position_bad or apex_bad:
                has_white = False

        error = self.prev_error

        if has_yellow and has_white:
            lane_center = (np.mean(yellow_pixels[1]) + np.mean(white_pixels[1])) / 2.0
            error       = (w / 2.0 - lane_center) / (w / 2.0)
        elif has_yellow:
            yellow_x = np.mean(yellow_pixels[1])
            yellow_y = np.mean(yellow_pixels[0])  # vertical centre of mass

            # Scale half-width by how low in the frame yellow is.
            # Bottom of frame (y=480) → full 160px offset
            # Top of ROI (y=192, i.e. 0.4*480) → reduced to ~60px offset
            # This prevents overcorrection when yellow is far away and high up.
            y_scale = (yellow_y - h * 0.4) / (h * 0.6)
            y_scale = float(np.clip(y_scale, 0.0, 1.0))
            scaled_half_width = 60.0 + y_scale * 100.0  # ranges 60→160px

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


lane_follower = LaneFollowingController()


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
            if current_node == goal_node:
                wheels.set_wheels_speed(0.0, 0.0)
                self._transition("completed")
                return False

            # Refresh route if needed
            if (self.current_route is None
                    or self.current_route.get('start') != current_node):
                self.current_route = self._get_route(current_node, goal_node)
                if self.current_route is None:
                    wheels.set_wheels_speed(0.0, 0.0)
                    return True

            # Lane-follow
            left, right = lane_follower.compute_commands(frame_bgr)

            self._driving_frames += 1
            armed = self._driving_frames >= RED_ARM_FRAMES

            if not armed:
                # Not yet armed — drive at full lane-follow speed
                wheels.set_wheels_speed(left, right)
            else:
                red_detected, red_mask = detect_red_line(frame_bgr)
                confirmed              = self._red_vote(red_detected)

                # Progressive slowdown: decelerate as vote fraction rises
                vote_fraction = sum(self._red_window) / max(len(self._red_window), 1)
                if vote_fraction > 0.3:
                    speed_scale = max(0.0, 1.0 - vote_fraction)
                    wheels.set_wheels_speed(left * speed_scale, right * speed_scale)
                else:
                    wheels.set_wheels_speed(left, right)

                if confirmed:
                    print("[Agent] Red line CONFIRMED — entering intersection")
                    wheels.set_wheels_speed(0.0, 0.0)
                    self._red_window.clear()

                    # Decide turn direction and launch the FSM
                    route_path = self.current_route.get('path', [])
                    direction  = get_direction_from_route(current_node, goal_node, route_path)
                    intersection_fsm.start(direction)
                    self._transition("crossing")

        # ── Build debug frame ──────────────────────────────────────────────────
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

            ok, frame_rgb = camera.read_rgb()
            if not ok or frame_rgb is None:
                time.sleep(0.02)
                continue

            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

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