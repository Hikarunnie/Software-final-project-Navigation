import os
import yaml
import numpy as np
import cv2
from collections import deque
from typing import Tuple

from tasks.visual_lane_servoing.packages import visual_servoing_activity as student
from tasks.visual_lane_servoing.packages.cuvrve_behavior import detect_curve

_CONFIG_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'config', 'lane_servoing_config.yaml'
))

_LINE_OFFSET = 160
_ROI_START = 0.47
_NUM_SLICES = 3
_SLICE_TOL = 5

# Hard boundary: if either line is closer than this fraction to the robot center,
# trigger a strong correction regardless of center tracking.
_YELLOW_HARD_LIMIT = 0.30   # yellow must stay to the LEFT of this x fraction
_WHITE_HARD_LIMIT  = 0.75   # white must stay to the RIGHT of this x fraction
_BOUNDARY_PENALTY  = 3.0    # multiplier for hard-limit violations


def detect_lines_in_slices(mask_yellow, mask_white, h):
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

    def __init__(self, config_path=None):
        path = config_path or _CONFIG_FILE
        try:
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        except Exception:
            cfg = {}

        self.p_gain              = cfg.get('p_gain', 0.4)
        self.d_gain              = cfg.get('d_gain', 0.15)
        self.max_steer           = cfg.get('max_steer', 0.20)
        self.base_speed          = cfg.get('base_speed', 0.17)
        self.curve_speed         = cfg.get('curve_speed', 0.12)
        self.curve_threshold     = cfg.get('curve_threshold', 350)
        self.steering_threshold  = cfg.get('steering_threshold', 0.15)
        self.curve_boost         = cfg.get('curve_boost', 1.05)
        self.detection_threshold = cfg.get('detection_threshold', 20)

        self.frame_count      = 0
        self._prev_error      = 0.0
        self._filtered_error  = 0.0
        self._lane_half_width = float(_LINE_OFFSET)
        self._left_history    = deque(maxlen=3)
        self._right_history   = deque(maxlen=3)
        self.last_debug_info  = self._empty_debug_info(480, 640)

    # Ideal lane-center as a fraction of frame width.
    # This is the midpoint between where yellow and white typically sit.
    _LANE_CENTER = 0.55          # target: sit just right of center, closer to yellow
    _YELLOW_NOMINAL = 0.38       # where yellow line ideally appears
    _WHITE_NOMINAL  = 0.72       # where white line ideally appears

    def _calculate_error(self, yellow_xs, white_xs, left_det, right_det, w, is_curve=False):
        """
        Strategy: always steer toward the lane center derived from visible lines.
        If only one line is visible, estimate center from it.
        Hard boundary penalties fire if either line is violated regardless of mode.
        """
        have_yellow = left_det and len(yellow_xs) > 0
        have_white  = right_det and len(white_xs) > 0

        y_fx = float(np.mean(yellow_xs)) / w if have_yellow else None
        w_fx = float(np.mean(white_xs))  / w if have_white  else None

        # ── 1. Compute lane-center error ──────────────────────────────────────
        if have_yellow and have_white:
            # Best case: both lines visible → true lane center
            lane_center = (y_fx + w_fx) / 2.0
            error = self._LANE_CENTER - lane_center

        elif have_yellow:
            # Only yellow: estimate center by adding expected half-lane width
            half_width = self._WHITE_NOMINAL - self._YELLOW_NOMINAL
            estimated_center = y_fx + half_width / 2.0
            error = self._LANE_CENTER - estimated_center

        elif have_white:
            # Only white: estimate center by subtracting expected half-lane width
            half_width = self._WHITE_NOMINAL - self._YELLOW_NOMINAL
            estimated_center = w_fx - half_width / 2.0
            error = self._LANE_CENTER - estimated_center

        else:
            # No lines: coast on memory, decaying
            return float(np.clip(self._prev_error * 0.9, -1.0, 1.0))

        # ── 2. Hard boundary penalties (line-crossing prevention) ─────────────
        # These fire on top of center tracking whenever we're dangerously close
        # to either line, including mid-curve overshoot.
        penalty = 0.0

        if have_yellow and y_fx > _YELLOW_HARD_LIMIT:
            # Yellow is too far right → robot drifted left → push right (positive error)
            violation = y_fx - _YELLOW_HARD_LIMIT
            penalty += violation * _BOUNDARY_PENALTY

        if have_white and w_fx < _WHITE_HARD_LIMIT:
            # White is too far left → robot drifted right → push left (negative error)
            violation = _WHITE_HARD_LIMIT - w_fx
            penalty -= violation * _BOUNDARY_PENALTY

        # On curves, boundary penalties are stronger because overshoot is faster
        if is_curve:
            penalty *= 1.5

        error += penalty

        return float(np.clip(error, -1.0, 1.0))

    def _calculate_steering(self, error):
        error_diff = error - self._prev_error
        self._prev_error = error
        steering = self.p_gain * error + self.d_gain * error_diff
        return float(np.clip(steering, -self.max_steer, self.max_steer))

    def _motor_commands(self, steering, recovery, is_curve, both_visible):
        if recovery:
            # No lines detected — coast forward slowly using last known steering
            coast_speed = self.curve_speed * 0.7
            left  = float(np.clip(coast_speed - self._prev_error * self.p_gain, 0.0, 1.0))
            right = float(np.clip(coast_speed + self._prev_error * self.p_gain, 0.0, 1.0))
            return left, right

        speed = self.curve_speed if is_curve else self.base_speed
        if not both_visible:
            speed *= 0.88  # slow down a bit when flying blind on one line

        left  = float(np.clip(speed - steering, 0.0, 1.0))
        right = float(np.clip(speed + steering, 0.0, 1.0))
        return left, right

    def _smooth(self, left, right, both_visible):
        buf = 2 if both_visible else 1
        if self._left_history.maxlen != buf:
            self._left_history  = deque(maxlen=buf)
            self._right_history = deque(maxlen=buf)
        self._left_history.append(left)
        self._right_history.append(right)
        return (sum(self._left_history) / len(self._left_history),
                sum(self._right_history) / len(self._right_history))

    def compute_commands(self, image):
        self.frame_count += 1
        bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        try:
            mask_left, mask_right = student.detect_lane_markings(bgr)
        except Exception as e:
            print(f"[Agent] detect_lane_markings error: {e}")
            return 0.0, 0.0

        mask_y = (mask_left  * 255).astype(np.uint8)
        mask_w = (mask_right * 255).astype(np.uint8)

        yellow_pixels = int(np.count_nonzero(mask_y))
        white_pixels  = int(np.count_nonzero(mask_w))
        total_pixels  = yellow_pixels + white_pixels

        combined = np.clip(mask_left + mask_right, 0, 1)
        self.last_debug_info = {
            'roi':               image,
            'lane_mask':         (combined * 255).astype(np.uint8),
            'white_mask':        mask_w,
            'yellow_mask':       mask_y,
            'total_lane_pixels': total_pixels,
            'lateral_error':     float(np.clip(self._prev_error, -1.0, 1.0)),
            'lane_detected':     total_pixels >= self.detection_threshold,
            'frame_count':       self.frame_count,
        }

        h, w      = mask_y.shape
        left_det  = yellow_pixels > 0
        right_det = white_pixels  > 0
        recovery  = total_pixels  < self.detection_threshold

        yellow_xs, white_xs = detect_lines_in_slices(mask_y, mask_w, h)
        both_visible = left_det and right_det and not recovery
        is_curve, curve_dir = detect_curve(yellow_xs, white_xs, self.curve_threshold)

        raw_error = self._calculate_error(
            yellow_xs, white_xs, left_det, right_det, w, is_curve
        )
        self._filtered_error = 0.7 * self._filtered_error + 0.3 * raw_error
        steering = self._calculate_steering(self._filtered_error)
        left, right = self._motor_commands(steering, recovery, is_curve, both_visible)
        left, right = self._smooth(left, right, both_visible)

        slice_height = int(h * 0.35 / _NUM_SLICES)
        start_y      = int(h * _ROI_START)
        self.last_debug_info.update({
            'yellow_xs': yellow_xs,
            'white_xs':  white_xs,
            'slice_ys':  [start_y + i * slice_height + slice_height // 2
                          for i in range(_NUM_SLICES)],
            'is_curve':  is_curve,
            'curve_dir': curve_dir,
        })

        return left, right

    def step(self, image, wheels_driver):
        left, right = self.compute_commands(image)
        wheels_driver.set_wheels_speed(left, right)
        return left, right

    def get_debug_info(self, image):
        return self.last_debug_info

    def _empty_debug_info(self, h, w):
        return {
            'roi':               np.zeros((h, w, 3), dtype=np.uint8),
            'lane_mask':         np.zeros((h, w), dtype=np.uint8),
            'white_mask':        np.zeros((h, w), dtype=np.uint8),
            'yellow_mask':       np.zeros((h, w), dtype=np.uint8),
            'total_lane_pixels': 0,
            'lateral_error':     0.0,
            'lane_detected':     False,
            'frame_count':       0,
        }