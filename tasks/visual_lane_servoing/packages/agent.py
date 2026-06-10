import os
import yaml
import numpy as np
import cv2
from collections import deque

from tasks.visual_lane_servoing.packages import visual_servoing_activity as student
from tasks.visual_lane_servoing.packages.cuvrve_behavior import detect_curve

_CONFIG_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__), '..', '..', '..', 'config', 'lane_servoing_config.yaml'
))

_ROI_START  = 0.47
_NUM_SLICES = 3
_SLICE_TOL  = 5

# Lines are only "too close" if they cross these thresholds.
# Inside the safe zone → error = 0, robot drives straight.
_YELLOW_TOO_CLOSE = 0.45   # yellow must stay LEFT  of this  → steer right if crosses
_WHITE_TOO_CLOSE  = 0.92   # white  must stay RIGHT of this  → steer left  if crosses


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

        self.p_gain              = cfg.get('p_gain',              0.35)
        self.d_gain              = cfg.get('d_gain',              0.08)
        self.max_steer           = cfg.get('max_steer',           0.06)
        self.base_speed          = cfg.get('base_speed',          0.17)
        self.curve_speed         = cfg.get('curve_speed',         0.12)
        self.curve_threshold     = cfg.get('curve_threshold',     350)
        self.detection_threshold = cfg.get('detection_threshold', 20)

        self.frame_count     = 0
        self._prev_error     = 0.0
        self._filtered_error = 0.0
        self._left_history   = deque(maxlen=4)
        self._right_history  = deque(maxlen=4)
        self.last_debug_info = self._empty_debug_info(480, 640)

    def _calculate_error(self, yellow_xs, white_xs, left_det, right_det, w):
        """
        Only react when a line crosses its threshold.
        Inside the safe zone → error = 0 → drive straight.
        """
        have_yellow = left_det  and len(yellow_xs) > 0
        have_white  = right_det and len(white_xs)  > 0

        y_fx = float(np.mean(yellow_xs)) / w if have_yellow else None
        w_fx = float(np.mean(white_xs))  / w if have_white  else None

        white_error  = 0.0
        yellow_error = 0.0

        if have_white and w_fx < _WHITE_TOO_CLOSE:
            # White crossed into danger zone → steer left
            # The further it crossed, the stronger the push
            white_error = _WHITE_TOO_CLOSE - w_fx   # positive → steer left

        if have_yellow and y_fx > _YELLOW_TOO_CLOSE:
            # Yellow crossed into danger zone → steer right
            yellow_error = -(y_fx - _YELLOW_TOO_CLOSE)  # negative → steer right

        error = white_error + yellow_error

        if error == 0.0 and not have_yellow and not have_white:
            # No lines at all — decay memory
            return float(np.clip(self._prev_error * 0.9, -1.0, 1.0))

        return float(np.clip(error, -1.0, 1.0))

    def _calculate_steering(self, error):
        error_diff       = error - self._prev_error
        self._prev_error = error
        steering         = self.p_gain * error + self.d_gain * error_diff
        return float(np.clip(steering, -self.max_steer, self.max_steer))

    def _motor_commands(self, steering, recovery, is_curve, both_visible, left_det):
        if recovery:
            coast = self.curve_speed * 0.7
            left  = float(np.clip(coast - self._prev_error * self.p_gain, 0.0, 1.0))
            right = float(np.clip(coast + self._prev_error * self.p_gain, 0.0, 1.0))
            return left, right

        speed = self.curve_speed if is_curve else self.base_speed

        if not both_visible:
            speed *= 0.70
        if not left_det:
            speed *= 0.60

        min_speed = speed * 0.5  # never let a wheel drop below half speed = smooth curves
        left  = float(np.clip(speed - steering, min_speed, 1.0))
        right = float(np.clip(speed + steering, min_speed, 1.0))
        return left, right

    def _smooth(self, left, right):
        self._left_history.append(left)
        self._right_history.append(right)
        return (sum(self._left_history)  / len(self._left_history),
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
            yellow_xs, white_xs, left_det, right_det, w
        )

        self._filtered_error = 0.7 * self._filtered_error + 0.3 * raw_error
        steering = self._calculate_steering(self._filtered_error)

        left, right = self._motor_commands(steering, recovery, is_curve,
                                           both_visible, left_det)
        left, right = self._smooth(left, right)

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
            'lane_mask':         np.zeros((h, w),    dtype=np.uint8),
            'white_mask':        np.zeros((h, w),    dtype=np.uint8),
            'yellow_mask':       np.zeros((h, w),    dtype=np.uint8),
            'total_lane_pixels': 0,
            'lateral_error':     0.0,
            'lane_detected':     False,
            'frame_count':       0,
        }