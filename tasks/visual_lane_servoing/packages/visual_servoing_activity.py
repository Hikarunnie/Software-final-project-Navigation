from typing import Tuple
import os
import numpy as np
import cv2
import yaml

_HSV_FILE = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config', 'lane_servoing_hsv_config.yaml')
try:
    with open(_HSV_FILE) as _f:
        _h = yaml.safe_load(_f) or {}
except FileNotFoundError:
    _h = {}

_yellow_lower = np.array([_h.get('yellow_lower_h', 22),  _h.get('yellow_lower_s', 100), _h.get('yellow_lower_v', 100)])
_yellow_upper = np.array([_h.get('yellow_upper_h', 35),  _h.get('yellow_upper_s', 255), _h.get('yellow_upper_v', 255)])
_white_lower  = np.array([_h.get('white_lower_h',   0),  _h.get('white_lower_s',    0), _h.get('white_lower_v',  175)])
_white_upper  = np.array([_h.get('white_upper_h', 179),  _h.get('white_upper_s',   55), _h.get('white_upper_v',  255)])


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns (yellow_mask, white_mask) as float32 binary arrays (0.0 or 1.0).
    Input: BGR image from the Duckiebot camera.

    Yellow = left dashed centre line  → only searched in LEFT 60 % of frame.
    White  = right solid edge line    → only searched in RIGHT 65 % of frame.

    Pipeline: blur → HSV colour → spatial crop → edge AND → morphology cleanup.
    """
    h, w = image.shape[:2]

    # 1. Only look at the bottom 55 % of the frame (road, not horizon/sky)
    crop_top = int(h * 0.45)
    roi = image[crop_top:, :]
    rh, rw = roi.shape[:2]

    # 2. Blur to suppress sensor noise before colour detection
    blurred = cv2.GaussianBlur(roi, (5, 5), 1.5)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 3. Colour masks
    yellow_colour = cv2.inRange(hsv, _yellow_lower, _yellow_upper)
    white_colour  = cv2.inRange(hsv, _white_lower,  _white_upper)

    # 4. Canny edge mask on the V (brightness) channel — painted lines have
    #    sharp edges; flat floors/walls/reflections do not.
    v_blur = cv2.GaussianBlur(hsv[:, :, 2], (3, 3), 1.0)
    edges  = cv2.Canny(v_blur, 25, 70)
    k_edge = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    edges  = cv2.dilate(edges, k_edge)   # widen so colour blob overlaps edge

    # 5. Combine: pixel must be BOTH the right colour AND on an edge
    yellow_mask = cv2.bitwise_and(yellow_colour, edges)
    white_mask  = cv2.bitwise_and(white_colour,  edges)

    # 6. Hard spatial split —————————————————————————————————————————————
    #    Yellow centre line is always on the LEFT side of the robot's view.
    #    Block right 40 % so reflections / white line never read as yellow.
    yellow_mask[:, int(rw * 0.60):] = 0

    #    White edge line is always on the RIGHT side.
    #    Block left 35 % so yellow line / left wall never read as white.
    white_mask[:, :int(rw * 0.35)] = 0

    # 7. Morphological cleanup: open kills noise, close fills dashed-line gaps
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    k7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN,  k3)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, k7)
    white_mask  = cv2.morphologyEx(white_mask,  cv2.MORPH_OPEN,  k3)
    white_mask  = cv2.morphologyEx(white_mask,  cv2.MORPH_CLOSE, k7)

    # 8. Embed ROI back into full-frame arrays
    full_yellow = np.zeros((h, w), dtype=np.uint8)
    full_white  = np.zeros((h, w), dtype=np.uint8)
    full_yellow[crop_top:, :] = yellow_mask
    full_white[crop_top:,  :] = white_mask

    return (full_yellow > 0).astype(np.float32), (full_white > 0).astype(np.float32)


def set_hsv_bounds(yellow_lower, yellow_upper, white_lower, white_upper):
    global _yellow_lower, _yellow_upper, _white_lower, _white_upper
    _yellow_lower = np.array(yellow_lower)
    _yellow_upper = np.array(yellow_upper)
    _white_lower  = np.array(white_lower)
    _white_upper  = np.array(white_upper)

def get_hsv_bounds():
    return {
        'yellow_lower_h': int(_yellow_lower[0]), 'yellow_upper_h': int(_yellow_upper[0]),
        'yellow_lower_s': int(_yellow_lower[1]), 'yellow_upper_s': int(_yellow_upper[1]),
        'yellow_lower_v': int(_yellow_lower[2]), 'yellow_upper_v': int(_yellow_upper[2]),
        'white_lower_h':  int(_white_lower[0]),  'white_upper_h':  int(_white_upper[0]),
        'white_lower_s':  int(_white_lower[1]),  'white_upper_s':  int(_white_upper[1]),
        'white_lower_v':  int(_white_lower[2]),  'white_upper_v':  int(_white_upper[2]),
    }