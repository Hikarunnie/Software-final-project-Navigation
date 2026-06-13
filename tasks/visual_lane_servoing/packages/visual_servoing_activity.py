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

_yellow_lower = np.array([_h.get('yellow_lower_h', 0),  _h.get('yellow_lower_s', 0),  _h.get('yellow_lower_v', 0)])
_yellow_upper = np.array([_h.get('yellow_upper_h', 0),  _h.get('yellow_upper_s', 0), _h.get('yellow_upper_v', 0)])

_white_lower = np.array([_h.get('white_lower_h', 0),   _h.get('white_lower_s', 0), _h.get('white_lower_v', 0)])
_white_upper = np.array([_h.get('white_upper_h', 0), _h.get('white_upper_s', 0), _h.get('white_upper_v', 0)])

def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Return (yellow_mask, white_mask) as float32 {0.0, 1.0} arrays.

    Detection is purely colour-based so the WHOLE line shows up — the solid
    white line and the full body of each yellow dash — instead of only the
    thin gradient outline the old edge-based method produced. Light morphology
    removes speckle and fills small gaps so each line reads as a solid blob,
    and the top of the frame (walls / horizon / background) is ignored.
    """
    h, w = image.shape[:2]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask_yellow = cv2.inRange(hsv, _yellow_lower, _yellow_upper)
    mask_white  = cv2.inRange(hsv, _white_lower,  _white_upper)

    # Only look at the road: drop everything above ~the horizon so coloured
    # walls / sockets / background can't leak into the masks.
    roi_top = int(h * 0.35)
    mask_yellow[:roi_top, :] = 0
    mask_white[:roi_top, :]  = 0

    # OPEN kills tiny speckles; CLOSE fills pinholes and joins the broken bits
    # of each line/dash so the mask comes out solid instead of "blended".
    k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN,  k_open)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, k_close)

    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN,  k_open)
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, k_close)

    mask_left  = (mask_yellow > 0).astype(np.float32)
    mask_right = (mask_white  > 0).astype(np.float32)

    return mask_left, mask_right





def set_hsv_bounds(yellow_lower, yellow_upper, white_lower, white_upper):
    global _yellow_lower, _yellow_upper, _white_lower, _white_upper
    _yellow_lower    = np.array(yellow_lower)
    _yellow_upper    = np.array(yellow_upper)
    _white_lower = np.array(white_lower)
    _white_upper = np.array(white_upper)

def get_hsv_bounds():
    return {
        'yellow_lower_h': int(_yellow_lower[0]),    'yellow_upper_h': int(_yellow_upper[0]),
        'yellow_lower_s': int(_yellow_lower[1]),    'yellow_upper_s': int(_yellow_upper[1]),
        'yellow_lower_v': int(_yellow_lower[2]),    'yellow_upper_v': int(_yellow_upper[2]),
        'white_lower_h':  int(_white_lower[0]), 'white_upper_h':  int(_white_upper[0]),
        'white_lower_s':  int(_white_lower[1]), 'white_upper_s':  int(_white_upper[1]),
        'white_lower_v':  int(_white_lower[2]), 'white_upper_v':  int(_white_upper[2]),
    }