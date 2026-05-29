from typing import List, Tuple
import numpy as np


def detect_curve(
    yellow_xs: List[int],
    white_xs: List[int],
    curve_threshold: int = 350,
) -> Tuple[bool, int]:
    # Prefer yellow if available, otherwise fallback to white
    xs = None
    if len(yellow_xs) > 1:
        xs = yellow_xs
    elif len(white_xs) > 1:
        xs = white_xs
    else:
        return False, 0  # not enough data

    # Near vs far points
    x_near = xs[0]
    x_far = xs[-1]

    shift = x_far - x_near

    # Detect curve
    if abs(shift) > curve_threshold:
        direction = 1 if shift > 0 else -1
        return True, direction

    return False, 0