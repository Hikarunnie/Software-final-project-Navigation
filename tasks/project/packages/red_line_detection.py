import numpy as np
import cv2


# HSV bounds for the red intersection line
_RED_LOWER_1 = np.array([0,   150, 100])
_RED_UPPER_1 = np.array([10,  255, 255])
_RED_LOWER_2 = np.array([170, 150, 100])
_RED_UPPER_2 = np.array([180, 255, 255])

# How much of the bottom portion of the frame to scan for red
_ROI_START = 0.55  # only look at bottom 45% of image
# Minimum red pixels to count as a red line detection
_MIN_RED_PIXELS = 500


def detect_red_line(image_bgr: np.ndarray) -> bool:
    """
    Detect the red stop line at an intersection.
    Returns True if a red line is visible and close enough to stop.

    Args:
        image_bgr: BGR image from the camera (numpy array)

    Returns:
        True if red line detected in the lower portion of the frame
    """
    h, w = image_bgr.shape[:2]

    # Only scan the bottom portion of the image — red line only matters when close
    roi = image_bgr[int(h * _ROI_START):, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Red wraps around in HSV so we need two ranges
    mask1 = cv2.inRange(hsv, _RED_LOWER_1, _RED_UPPER_1)
    mask2 = cv2.inRange(hsv, _RED_LOWER_2, _RED_UPPER_2)
    mask  = cv2.bitwise_or(mask1, mask2)

    red_pixels = int(np.count_nonzero(mask))

    return red_pixels >= _MIN_RED_PIXELS


def red_line_pixel_count(image_bgr: np.ndarray) -> int:
    """Return the raw red pixel count in the lower ROI — useful for tuning threshold."""
    h, w = image_bgr.shape[:2]
    roi  = image_bgr[int(h * _ROI_START):, :]
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, _RED_LOWER_1, _RED_UPPER_1)
    mask2 = cv2.inRange(hsv, _RED_LOWER_2, _RED_UPPER_2)
    return int(np.count_nonzero(cv2.bitwise_or(mask1, mask2)))