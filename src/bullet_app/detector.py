"""Detect the largest red target using the fly_smart HSV thresholds."""

import cv2
import numpy as np


def detect_red_box(bgr: np.ndarray) -> tuple[int, int, int, int] | None:
    """Return (x, y, width, height) in pixels, leaving the BGR uint8 input intact."""
    if (not isinstance(bgr, np.ndarray) or bgr.dtype != np.uint8
            or bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) == 0):
        raise ValueError("expected a nonempty HxWx3 uint8 BGR image")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
    high_red = cv2.inRange(hsv, (170, 100, 80), (180, 255, 255))
    mask = cv2.morphologyEx(low_red | high_red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 4:
        return None
    x, y, width, height = cv2.boundingRect(contour)
    if width < 3 or height < 3:
        return None
    return x, y, width, height
