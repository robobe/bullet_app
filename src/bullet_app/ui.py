"""OpenCV-only presentation helpers."""
import textwrap

import cv2


def draw_debug(image, bbox, status):
    """Annotate a copy so display overlays never enter the detector."""
    display = image.copy()
    if bbox is not None:
        x, y, width, height = bbox
        cv2.rectangle(display, (x, y), (x + width - 1, y + height - 1), (0, 255, 255), 2)
    lines = textwrap.wrap(status, width=max(20, (display.shape[1] - 16) // 10))
    cv2.rectangle(display, (0, 0), (display.shape[1] - 1, min(display.shape[0] - 1, len(lines) * 22 + 6)),
                  (15, 25, 30), -1)
    for index, line in enumerate(lines):
        cv2.putText(display, line, (8, 23 + index * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (80, 240, 100), 1, cv2.LINE_AA)
    return display
