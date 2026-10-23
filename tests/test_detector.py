import cv2
import numpy as np
import pytest

from bullet_app.detector import detect_red_box


@pytest.mark.parametrize("hue", [0, 5, 175, 179])
def test_red_hues_and_input_preserved(hue):
    hsv = np.zeros((40, 60, 3), dtype=np.uint8)
    hsv[10:20, 15:35] = (hue, 255, 255)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    original = image.copy()
    assert detect_red_box(image) == (15, 10, 20, 10)
    np.testing.assert_array_equal(image, original)


def test_largest_object_and_noise():
    image = np.zeros((60, 80, 3), dtype=np.uint8)
    image[5:10, 5:10] = (0, 0, 255)
    image[20:40, 30:60] = (0, 0, 255)
    image[50, 70] = (0, 0, 255)
    assert detect_red_box(image) == (30, 20, 30, 20)


def test_no_target_small_noise_and_minimum_target():
    image = np.zeros((30, 30, 3), dtype=np.uint8)
    image[5:15, 5:15] = (255, 0, 0)
    assert detect_red_box(image) is None
    image[20:22, 20:22] = (0, 0, 255)
    assert detect_red_box(image) is None
    image[20:23, 20:23] = (0, 0, 255)
    assert detect_red_box(image) == (20, 20, 3, 3)


@pytest.mark.parametrize("image", [None, [], np.zeros((0, 4, 3), np.uint8),
                                    np.zeros((4, 4), np.uint8), np.zeros((4, 4, 4), np.uint8),
                                    np.zeros((4, 4, 3), np.float32)])
def test_invalid_input(image):
    with pytest.raises(ValueError, match="uint8 BGR"):
        detect_red_box(image)
