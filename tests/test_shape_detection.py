import cv2
import numpy as np
import pytest
from peg_sorting.vision.shape_detection import classify_shape


@pytest.mark.parametrize("shape,n", [("triangle", 3), ("square", 4), ("hexagon", 6), ("circle", 64)])
@pytest.mark.parametrize("angle", [0, .32, .83])
def test_rotated_noisy_shape(shape, n, angle):
    image = np.zeros((260, 260), np.uint8)
    theta = np.arange(n) * 2 * np.pi / n + angle
    vertices = np.column_stack((130 + 70 * np.cos(theta), 125 + 70 * np.sin(theta))).round().astype(np.int32)
    cv2.fillPoly(image, [vertices], 255)
    image = cv2.GaussianBlur(image, (3, 3), .8)
    image = cv2.threshold(image, 120, 255, cv2.THRESH_BINARY)[1]
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    assert classify_shape(max(contours, key=cv2.contourArea)) == shape


def test_degenerate_shape():
    assert classify_shape(np.zeros((3, 1, 2), np.int32)) == "unknown"
