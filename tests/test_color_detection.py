import cv2
import numpy as np
import pytest
from peg_sorting.vision.color_segmentation import ColorSegmenter


@pytest.mark.parametrize("name,rgb", [("red", (255, 0, 0)), ("green", (0, 255, 0)), ("blue", (0, 0, 255)), ("yellow", (255, 255, 0))])
def test_color_centroid_and_noise(name, rgb):
    image = np.zeros((160, 200, 3), np.uint8)
    image[40:120, 60:140] = rgb
    image[5, 5] = rgb
    detections = ColorSegmenter().detect(image)
    assert len(detections) == 1
    assert detections[0].color == name
    assert np.allclose(detections[0].centroid, (99.5, 79.5), atol=.5)
    assert detections[0].bbox == (60, 40, 80, 80)


def test_red_hue_wrap():
    hsv = np.zeros((100, 100, 3), np.uint8)
    hsv[20:80, 20:80] = (178, 255, 220)
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    assert ColorSegmenter().detect(rgb)[0].color == "red"


def test_reject_bad_image():
    with pytest.raises(ValueError):
        ColorSegmenter().detect(np.ones((10, 10), float))
