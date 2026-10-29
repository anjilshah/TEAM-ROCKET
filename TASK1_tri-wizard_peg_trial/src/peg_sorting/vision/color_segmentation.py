from dataclasses import dataclass
import cv2
import numpy as np
from peg_sorting.utils.helpers import config


@dataclass
class ColorDetection:
    color: str
    contour: np.ndarray
    centroid: tuple
    bbox: tuple
    area: float


class ColorSegmenter:
    def __init__(self, thresholds=None, min_area=90):
        self.thresholds = thresholds or config("colors")
        self.min_area = min_area

    def masks(self, rgb):
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError("Expected uint8 RGB image")
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        kernel = np.ones((3, 3), np.uint8)
        result = {}
        for name, spec in self.thresholds.items():
            mask = np.zeros(rgb.shape[:2], np.uint8)
            for lower, upper in spec["hsv_ranges"]:
                mask |= cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            result[name] = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return result

    def detect(self, rgb):
        result = []
        for color, mask in self.masks(rgb).items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                moments = cv2.moments(contour)
                if area < self.min_area or not moments["m00"]:
                    continue
                centroid = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
                result.append(ColorDetection(color, contour, centroid, cv2.boundingRect(contour), area))
        return result
