import cv2
import numpy as np
from scipy.optimize import least_squares


def approximate(contour):
    return cv2.approxPolyDP(contour, .022 * cv2.arcLength(contour, True), True)


def classify_shape(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if area <= 0 or perimeter <= 0:
        return "unknown"
    vertices = len(approximate(contour))
    circularity = 4 * np.pi * area / perimeter ** 2
    if vertices == 3:
        return "triangle"
    if vertices == 4:
        _, (w, h), _ = cv2.minAreaRect(contour)
        if min(w, h) / max(w, h) > .75:
            return "square"
    if vertices == 6 and circularity < .91:
        return "hexagon"
    if vertices >= 7 and circularity > .78:
        return "circle"
    return "unknown"


def polygon_orientation(vertices, center, shape):
    if shape == "circle":
        return 0.0
    n = {"square": 4, "triangle": 3, "hexagon": 6}[shape]
    offset = np.pi / 4 if shape == "square" else 0.
    xy = np.asarray(vertices) - center
    angles = np.arctan2(xy[:, 1], xy[:, 0])
    angle = np.angle(np.mean(np.exp(1j * n * angles))) / n - offset
    period = 2 * np.pi / n
    return float((angle + period / 2) % period - period / 2)


def refine_polygon_pose(points, center, yaw, shape):
    """Fit all boundary pixels to regular polygon edges, reducing raster yaw bias."""
    if shape == "circle":
        return np.asarray(center), 0.
    n = {"square": 4, "triangle": 3, "hexagon": 6}[shape]
    vertex_offset = np.pi / 4 if shape == "square" else 0.
    angles = vertex_offset + (np.arange(n) * 2 + 1) * np.pi / n
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    points, center = np.asarray(points), np.asarray(center)
    radius = np.median(np.linalg.norm(points - center, axis=1))

    def residual(params):
        dx = points - params[:2]
        c, s = np.cos(params[2]), np.sin(params[2])
        local = dx @ np.array([[c, -s], [s, c]])
        return np.max(local @ normals.T, axis=1) - params[3]

    fit = least_squares(residual, [*center, yaw, radius * np.cos(np.pi / n)],
                        loss="soft_l1", f_scale=.0005, max_nfev=80)
    return fit.x[:2], float(fit.x[2])
