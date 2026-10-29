import cv2
import numpy as np
from peg_sorting.vision.pose_estimation import ObservedPose, pixel_on_plane, depth_world_points
from peg_sorting.vision.shape_detection import approximate, classify_shape, polygon_orientation, refine_polygon_pose

PEG_TO_SOCKET = {s: s for s in ("square", "triangle", "circle", "hexagon")}


def board_mask(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    purple = cv2.inRange(hsv, (138, 80, 45), (169, 255, 255))
    contours, _ = cv2.findContours(purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("Board perimeter not visible")
    hull = cv2.convexHull(np.vstack(contours))
    mask = np.zeros(rgb.shape[:2], np.uint8)
    cv2.drawContours(mask, [hull], -1, 255, -1)
    return cv2.erode(mask, np.ones((7, 7), np.uint8))


def detect_sockets(frame):
    mask = board_mask(frame.rgb)
    u, v, xyz = depth_world_points(frame, mask)
    # Estimate the rim plane from the neutral board surface, not stored locations.
    neutral = np.ptp(frame.rgb[v, u].astype(float), axis=1) < 25
    heights = xyz[neutral, 2]
    if len(heights) < 100:
        raise RuntimeError("Insufficient board surface for plane estimate")
    rim_z = float(np.quantile(heights, .75))
    holes = np.zeros_like(mask)
    holes[v[xyz[:, 2] < rim_z - .002], u[xyz[:, 2] < rim_z - .002]] = 255
    # RETR_LIST also finds apertures when a dark margin surrounds the board tile.
    contours, _ = cv2.findContours(holes, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    result = {}
    for contour in contours:
        if not 150 < cv2.contourArea(contour) < 6000:
            continue
        shape = classify_shape(contour)
        if shape == "unknown":
            continue
        moments = cv2.moments(contour)
        uv = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        center = pixel_on_plane(*uv, rim_z, frame)
        poly = approximate(contour).reshape(-1, 2)
        vertices = pixel_on_plane(poly[:, 0], poly[:, 1], rim_z, frame)
        yaw = polygon_orientation(vertices[:, :2], center[:2], shape)
        boundary = contour.reshape(-1, 2)
        world_boundary = pixel_on_plane(boundary[:, 0], boundary[:, 1], rim_z, frame)
        center[:2], yaw = refine_polygon_pose(world_boundary[:, :2], center[:2], yaw, shape)
        result[shape] = ObservedPose(shape, "socket", center, yaw, .95, contour, uv)
    return result


def match_sockets(pegs, sockets):
    missing = set(pegs) - set(sockets)
    if missing:
        raise ValueError(f"Missing matching sockets: {sorted(missing)}")
    return [(peg, sockets[PEG_TO_SOCKET[shape]]) for shape, peg in pegs.items()]
