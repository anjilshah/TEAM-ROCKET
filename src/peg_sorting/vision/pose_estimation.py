from dataclasses import dataclass
import cv2
import numpy as np
from peg_sorting.vision.color_segmentation import ColorSegmenter
from peg_sorting.vision.shape_detection import approximate, classify_shape, polygon_orientation, refine_polygon_pose


@dataclass
class ObservedPose:
    shape: str
    color: str
    position: np.ndarray
    yaw: float
    confidence: float
    contour: np.ndarray
    centroid: tuple


def pixel_to_world(u, v, depth, K, camera_to_world):
    z = np.asarray(depth, float)
    if np.any(~np.isfinite(z) | (z <= 0) | (z > 5)):
        raise ValueError("Invalid metric depth")
    camera = np.stack(((np.asarray(u) - K[0, 2]) * z / K[0, 0],
                       (np.asarray(v) - K[1, 2]) * z / K[1, 1], z), axis=-1)
    return camera @ camera_to_world[:3, :3].T + camera_to_world[:3, 3]


def world_to_pixel(world, K, camera_to_world):
    cam = (np.asarray(world) - camera_to_world[:3, 3]) @ camera_to_world[:3, :3]
    if np.any(cam[..., 2] <= 0):
        raise ValueError("Point is behind camera")
    return np.stack((cam[..., 0] * K[0, 0] / cam[..., 2] + K[0, 2],
                     cam[..., 1] * K[1, 1] / cam[..., 2] + K[1, 2]), axis=-1), cam[..., 2]


def pixel_on_plane(u, v, z_world, frame):
    ray = np.stack(((np.asarray(u) - frame.K[0, 2]) / frame.K[0, 0],
                    (np.asarray(v) - frame.K[1, 2]) / frame.K[1, 1], np.ones_like(np.asarray(u))), axis=-1)
    ray_world = ray @ frame.camera_to_world[:3, :3].T
    scale = (z_world - frame.camera_to_world[2, 3]) / ray_world[..., 2]
    if np.any(scale <= 0):
        raise ValueError("Plane is behind camera")
    return frame.camera_to_world[:3, 3] + scale[..., None] * ray_world


def depth_world_points(frame, mask):
    v, u = np.where(mask > 0)
    depth = frame.depth[v, u]
    valid = np.isfinite(depth) & (depth > 0) & (depth < 5)
    return u[valid], v[valid], pixel_to_world(u[valid], v[valid], depth[valid], frame.K, frame.camera_to_world)


def detect_pegs(frame, segmenter=None):
    segmenter = segmenter or ColorSegmenter()
    result = {}
    for color, mask in segmenter.masks(frame.rgb).items():
        u, v, xyz = depth_world_points(frame, mask)
        if len(xyz) < 80:
            continue
        top_z = float(np.quantile(xyz[:, 2], .8))
        # Remove visible side walls before shape, centroid, and yaw estimation.
        top = np.zeros_like(mask)
        select = np.abs(xyz[:, 2] - top_z) < .0015
        top[v[select], u[select]] = 255
        contours, _ = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 80:
            continue
        shape = classify_shape(contour)
        if shape == "unknown":
            continue
        moments = cv2.moments(contour)
        uv = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        position = pixel_on_plane(*uv, top_z, frame)
        poly = approximate(contour).reshape(-1, 2)
        vertices = pixel_on_plane(poly[:, 0], poly[:, 1], top_z, frame)
        yaw = polygon_orientation(vertices[:, :2], position[:2], shape)
        boundary = contour.reshape(-1, 2)
        world_boundary = pixel_on_plane(boundary[:, 0], boundary[:, 1], top_z, frame)
        position[:2], yaw = refine_polygon_pose(world_boundary[:, :2], position[:2], yaw, shape)
        confidence = float(min(1., cv2.contourArea(contour) / 250) * np.mean(select))
        if confidence < .4:
            continue
        result[shape] = ObservedPose(shape, color, position, yaw, confidence, contour, uv)
    return result
