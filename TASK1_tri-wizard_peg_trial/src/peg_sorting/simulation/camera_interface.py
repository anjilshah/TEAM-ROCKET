from dataclasses import dataclass
import mujoco
import numpy as np
from peg_sorting.utils.helpers import config


@dataclass
class RGBDFrame:
    rgb: np.ndarray
    depth: np.ndarray
    K: np.ndarray
    camera_to_world: np.ndarray
    camera_name: str


def raw_depth_to_metric(buffer, near, far, reverse_z=False):
    """For low-level OpenGL buffers only. Renderer.render already converts depth."""
    if not (0 < near < far):
        raise ValueError("Invalid clipping planes")
    buffer = np.asarray(buffer, float)
    if np.any((buffer < 0) | (buffer > 1) | ~np.isfinite(buffer)):
        raise ValueError("Depth buffer must be finite in [0, 1]")
    if reverse_z:
        buffer = 1 - buffer
    return near * far / (far - buffer * (far - near))


class CameraInterface:
    def __init__(self, env, width=None, height=None):
        self.env = env
        settings = config("cameras")
        self.width, self.height = width or settings["width"], height or settings["height"]
        self.renderer = mujoco.Renderer(env.model, height=self.height, width=self.width)

    def intrinsics(self, name):
        cam = self.env.model.camera(name)
        f = .5 * self.height / np.tan(np.deg2rad(cam.fovy[0]) / 2)
        return np.array([[f, 0, (self.width - 1) / 2], [0, f, (self.height - 1) / 2], [0, 0, 1.]])

    def extrinsics(self, name):
        cam = self.env.data.camera(name)
        transform = np.eye(4)
        # OpenCV x-right/y-down/z-forward -> MuJoCo x-right/y-up/z-backward.
        transform[:3, :3] = cam.xmat.reshape(3, 3) @ np.diag([1, -1, -1])
        transform[:3, 3] = cam.xpos
        return transform

    def capture(self, name="overhead"):
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.env.data, camera=name)
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        # Pinned MuJoCo 3.3.5 Renderer returns metric optical Z and flips both
        # images to top-left origin internally. Never convert or flip a second time.
        return RGBDFrame(rgb, depth, self.intrinsics(name), self.extrinsics(name), name)

    def close(self):
        self.renderer.close()
