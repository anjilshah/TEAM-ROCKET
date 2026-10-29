"""Actuated fingers with a disclosed, local-pose-gated MuJoCo weld grasp."""
import mujoco
import numpy as np
from peg_sorting.utils.transforms import wxyz
from peg_sorting.simulation.scene_manager import SHAPES, polygon


class GripperController:
    def __init__(self, env):
        self.env = env
        self.actuator = env.model.actuator("gripper").id
        self.grasp_count = 0

    def open(self):
        e = self.env
        for shape in SHAPES:
            e.data.eq_active[e.model.equality(f"grasp_{shape}").id] = False
        e.held_shape = None
        e.data.ctrl[self.actuator] = .037
        e.step(seconds=.35)

    def close(self, shape=None):
        e = self.env
        if e.held_shape:
            raise ValueError("Gripper already holds an object")
        if shape is None:
            site, _ = e.site_pose()
            candidates = [(np.linalg.norm(e.data.body(f"peg_{s}").xpos + [0, 0, .032] - site), s) for s in SHAPES]
            shape = min(candidates)[1]
        body = e.data.body(f"peg_{shape}")
        site, rotation = e.site_pose()
        expected = body.xpos + body.xmat.reshape(3, 3) @ [0, 0, .032]
        # These checks use local simulated contact geometry, not navigation targets.
        if np.linalg.norm(site - expected) > .009:
            raise ValueError("Grasp rejected: tool is farther than 9 mm from grasp region")
        if np.dot(rotation[:, 0], [0, 0, -1]) < .995 or body.xmat.reshape(3, 3)[2, 2] < .995:
            raise ValueError("Grasp rejected: tool or peg is tilted")
        jaw_width = np.ptp((polygon(shape) @ body.xmat.reshape(3, 3)[:2, :2].T) @ rotation[:2, 1])
        if jaw_width > .058:
            raise ValueError("Grasp rejected: peg exceeds available inner jaw opening")
        if np.linalg.norm(e.data.qvel[e.dofs]) > .07:
            raise ValueError("Grasp rejected: arm is moving")
        # Close to the geometric width; contacts provide a physical jaw stop.
        e.data.ctrl[self.actuator] = max(.015, min(.037, jaw_width / 2 + .008))
        e.step(seconds=.25)
        site, rotation = e.site_pose()
        body = e.data.body(f"peg_{shape}")
        expected = body.xpos + body.xmat.reshape(3, 3) @ [0, 0, .032]
        if np.linalg.norm(site - expected) > .009:
            raise ValueError("Grasp rejected: peg moved outside capture region")
        grip = e.data.body("wx250s/gripper_link")
        R = grip.xmat.reshape(3, 3)
        relative_position = R.T @ (body.xpos - grip.xpos)
        relative_rotation = R.T @ body.xmat.reshape(3, 3)
        eq_id = e.model.equality(f"grasp_{shape}").id
        e.model.eq_data[eq_id, 3:6] = relative_position
        e.model.eq_data[eq_id, 6:10] = wxyz(relative_rotation)
        e.data.eq_active[eq_id] = True
        e.held_shape = shape
        mujoco.mj_forward(e.model, e.data)
        e.step(seconds=.15)
        self.grasp_count += 1
        return self.is_holding(shape)

    def is_holding(self, shape):
        e = self.env
        eq_id = e.model.equality(f"grasp_{shape}").id
        if e.held_shape != shape or not e.data.eq_active[eq_id]:
            return False
        grip, body = e.data.body("wx250s/gripper_link"), e.data.body(f"peg_{shape}")
        relative = grip.xmat.reshape(3, 3).T @ (body.xpos - grip.xpos)
        return bool(np.linalg.norm(relative - e.model.eq_data[eq_id, 3:6]) < .006)

    def release(self):
        self.open()
