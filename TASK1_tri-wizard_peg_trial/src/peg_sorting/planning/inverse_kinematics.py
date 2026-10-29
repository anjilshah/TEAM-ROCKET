from dataclasses import dataclass
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from peg_sorting.utils.helpers import config


@dataclass
class IKResult:
    success: bool
    q: np.ndarray
    position_error: float
    orientation_error: float
    iterations: int


class InverseKinematics:
    def __init__(self, env):
        self.env = env
        self.data = mujoco.MjData(env.model)
        self.settings = config("planning")

    def solve(self, position, rotation, seed=None, restarts=5):
        position, rotation = np.asarray(position, float), np.asarray(rotation, float)
        if position.shape != (3,) or rotation.shape != (3, 3) or not np.isfinite(position).all() or not np.isfinite(rotation).all():
            raise ValueError("Invalid IK target")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or np.linalg.det(rotation) < .999:
            raise ValueError("Target orientation is not a rotation")
        e, data = self.env, self.data
        data.qpos[:] = e.data.qpos
        seed = e.data.qpos[e.qids].copy() if seed is None else np.asarray(seed).copy()
        if seed.shape != (6,) or not np.isfinite(seed).all():
            raise ValueError("Invalid IK seed")
        radial = np.arctan2(position[1], position[0])
        seeds = [seed, [radial, -.5, .8, 0, 1.27, 0],
                 [radial, .1, -.5, 0, 1.97, 0], [radial, -.9, 1.2, 0, 1.27, 0],
                 [radial, .7, -1.1, 0, 1.97, 0], [radial, -.3, .7, np.pi, -1.17, 0]]
        best, best_cost = None, float("inf")
        for initial in seeds[:restarts + 1]:
            q = np.clip(initial, e.limits[:, 0] + 1e-5, e.limits[:, 1] - 1e-5)
            for iteration in range(240):
                data.qpos[e.qids] = q
                mujoco.mj_forward(e.model, data)
                current, current_rot = e.site_pose(data)
                dp = position - current
                dr = Rotation.from_matrix(rotation @ current_rot.T).as_rotvec()
                pe, re = np.linalg.norm(dp), np.linalg.norm(dr)
                result = IKResult(False, q.copy(), float(pe), float(re), iteration + 1)
                if pe + .12 * re < best_cost:
                    best, best_cost = result, pe + .12 * re
                if pe < self.settings["position_tolerance"] and re < self.settings["orientation_tolerance"]:
                    result.success = True
                    return result
                jp, jr = np.zeros((3, e.model.nv)), np.zeros((3, e.model.nv))
                mujoco.mj_jacSite(e.model, data, jp, jr, e.site_id)
                jac = np.vstack((jp[:, e.dofs], .18 * jr[:, e.dofs]))
                error = np.r_[dp, .18 * dr]
                damping = .006
                dq = jac.T @ np.linalg.solve(jac @ jac.T + damping ** 2 * np.eye(6), error)
                norm = np.max(np.abs(dq))
                if norm > .15:
                    dq *= .15 / norm
                q = np.clip(q + dq, e.limits[:, 0] + 1e-5, e.limits[:, 1] - 1e-5)
        return best

    def require(self, position, rotation, seed=None):
        result = self.solve(position, rotation, seed)
        if not result.success:
            raise ValueError(f"Unreachable pose: {np.round(position, 4)}, position error={result.position_error:.4f} m, rotation error={result.orientation_error:.4f} rad")
        return result.q
