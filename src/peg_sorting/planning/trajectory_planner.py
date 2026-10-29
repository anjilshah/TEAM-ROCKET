"""Conservative waypoint trajectories with discretized robot/carried-peg collision checks."""
from dataclasses import dataclass
import mujoco
import numpy as np
from peg_sorting.utils.helpers import config
from peg_sorting.utils.transforms import matrix_from_wxyz, wxyz


@dataclass
class JointSegment:
    start: np.ndarray
    goal: np.ndarray
    duration: float

    def sample(self, time):
        u = np.clip(time / self.duration, 0, 1)
        blend = 10 * u**3 - 15 * u**4 + 6 * u**5
        return self.start + blend * (self.goal - self.start)


class TrajectoryPlanner:
    def __init__(self, env):
        self.env = env
        self.settings = config("planning")
        self.data = mujoco.MjData(env.model)
        m = env.model
        self.robot_bodies = {i for i in range(m.nbody) if (m.body(i).name or "").startswith("wx250s/")}
        self.grip_bodies = {m.body(n).id for n in ["wx250s/gripper_link", "wx250s/left_finger_link", "wx250s/right_finger_link"]}

    def collision(self, q, allowed_peg=None):
        e, d, m = self.env, self.data, self.env.model
        if np.any(q < e.limits[:, 0]) or np.any(q > e.limits[:, 1]):
            return "joint limit"
        d.qpos[:] = e.data.qpos
        d.qvel[:] = 0
        d.qpos[e.qids] = q
        # Planning runs on a separate MjData; simulation objects are never teleported.
        mujoco.mj_forward(m, d)
        held_body = -1
        if e.held_shape:
            held_body = m.body(f"peg_{e.held_shape}").id
            eq = m.equality(f"grasp_{e.held_shape}")
            rel = m.eq_data[eq.id]
            grip = d.body("wx250s/gripper_link")
            R = grip.xmat.reshape(3, 3)
            pos = grip.xpos + R @ rel[3:6]
            rot = R @ matrix_from_wxyz(rel[6:10])
            adr = m.joint(f"free_{e.held_shape}").qposadr[0]
            d.qpos[adr:adr + 7] = np.r_[pos, wxyz(rot)]
            mujoco.mj_forward(m, d)
        allowed_body = m.body(f"peg_{allowed_peg}").id if allowed_peg else -1
        for contact in d.contact:
            if contact.dist > -.0007:
                continue
            ga, gb = int(contact.geom1), int(contact.geom2)
            a, b = int(m.geom_bodyid[ga]), int(m.geom_bodyid[gb])
            if a not in self.robot_bodies and b not in self.robot_bodies and held_body not in (a, b):
                continue
            # The fixed robot base is mounted on the tabletop.
            if m.body("wx250s/base_link").id in (a, b) and (m.geom(ga).name == "table" or m.geom(gb).name == "table"):
                continue
            if (a in self.grip_bodies and b in (held_body, allowed_body)) or (b in self.grip_bodies and a in (held_body, allowed_body)):
                continue
            return f"{m.geom(ga).name or m.body(a).name} / {m.geom(gb).name or m.body(b).name}: {contact.dist:.4f} m"
        return None

    def plan(self, start, goal, allowed_peg=None, minimum_duration=.35):
        start, goal = np.asarray(start), np.asarray(goal)
        if start.shape != (6,) or goal.shape != (6,) or not np.isfinite(np.r_[start, goal]).all():
            raise ValueError("Invalid joint trajectory")
        if np.any(start < self.env.limits[:, 0]) or np.any(start > self.env.limits[:, 1]) or np.any(goal < self.env.limits[:, 0]) or np.any(goal > self.env.limits[:, 1]):
            raise ValueError("Trajectory rejected: joint limit")
        delta = np.max(np.abs(goal - start))
        duration = max(minimum_duration, 1.875 * delta / self.settings["max_joint_speed"],
                       np.sqrt(5.78 * delta / self.settings["max_joint_acceleration"]))
        segment = JointSegment(start.copy(), goal.copy(), float(duration))
        count = max(2, int(np.ceil(delta / self.settings["sample_joint_step"])) + 1)
        for alpha in np.linspace(0, 1, count):
            reason = self.collision(start + alpha * (goal - start), allowed_peg)
            if reason:
                raise ValueError(f"Trajectory collision rejected: {reason}")
        return segment
