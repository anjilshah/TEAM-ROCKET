"""Conservative waypoint trajectories with discretized robot/carried-peg collision checks."""
from dataclasses import dataclass
import mujoco
import numpy as np
from scipy.interpolate import CubicSpline
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


@dataclass
class JointPath:
    """A continuous joint curve with one smooth start and stop for the whole path."""
    curve: CubicSpline
    duration: float

    def sample(self, time):
        u = np.clip(np.asarray(time) / self.duration, 0, 1)
        return self.curve(10 * u**3 - 15 * u**4 + 6 * u**5)


def derivative_bounds(curve):
    """Exact first/second derivative bounds for a piecewise cubic joint curve."""
    a, b, c, _ = curve.c
    h = np.diff(curve.x)[:, None]
    first_start = c
    first_end = 3 * a * h**2 + 2 * b * h + c
    critical = np.divide(-b, 3 * a, out=np.zeros_like(a), where=np.abs(a) > 1e-14)
    first_critical = 3 * a * critical**2 + 2 * b * critical + c
    first_critical = np.where((critical > 0) & (critical < h), first_critical, 0.)
    first = np.max(np.abs([first_start, first_end, first_critical]))
    second = np.max(np.abs([2 * b, 6 * a * h + 2 * b]))
    return float(first), float(second)


def position_extrema(curve):
    """Include exact spline turning points when enforcing joint limits."""
    progress = list(curve.x)
    for i, width in enumerate(np.diff(curve.x)):
        for joint in range(curve.c.shape[2]):
            a, b, c = curve.c[:3, i, joint]
            for root in np.roots([3 * a, 2 * b, c]):
                if abs(root.imag) < 1e-10 and 0 < root.real < width:
                    progress.append(float(curve.x[i] + root.real))
    return np.asarray(progress)


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

    def plan_path(self, waypoints, allowed_peg=None, minimum_duration=.15):
        points = np.asarray(waypoints, float)
        if points.ndim != 2 or points.shape[1] != 6 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError("Invalid joint path")
        if not np.isfinite(minimum_duration) or minimum_duration <= 0:
            raise ValueError("Invalid path duration")
        e = self.env
        if np.any(points < e.limits[:, 0]) or np.any(points > e.limits[:, 1]):
            raise ValueError("Trajectory rejected: joint limit")
        curve = CubicSpline(np.linspace(0, 1, len(points)), points)
        first, second = derivative_bounds(curve)
        # qdot = q'(s) sdot; qddot = q''(s) sdot² + q'(s) sddot.
        # Quintic progress has max sdot=1.875/T and max |sddot|<5.78/T².
        duration = max(minimum_duration,
                       1.875 * first / self.settings["max_joint_speed"],
                       np.sqrt((1.875**2 * second + 5.78 * first) /
                               self.settings["max_joint_acceleration"]))
        # Check the actual interpolated curve, including spline overshoot between IK nodes.
        count = max(2, int(np.ceil(first / self.settings["sample_joint_step"])) + 1)
        progress = np.unique(np.r_[np.linspace(0, 1, count), position_extrema(curve)])
        for q in curve(progress):
            reason = self.collision(q, allowed_peg)
            if reason:
                raise ValueError(f"Trajectory collision rejected: {reason}")
        return JointPath(curve, float(duration))
