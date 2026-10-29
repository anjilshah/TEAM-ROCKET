import numpy as np
from peg_sorting.planning.inverse_kinematics import InverseKinematics
from peg_sorting.planning.trajectory_planner import TrajectoryPlanner
from peg_sorting.utils.helpers import config


class RobotController:
    def __init__(self, env):
        self.env = env
        self.ik = InverseKinematics(env)
        self.planner = TrajectoryPlanner(env)
        self.settings = config("planning")

    def move_joints(self, goal, allowed_peg=None, minimum_duration=.35):
        e = self.env
        path = self.planner.plan(e.data.qpos[e.qids].copy(), goal, allowed_peg, minimum_duration)
        self._execute(path, goal)

    def _execute(self, path, goal):
        e = self.env
        count = int(np.ceil(path.duration / e.model.opt.timestep))
        for i in range(count):
            e.data.ctrl[e.actuators] = path.sample((i + 1) * e.model.opt.timestep)
            e.step(steps=1)
        e.data.ctrl[e.actuators] = goal
        # Settle at the end of a complete motion, never at every Cartesian IK node.
        elapsed = 0.
        while elapsed < self.settings["tracking_settle_max_seconds"]:
            e.step(seconds=.01)
            elapsed += .01
            if (elapsed >= self.settings["tracking_settle_min_seconds"] and
                    np.max(np.abs(e.data.qpos[e.qids] - goal)) < .002 and
                    np.linalg.norm(e.data.qvel[e.dofs]) < .04):
                break
        if np.max(np.abs(e.data.qpos[e.qids] - goal)) > .012:
            raise RuntimeError("Robot failed to track joint command")
        if np.linalg.norm(e.data.qvel[e.dofs]) >= .04:
            raise RuntimeError("Robot did not settle before the next motion")

    def move_pose(self, position, rotation, allowed_peg=None, minimum_duration=.35):
        goal = self.ik.require(position, rotation)
        self.move_joints(goal, allowed_peg, minimum_duration)
        self._check_pose(position, rotation)

    def _check_pose(self, position, rotation):
        actual, rot = self.env.site_pose()
        from scipy.spatial.transform import Rotation
        if np.linalg.norm(actual - position) > .002 or np.linalg.norm(Rotation.from_matrix(rotation @ rot.T).as_rotvec()) > .025:
            raise RuntimeError("Robot failed to track pose command")

    def cartesian(self, position, rotation, allowed_peg=None, speed=None):
        speed = self.settings["approach_speed"] if speed is None else speed
        if not np.isfinite(speed) or speed <= 0:
            raise ValueError("Cartesian speed must be finite and positive")
        start, _ = self.env.site_pose()
        distance = np.linalg.norm(np.asarray(position) - start)
        count = max(1, int(np.ceil(distance / self.settings["approach_step"])))
        # Validate the entire approach before commanding any part of it.
        seed = self.env.data.qpos[self.env.qids].copy()
        targets = [seed]
        for p in np.linspace(start, position, count + 1)[1:]:
            goal = self.ik.require(p, rotation, seed)
            targets.append(goal)
            seed = goal
        # Keep all IK/collision samples while removing intermediate start/stop delays.
        path = self.planner.plan_path(targets, allowed_peg, max(.15, 1.875 * distance / speed))
        self._execute(path, targets[-1])
        self._check_pose(position, rotation)

    def stop(self):
        self.env.hold()
