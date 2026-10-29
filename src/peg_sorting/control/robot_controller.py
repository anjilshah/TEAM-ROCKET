import numpy as np
from peg_sorting.planning.inverse_kinematics import InverseKinematics
from peg_sorting.planning.trajectory_planner import TrajectoryPlanner


class RobotController:
    def __init__(self, env):
        self.env = env
        self.ik = InverseKinematics(env)
        self.planner = TrajectoryPlanner(env)

    def move_joints(self, goal, allowed_peg=None, minimum_duration=.35):
        e = self.env
        path = self.planner.plan(e.data.qpos[e.qids].copy(), goal, allowed_peg, minimum_duration)
        count = int(np.ceil(path.duration / e.model.opt.timestep))
        for i in range(count):
            e.data.ctrl[e.actuators] = path.sample((i + 1) * e.model.opt.timestep)
            e.step(steps=1)
        e.data.ctrl[e.actuators] = goal
        e.step(seconds=.25)
        if np.max(np.abs(e.data.qpos[e.qids] - goal)) > .012:
            raise RuntimeError("Robot failed to track joint command")

    def move_pose(self, position, rotation, allowed_peg=None, minimum_duration=.35):
        goal = self.ik.require(position, rotation)
        self.move_joints(goal, allowed_peg, minimum_duration)
        actual, rot = self.env.site_pose()
        from scipy.spatial.transform import Rotation
        if np.linalg.norm(actual - position) > .002 or np.linalg.norm(Rotation.from_matrix(rotation @ rot.T).as_rotvec()) > .025:
            raise RuntimeError("Robot failed to track pose command")

    def cartesian(self, position, rotation, allowed_peg=None, speed=.06):
        start, _ = self.env.site_pose()
        distance = np.linalg.norm(np.asarray(position) - start)
        count = max(1, int(np.ceil(distance / .008)))
        # Validate the entire approach before commanding any part of it.
        targets, seed = [], self.env.data.qpos[self.env.qids].copy()
        for p in np.linspace(start, position, count + 1)[1:]:
            goal = self.ik.require(p, rotation, seed)
            self.planner.plan(seed, goal, allowed_peg)
            targets.append(goal)
            seed = goal
        for goal in targets:
            self.move_joints(goal, allowed_peg, max(.12, distance / count / speed))

    def stop(self):
        self.env.hold()
