import numpy as np
import pytest
from peg_sorting.planning.trajectory_planner import TrajectoryPlanner
from peg_sorting.planning.grasp_planner import plan_grasp
from peg_sorting.vision.pose_estimation import ObservedPose
from peg_sorting.control.gripper_controller import GripperController


def test_trajectory_limits_and_smoothness(env):
    start = env.data.qpos[env.qids].copy()
    goal = start.copy()
    goal[0] += .15
    planner = TrajectoryPlanner(env)
    path = planner.plan(start, goal)
    times = np.linspace(0, path.duration, 1001)
    qs = np.array([path.sample(t) for t in times])
    dt = times[1] - times[0]
    velocity = np.gradient(qs, dt, axis=0)
    acceleration = np.gradient(velocity, dt, axis=0)
    assert np.max(np.abs(velocity)) <= .65 + .001
    assert np.max(np.abs(acceleration)) <= 1.4 + .001
    assert np.allclose(qs[0], start) and np.allclose(qs[-1], goal)
    bad = goal.copy()
    bad[1] = 10
    with pytest.raises(ValueError, match="joint limit"):
        planner.plan(start, bad)


def test_grasp_geometry_and_confidence():
    peg = ObservedPose("square", "red", np.array([.2, -.1, .1]), .3, .95, np.empty((0, 1, 2)), (0, 0))
    grasp = plan_grasp(peg)
    assert grasp.pregrasp[2] > grasp.grasp[2]
    assert grasp.lift[2] - .082 > .10
    assert np.allclose(grasp.rotation[:, 0], [0, 0, -1])
    peg.confidence = .1
    with pytest.raises(ValueError):
        plan_grasp(peg)


def test_remote_grasp_is_rejected(env):
    gripper = GripperController(env)
    with pytest.raises(ValueError, match="farther"):
        gripper.close("circle")
    assert not env.data.eq_active[env.model.equality("grasp_circle").id]


def test_actual_table_collision_is_rejected(env):
    from peg_sorting.planning.inverse_kinematics import InverseKinematics
    from peg_sorting.utils.transforms import downward_rotation
    target = InverseKinematics(env).require([.18, -.115, -.01], downward_rotation())
    planner = TrajectoryPlanner(env)
    assert "table" in planner.collision(target, allowed_peg="square")
    with pytest.raises(ValueError, match="Trajectory collision rejected"):
        planner.plan(env.data.qpos[env.qids], target, allowed_peg="square")
