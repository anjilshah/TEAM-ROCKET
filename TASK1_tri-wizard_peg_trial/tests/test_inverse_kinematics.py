import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from peg_sorting.planning.inverse_kinematics import InverseKinematics
from peg_sorting.utils.transforms import downward_rotation


def test_official_zero_configuration_and_limits(env):
    data = mujoco.MjData(env.model)
    data.qpos[env.qids] = 0
    mujoco.mj_forward(env.model, data)
    assert np.allclose(data.site("official_tcp").xpos, [.458325, 0, .36065], atol=1e-7)
    official_degrees = [[-180, 180], [-108, 114], [-123, 92], [-180, 180], [-100, 123], [-180, 180]]
    assert np.allclose(np.rad2deg(env.limits), official_degrees, atol=.001)


@pytest.mark.parametrize("position,yaw", [([.18, -.115, .082], .3), ([.34, .205, .22], -.2), ([.31, -.245, .22], 0)])
def test_ik_requires_full_pose(env, position, yaw):
    target = downward_rotation(yaw)
    solution = InverseKinematics(env).solve(position, target)
    assert solution.success
    assert solution.position_error < .0004
    assert solution.orientation_error < .008
    data = mujoco.MjData(env.model)
    data.qpos[:] = env.data.qpos
    data.qpos[env.qids] = solution.q
    mujoco.mj_forward(env.model, data)
    actual, rot = env.site_pose(data)
    assert np.linalg.norm(actual - position) < .0004
    assert np.linalg.norm(Rotation.from_matrix(target @ rot.T).as_rotvec()) < .008
    assert np.all(solution.q >= env.limits[:, 0]) and np.all(solution.q <= env.limits[:, 1])


def test_ik_unreachable(env):
    solution = InverseKinematics(env).solve([2, 0, .3], downward_rotation())
    assert not solution.success


def test_ik_rejects_bad_rotation(env):
    with pytest.raises(ValueError):
        InverseKinematics(env).solve([.2, 0, .2], np.ones((3, 3)))
