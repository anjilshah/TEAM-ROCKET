import mujoco
import numpy as np
import pytest
from peg_sorting.simulation.object_manager import evaluate_placement


@pytest.mark.integration
@pytest.mark.parametrize("shape", ["square", "triangle", "circle", "hexagon"])
def test_physical_socket_is_open(env, shape):
    # Controlled free-fall fixture test; autonomy never sets peg qpos.
    center = env.data.site(f"socket_{shape}").xpos
    adr = env.model.joint(f"free_{shape}").qposadr[0]
    env.data.qpos[adr:adr + 7] = [*center[:2], .09, 1, 0, 0, 0]
    env.data.qvel[:] = 0
    mujoco.mj_forward(env.model, env.data)
    env.step(seconds=2)
    result = evaluate_placement(env, shape)
    assert result["success"], result


def test_near_socket_is_not_inserted(env):
    shape = "square"
    center = env.data.site("socket_square").xpos
    adr = env.model.joint("free_square").qposadr[0]
    env.data.qpos[adr:adr + 7] = [*center[:2], .081, 1, 0, 0, 0]
    mujoco.mj_forward(env.model, env.data)
    assert not evaluate_placement(env, shape)["success"]


@pytest.mark.integration
def test_wrong_shape_is_physically_blocked(env):
    center = env.data.site("socket_triangle").xpos
    adr = env.model.joint("free_square").qposadr[0]
    env.data.qpos[adr:adr + 7] = [*center[:2], .10, 1, 0, 0, 0]
    env.data.qvel[:] = 0
    mujoco.mj_forward(env.model, env.data)
    env.step(seconds=2)
    assert env.data.body("peg_square").xpos[2] > .070
