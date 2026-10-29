import os
os.environ.setdefault("MUJOCO_GL", "egl")
import pytest
from peg_sorting.simulation.mujoco_env import MujocoEnv
from peg_sorting.simulation.camera_interface import CameraInterface


@pytest.fixture
def env():
    e = MujocoEnv(seed=0)
    e.step(seconds=.6)
    return e


@pytest.fixture
def camera(env):
    c = CameraInterface(env)
    yield c
    c.close()
