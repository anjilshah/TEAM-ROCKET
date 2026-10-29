import numpy as np
import pytest
from peg_sorting.control.teleop_keyboard import KeyboardTeleop
from peg_sorting.simulation.object_manager import evaluate_placement


@pytest.mark.integration
def test_keyboard_move_and_stop(env, camera):
    teleop = KeyboardTeleop(env, camera)
    before, _ = env.site_pose()
    teleop.handle("R")
    after, _ = env.site_pose()
    assert after[2] - before[2] == pytest.approx(.01, abs=.002)
    teleop.handle("F")
    teleop.enqueue(ord(" "))
    assert env.stop_requested
    teleop.poll()
    assert not env.stop_requested
    assert np.allclose(env.data.ctrl[env.actuators], env.data.qpos[env.qids])
    teleop.enqueue(ord("Q"))
    assert not teleop.running


@pytest.mark.integration
def test_keyboard_pick_and_place(env, camera):
    teleop = KeyboardTeleop(env, camera)
    teleop.handle("P")
    shape = env.held_shape
    assert shape in ("square", "triangle", "circle", "hexagon")
    teleop.handle("L")
    env.step(seconds=1)
    assert env.held_shape is None
    assert evaluate_placement(env, shape)["success"]


@pytest.mark.integration
def test_space_interrupts_active_motion(env, camera):
    teleop = KeyboardTeleop(env, camera)
    initial_time = env.data.time
    fired = False

    def simulated_keypress(e):
        nonlocal fired
        if not fired and e.data.time - initial_time > .05:
            fired = True
            teleop.enqueue(ord(" "))

    env.on_step = simulated_keypress
    teleop.keys.append("R")
    teleop.poll()
    assert fired
    assert "stopped by operator" in teleop.message
    assert not env.stop_requested
    assert np.allclose(env.data.ctrl[env.actuators], env.data.qpos[env.qids])
