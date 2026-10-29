import numpy as np
import pytest
import mujoco
from peg_sorting.vision.socket_detection import detect_sockets, match_sockets
from peg_sorting.vision.pose_estimation import detect_pegs


def test_matching_rejects_missing_socket():
    with pytest.raises(ValueError, match="Missing matching sockets"):
        match_sockets({"triangle": object()}, {"circle": object()})


@pytest.mark.integration
def test_socket_pose_and_matching(camera, env):
    frame = camera.capture()
    pegs, sockets = detect_pegs(frame), detect_sockets(frame)
    assert len(match_sockets(pegs, sockets)) == 4
    for shape, socket in sockets.items():
        truth = env.data.site(f"socket_{shape}").xpos
        assert np.linalg.norm(socket.position[:2] - truth[:2]) < .0015
        assert abs(socket.position[2] - .03) < .0001
        if shape != "circle":
            assert abs(socket.yaw) < .025


@pytest.mark.integration
def test_board_detects_after_translation(camera, env):
    env.model.body("socket_board").pos[:] = [.014, -.012, 0]
    mujoco.mj_forward(env.model, env.data)
    sockets = detect_sockets(camera.capture())
    assert len(sockets) == 4
    for shape, socket in sockets.items():
        assert np.linalg.norm(socket.position[:2] - env.data.site(f"socket_{shape}").xpos[:2]) < .0015
