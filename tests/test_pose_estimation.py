import numpy as np
import pytest
from peg_sorting.simulation.camera_interface import raw_depth_to_metric
from peg_sorting.vision.pose_estimation import pixel_to_world, world_to_pixel, detect_pegs
from peg_sorting.utils.transforms import symmetry_error


def test_buffer_depth_conversion():
    assert np.allclose(raw_depth_to_metric([0, 1], .1, 10), [.1, 10])
    assert np.allclose(raw_depth_to_metric([1, 0], .1, 10, reverse_z=True), [.1, 10])
    z = 2.
    raw = (10 - .1 * 10 / z) / (10 - .1)
    assert raw_depth_to_metric(raw, .1, 10) == pytest.approx(z)


@pytest.mark.parametrize("depth", [0, -1, np.nan, np.inf, 6])
def test_invalid_depth(depth):
    with pytest.raises(ValueError):
        pixel_to_world(10, 10, depth, np.eye(3), np.eye(4))


@pytest.mark.integration
@pytest.mark.parametrize("name", ["overhead", "angled"])
def test_actual_rgbd_camera_projection(camera, env, name):
    frame = camera.capture(name)
    assert frame.rgb.shape == (800, 800, 3)
    assert frame.depth.shape == (800, 800)
    # Verify visible flat tabletop against a known world plane, including y flip.
    for point in [[.05, -.4, 0], [.5, -.1, 0], [.5, .3, 0]]:
        uv, expected_z = world_to_pixel(point, frame.K, frame.camera_to_world)
        u, v = np.round(uv).astype(int)
        assert 0 <= u < 800 and 0 <= v < 800
        z = frame.depth[v, u]
        reconstructed = pixel_to_world(u, v, z, frame.K, frame.camera_to_world)
        assert abs(reconstructed[2]) < .002
        assert np.linalg.norm(reconstructed - point) < .004
        assert z == pytest.approx(float(expected_z), abs=.004)


@pytest.mark.integration
def test_rendered_peg_position_and_yaw(camera, env):
    pegs = detect_pegs(camera.capture())
    assert set(pegs) == {"triangle", "square", "circle", "hexagon"}
    for shape, peg in pegs.items():
        body = env.data.body(f"peg_{shape}")
        assert np.linalg.norm(peg.position - (body.xpos + [0, 0, .05])) < .0015
        yaw = np.arctan2(body.xmat[3], body.xmat[0])
        assert abs(symmetry_error(peg.yaw, yaw, shape)) < .025
