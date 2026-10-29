import numpy as np
import pytest
from peg_sorting.cli import WallClockRate


def test_slow_camera_refresh_does_not_block_viewer_or_key_polling():
    viewer, camera, keyboard = WallClockRate(30), WallClockRate(4), WallClockRate(50)
    counts = [0, 0, 0]
    for now in np.arange(0, 1, .001):
        for i, rate in enumerate((viewer, camera, keyboard)):
            counts[i] += rate.ready(float(now))
    assert 28 <= counts[0] <= 31
    assert counts[1] == 4
    assert 48 <= counts[2] <= 51


def test_expensive_frame_leaves_a_physics_window_after_rendering():
    camera = WallClockRate(4)
    assert camera.ready(0)
    camera.completed(.4)  # Software rendering took longer than one frame period.
    assert not camera.ready(.5)
    assert camera.ready(.65)


def test_missed_refreshes_do_not_accumulate_a_render_backlog():
    rate = WallClockRate(10)
    assert rate.ready(0)
    assert rate.ready(4)
    assert not rate.ready(4.001)
    assert rate.ready(4.101)


@pytest.mark.parametrize("frequency", [0, -1, float("nan"), float("inf")])
def test_invalid_refresh_frequency(frequency):
    with pytest.raises(ValueError, match="finite and positive"):
        WallClockRate(frequency)
