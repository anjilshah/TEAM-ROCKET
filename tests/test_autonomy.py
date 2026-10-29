import pytest
from peg_sorting.control.autonomous_controller import AutonomousController


@pytest.mark.integration
def test_full_episode(env, camera):
    result = AutonomousController(env, camera).run(seed=0)
    assert result["success"], result
    assert result["successful_placements"] == 4
    assert not env.held_shape
    assert all(p["verification"]["camera"]["success"] and p["verification"]["physics"]["success"] for p in result["placements"])
