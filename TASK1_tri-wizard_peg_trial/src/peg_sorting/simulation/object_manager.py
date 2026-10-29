"""Ground truth is exposed only for grasp gating and independent evaluation."""
import numpy as np
from peg_sorting.simulation.scene_manager import SHAPES


def ground_truth(env):
    return {shape: {"position": env.data.body(f"peg_{shape}").xpos.copy(),
                    "rotation": env.data.body(f"peg_{shape}").xmat.reshape(3, 3).copy()}
            for shape in SHAPES}


def evaluate_placement(env, shape, samples=None):
    peg = env.data.body(f"peg_{shape}")
    center = env.data.site(f"socket_{shape}").xpos
    xy_error = float(np.linalg.norm(peg.xpos[:2] - center[:2]))
    bottom_error = float(abs(peg.xpos[2] - .05 - center[2]))
    upright = float(peg.xmat.reshape(3, 3)[2, 2])
    dof = env.model.joint(f"free_{shape}").dofadr[0]
    velocity = env.data.qvel[dof:dof + 6]
    linear_speed = float(np.linalg.norm(velocity[:3]))
    angular_speed = float(np.linalg.norm(velocity[3:]))
    from peg_sorting.utils.transforms import symmetry_error
    yaw = np.arctan2(peg.xmat[3], peg.xmat[0])
    orientation_error = abs(symmetry_error(yaw, 0, shape))
    stable = samples is None or np.max(np.linalg.norm(np.asarray(samples) - samples[-1], axis=1)) < .002
    released = not bool(env.data.eq_active[env.model.equality(f"grasp_{shape}").id])
    return {"success": bool(released and xy_error < .003 and bottom_error < .0025 and
                            upright > .995 and orientation_error < .045 and linear_speed < .01 and angular_speed < .10 and stable),
            "released": released, "xy_error_m": xy_error, "bottom_error_m": bottom_error,
            "orientation_error_rad": float(orientation_error), "linear_speed_m_s": linear_speed,
            "angular_speed_rad_s": angular_speed, "stable": bool(stable)}
