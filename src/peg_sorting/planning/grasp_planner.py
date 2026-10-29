from dataclasses import dataclass
import numpy as np
from peg_sorting.utils.helpers import config
from peg_sorting.utils.transforms import downward_rotation


@dataclass
class GraspPlan:
    pregrasp: np.ndarray
    grasp: np.ndarray
    lift: np.ndarray
    rotation: np.ndarray


def plan_grasp(peg):
    settings = config("planning")
    if peg.confidence < .4 or peg.position[2] < .08:
        raise ValueError("Peg pose confidence or height is insufficient")
    grasp = peg.position.copy()
    grasp[2] -= settings["grasp_below_top"]
    pre = grasp + [0, 0, settings["pregrasp_offset"]]
    lift = grasp.copy()
    lift[2] = settings["clearance_z"]
    # Peg widths (44-58 mm) fit the upstream 74 mm maximum aperture.
    return GraspPlan(pre, grasp, lift, downward_rotation(peg.yaw))
