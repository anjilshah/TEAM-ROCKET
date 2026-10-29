"""Task-space waypoints based solely on perceived peg/socket poses."""
import numpy as np
from peg_sorting.planning.grasp_planner import plan_grasp
from peg_sorting.utils.helpers import config
from peg_sorting.utils.transforms import downward_rotation, symmetry_error


def placement_targets(peg, socket):
    settings = config("planning")
    # Preserve the estimated grasp offset while rotating into socket symmetry.
    yaw = peg.yaw - symmetry_error(peg.yaw, socket.yaw, peg.shape)
    if peg.shape == "circle":
        yaw = peg.yaw
    high = np.array([*socket.position[:2], settings["clearance_z"]])
    # Rim at 30 mm, floor 28 mm below. Leave 2 mm for gravity to settle.
    bottom = socket.position[2] - config("simulation")["socket_depth"] + .002
    low = np.array([*socket.position[:2], bottom + .10 - settings["grasp_below_top"]])
    return high, low, downward_rotation(yaw)


def execute_pick(robot, gripper, peg):
    if robot.env.held_shape:
        raise ValueError("Place or release the held peg before picking another")
    grasp = plan_grasp(peg)
    # Reject unreachable lift targets before the fingers capture an object.
    seed = robot.env.data.qpos[robot.env.qids].copy()
    for target in (grasp.pregrasp, grasp.grasp, grasp.lift):
        seed = robot.ik.require(target, grasp.rotation, seed)
    gripper.open()
    robot.move_pose(grasp.pregrasp, grasp.rotation, peg.shape)
    robot.cartesian(grasp.grasp, grasp.rotation, peg.shape)
    robot.env.step(seconds=config("planning")["settle_seconds"])
    if not gripper.close(peg.shape):
        raise RuntimeError("Grasp attachment failed")
    robot.cartesian(grasp.lift, grasp.rotation, peg.shape)
    if not gripper.is_holding(peg.shape):
        raise RuntimeError("Peg did not remain with the gripper after lifting")
    return grasp


def execute_place(robot, gripper, peg, socket):
    high, low, rotation = placement_targets(peg, socket)
    robot.move_pose(high, rotation)
    # Travel above the rim at approach speed; use the lower speed for insertion.
    insertion_entry = low + [0, 0, config("simulation")["socket_depth"] + config("planning")["insertion_entry_clearance"]]
    robot.cartesian(insertion_entry, rotation)
    robot.cartesian(low, rotation, speed=config("planning")["placement_speed"])
    gripper.release()
    robot.cartesian(high, rotation)
