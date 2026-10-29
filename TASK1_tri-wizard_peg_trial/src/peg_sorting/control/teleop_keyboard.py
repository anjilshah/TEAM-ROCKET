"""Viewer keyboard commands. Space stops commanded simulation motion only."""
from collections import deque
import numpy as np
from peg_sorting.control.robot_controller import RobotController
from peg_sorting.control.gripper_controller import GripperController
from peg_sorting.planning.pick_place_planner import execute_pick, execute_place
from peg_sorting.vision.pose_estimation import detect_pegs
from peg_sorting.vision.socket_detection import detect_sockets

INSTRUCTIONS = "W/S forward/back | A/D left/right | R/F up/down | O open | C close | P pick nearest | L place held | SPACE stop motion | Q quit"


class KeyboardTeleop:
    def __init__(self, env, cameras):
        self.env, self.cameras = env, cameras
        self.robot, self.gripper = RobotController(env), GripperController(env)
        self.keys, self.running = deque(), True
        self.held_observation = None
        self.message = INSTRUCTIONS

    def enqueue(self, code):
        key = chr(code).upper() if 0 <= code < 256 else ""
        if key in (" ", "Q"):
            self.env.stop_requested = True
            self.keys.clear()
        if key == "Q":
            self.running = False
        self.keys.append(key)

    def handle(self, key):
        self.env.stop_requested = False
        if key in (" ", "Q"):
            self.robot.stop()
            return
        position, rotation = self.env.site_pose()
        offsets = {"W": [.01, 0, 0], "S": [-.01, 0, 0], "A": [0, .01, 0],
                   "D": [0, -.01, 0], "R": [0, 0, .01], "F": [0, 0, -.01]}
        if key in offsets:
            target = position + offsets[key]
            if not (-.5 < target[0] < .55 and -.5 < target[1] < .5 and .06 < target[2] < .55):
                raise ValueError("Teleoperation workspace limit")
            self.robot.move_pose(target, rotation)
        elif key == "O":
            self.gripper.open()
            self.held_observation = None
        elif key == "C":
            self.gripper.close()
        elif key == "P":
            pegs = detect_pegs(self.cameras.capture())
            if not pegs:
                raise ValueError("No unoccluded peg detected")
            peg = min(pegs.values(), key=lambda p: np.linalg.norm(p.position - position))
            execute_pick(self.robot, self.gripper, peg)
            self.held_observation = peg
        elif key == "L":
            if self.held_observation is None or not self.env.held_shape:
                raise ValueError("Use P to pick a visually identified peg first")
            sockets = detect_sockets(self.cameras.capture())
            shape = self.env.held_shape
            if shape not in sockets:
                raise ValueError("Matching socket is occluded")
            execute_place(self.robot, self.gripper, self.held_observation, sockets[shape])
            self.held_observation = None

    def poll(self):
        if not self.keys:
            return
        key = self.keys.popleft()
        try:
            self.handle(key)
            self.message = f"Command: {key!r}"
        except (ValueError, RuntimeError, InterruptedError) as exc:
            self.env.stop_requested = False
            self.robot.stop()
            self.message = str(exc)
        print(self.message, flush=True)
