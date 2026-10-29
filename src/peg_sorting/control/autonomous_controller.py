import time
import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from peg_sorting.control.robot_controller import RobotController
from peg_sorting.control.gripper_controller import GripperController
from peg_sorting.planning.pick_place_planner import execute_pick, execute_place
from peg_sorting.vision.pose_estimation import detect_pegs, depth_world_points
from peg_sorting.vision.color_segmentation import ColorSegmenter
from peg_sorting.vision.socket_detection import detect_sockets, match_sockets
from peg_sorting.vision.visualization import annotate
from peg_sorting.simulation.object_manager import ground_truth, evaluate_placement
from peg_sorting.simulation.scene_manager import SHAPES, COLORS
from peg_sorting.utils.helpers import config
from peg_sorting.utils.logger import save_json


class AutonomousController:
    def __init__(self, env, cameras, output=None):
        self.env, self.cameras, self.output = env, cameras, output
        self.robot, self.gripper = RobotController(env), GripperController(env)
        self.home = env.data.qpos[env.qids].copy()
        self.last_pegs, self.sockets = {}, {}

    def observe(self):
        frame = self.cameras.capture("overhead")
        self.last_pegs = detect_pegs(frame)
        self.sockets = detect_sockets(frame)
        if self.output:
            cv2.imwrite(str(self.output / "initial_perception.png"), cv2.cvtColor(annotate(frame.rgb, self.last_pegs, self.sockets), cv2.COLOR_RGB2BGR))
        return self.last_pegs, self.sockets

    def camera_verification(self, shape, socket):
        overhead = detect_pegs(self.cameras.capture("overhead")).get(shape)
        # The angled RGB-D observation independently checks colored top height.
        angled = self.cameras.capture("angled")
        mask = ColorSegmenter().masks(angled.rgb)[COLORS[shape]]
        _, _, points = depth_world_points(angled, mask)
        top_z = float(np.quantile(points[:, 2], .9)) if len(points) > 30 else None
        expected_top = socket.position[2] - config("simulation")["socket_depth"] + .10
        success = bool(overhead is not None and
                       np.linalg.norm(overhead.position[:2] - socket.position[:2]) < .004 and
                       abs(overhead.position[2] - expected_top) < .003 and
                       top_z is not None and abs(top_z - expected_top) < .005)
        return {"success": success, "overhead_top": overhead.position if overhead else None,
                "angled_top_z_m": top_z, "expected_top_z_m": expected_top}

    def verify(self, shape, socket):
        samples = []
        for _ in range(6):
            self.env.step(seconds=.15)
            samples.append(self.env.data.body(f"peg_{shape}").xpos.copy())
        physical = evaluate_placement(self.env, shape, samples)
        camera = self.camera_verification(shape, socket)
        return {"success": physical["success"] and camera["success"], "physics": physical, "camera": camera}

    def run(self, seed=0):
        start_wall, start_sim = time.perf_counter(), self.env.data.time
        self.env.step(seconds=.6)
        pegs, sockets = self.observe()
        truth = ground_truth(self.env)  # Evaluation only; never passed into planners.
        observations = list(pegs.values())
        truth_labels = list(truth)
        errors, color_correct, shape_correct = {}, 0, 0
        if observations:
            distances = np.array([[np.linalg.norm(p.position - (truth[s]["position"] + [0, 0, .05]))
                                   for s in truth_labels] for p in observations])
            observed_ids, truth_ids = linear_sum_assignment(distances)
            for i, j in zip(observed_ids, truth_ids):
                if distances[i, j] < .025:
                    s, p = truth_labels[j], observations[i]
                    errors[s] = float(distances[i, j])
                    color_correct += p.color == COLORS[s]
                    shape_correct += p.shape == s
        result = {"seed": seed, "robot": "WidowX-250S 6DOF", "grasp_mechanism": "proximity-gated weld with actuated fingers",
                  "detected_pegs": len(pegs), "correct_colors": color_correct, "correct_classifications": shape_correct,
                  "detected_sockets": len(sockets), "pose_errors_m": errors, "placements": [], "successful_grasps": 0}
        try:
            if set(pegs) != set(SHAPES) or set(sockets) != set(SHAPES):
                raise RuntimeError(f"Perception incomplete: pegs={list(pegs)}, sockets={list(sockets)}")
            pairs = dict((p.shape, (p, s)) for p, s in match_sockets(pegs, sockets))
            for shape in SHAPES:
                peg, socket = pairs[shape]
                item_start = self.env.data.time
                item = {"shape": shape, "success": False, "attempts": 0}
                for attempt in range(config("planning")["max_retries"] + 1):
                    item["attempts"] = attempt + 1
                    try:
                        print(f"[{shape}] pick / attempt {attempt + 1}", flush=True)
                        execute_pick(self.robot, self.gripper, peg)
                        print(f"[{shape}] transfer and insert", flush=True)
                        execute_place(self.robot, self.gripper, peg, socket)
                        self.robot.move_joints(self.home)
                        item["verification"] = self.verify(shape, socket)
                        item["success"] = item["verification"]["success"]
                        if item["success"]:
                            item.pop("error", None)
                            break
                        # Retry only from a new observed upright pose, with no active weld.
                        observed = detect_pegs(self.cameras.capture()).get(shape)
                        if observed is None or self.env.held_shape:
                            break
                        peg = observed
                    except (ValueError, RuntimeError) as exc:
                        item["error"] = str(exc)
                        # A failure while holding needs operator recovery; never drop blindly.
                        if self.env.held_shape:
                            break
                        self.robot.stop()
                        observed = detect_pegs(self.cameras.capture()).get(shape)
                        if observed is None:
                            break
                        peg = observed
                item["simulation_seconds"] = self.env.data.time - item_start
                result["placements"].append(item)
                print(f"[{shape}] {'VERIFIED' if item['success'] else 'FAILED'} {item.get('error', '')}", flush=True)
                if not item["success"]:
                    break
        except (RuntimeError, ValueError) as exc:
            result["error"] = str(exc)
        result["successful_grasps"] = self.gripper.grasp_count
        result["successful_placements"] = sum(p["success"] for p in result["placements"])
        result["failed_placements"] = sum(not p["success"] for p in result["placements"])
        result["unattempted_placements"] = 4 - len(result["placements"])
        if result["successful_placements"] == 4:
            result["final_physical_verification"] = {s: evaluate_placement(self.env, s) for s in SHAPES}
            for item in result["placements"]:
                if not result["final_physical_verification"][item["shape"]]["success"]:
                    item["success"] = False
            result["successful_placements"] = sum(p["success"] for p in result["placements"])
            result["failed_placements"] = 4 - result["successful_placements"]
        result["success"] = result["successful_placements"] == 4
        result["success_rate"] = result["successful_placements"] / 4
        result["simulation_seconds"] = self.env.data.time - start_sim
        result["wall_seconds"] = time.perf_counter() - start_wall
        result["average_simulation_seconds_per_peg"] = result["simulation_seconds"] / len(result["placements"]) if result["placements"] else None
        if self.output:
            save_json(self.output / "episode.json", result)
            for name in ("overhead", "angled", "overview"):
                cv2.imwrite(str(self.output / f"final_{name}.png"), cv2.cvtColor(self.cameras.capture(name).rgb, cv2.COLOR_RGB2BGR))
        return result
