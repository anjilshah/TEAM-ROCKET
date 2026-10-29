import argparse
from pathlib import Path
import time
import numpy as np
from peg_sorting.simulation.mujoco_env import MujocoEnv
from peg_sorting.simulation.camera_interface import CameraInterface
from peg_sorting.control.autonomous_controller import AutonomousController
from peg_sorting.utils.logger import save_csv, save_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("results/evaluation"))
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("--trials must be positive")
    rows = []
    for seed in range(args.seed, args.seed + args.trials):
        env = MujocoEnv(seed)
        cameras = CameraInterface(env)
        out = args.output / f"seed_{seed}"
        out.mkdir(parents=True, exist_ok=True)
        try:
            result = AutonomousController(env, cameras, out).run(seed)
        finally:
            cameras.close()
        errors = list(result["pose_errors_m"].values())
        rows.append({"seed": seed, "detected_pegs": result["detected_pegs"], "correct_colors": result["correct_colors"],
                     "correct_classifications": result["correct_classifications"], "pose_error_mean_m": np.mean(errors) if errors else float("nan"),
                     "successful_grasps": result["successful_grasps"], "successful_placements": result["successful_placements"],
                     "failed_placements": result["failed_placements"], "unattempted_placements": result["unattempted_placements"],
                     "success_rate": result["success_rate"], "complete_trial": result["success"],
                     "simulation_seconds": result["simulation_seconds"], "wall_seconds": result["wall_seconds"],
                     "average_simulation_seconds_per_peg": result["average_simulation_seconds_per_peg"]})
        save_csv(args.output / "trials.csv", rows)
    summary = {"trials": args.trials, "seeds": [r["seed"] for r in rows],
               "placement_success_rate": sum(r["successful_placements"] for r in rows) / (4 * args.trials),
               "complete_trial_rate": sum(r["complete_trial"] for r in rows) / args.trials,
               "mean_simulation_seconds": float(np.mean([r["simulation_seconds"] for r in rows])),
               "mean_wall_seconds": float(np.mean([r["wall_seconds"] for r in rows])),
               "mean_pose_error_m": float(np.mean([r["pose_error_mean_m"] for r in rows]))}
    save_json(args.output / "summary.json", summary)
    print(summary)
    return 0 if summary["complete_trial_rate"] == 1 else 1


if __name__ == "__main__":
    raise SystemExit(main())
