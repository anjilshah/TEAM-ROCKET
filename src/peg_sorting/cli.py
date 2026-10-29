"""Runnable simulation, vision, teleoperation, and autonomous experiment modes."""
import argparse
from pathlib import Path
import time
import cv2
import numpy as np
from peg_sorting.simulation.mujoco_env import MujocoEnv
from peg_sorting.simulation.camera_interface import CameraInterface
from peg_sorting.control.autonomous_controller import AutonomousController
from peg_sorting.control.teleop_keyboard import KeyboardTeleop, INSTRUCTIONS
from peg_sorting.vision.pose_estimation import detect_pegs
from peg_sorting.vision.socket_detection import detect_sockets
from peg_sorting.vision.visualization import annotate, depth_visual
from peg_sorting.utils.helpers import project_root


class DemoRecorder:
    def __init__(self, cameras, path, fps=6):
        self.cameras, self.fps, self.next_time = cameras, fps, 0.
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (1200, 860))
        if not self.writer.isOpened():
            raise RuntimeError(f"Cannot create demo video: {path}")
        self.frames = 0

    def capture(self, env, final_text=""):
        if env.data.time + 1e-6 < self.next_time and not final_text:
            return
        self.next_time = env.data.time + 1 / self.fps
        overview, overhead, angled = [self.cameras.capture(n) for n in ("overview", "overhead", "angled")]
        try:
            pegs = detect_pegs(overhead)
            sockets = detect_sockets(overhead)
            annotated = annotate(overhead.rgb, pegs, sockets)
        except (RuntimeError, ValueError):
            annotated = overhead.rgb
        canvas = np.zeros((860, 1200, 3), np.uint8)
        canvas[:800, :800] = overview.rgb
        canvas[:400, 800:] = cv2.resize(annotated, (400, 400))
        canvas[400:800, 800:] = cv2.resize(angled.rgb, (400, 400))
        for y, label in [(0, "OVERHEAD RGB + detected shapes / poses"), (400, "ANGLED RGB-D: grasp and insertion view")]:
            cv2.rectangle(canvas, (800, y), (1199, y + 26), (20, 24, 30), -1)
            cv2.putText(canvas, label, (809, y + 18), cv2.FONT_HERSHEY_SIMPLEX, .46, (245, 245, 245), 1, cv2.LINE_AA)
        # Include real metric depth previews inside the main camera view.
        for i, f in enumerate((overhead, angled)):
            canvas[670:790, 10 + i * 170:170 + i * 170] = cv2.resize(depth_visual(f.depth), (160, 120))
        text = final_text or f"Tri-Wizard Peg Trial | WidowX-250S | sim {env.data.time:.1f}s | held: {env.held_shape or 'none'}"
        cv2.putText(canvas, text, (16, 828), cv2.FONT_HERSHEY_SIMPLEX, .62, (240, 240, 240), 1)
        cv2.putText(canvas, "Simulation grasp: local proximity/alignment gated weld + actuated fingers", (16, 850), cv2.FONT_HERSHEY_SIMPLEX, .48, (220, 220, 220), 1)
        self.writer.write(cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
        self.frames += 1

    def close(self):
        self.writer.release()


def camera_panel(cameras, vision=False):
    overhead, angled = cameras.capture("overhead"), cameras.capture("angled")
    rgb = overhead.rgb
    if vision:
        rgb = annotate(rgb, detect_pegs(overhead), detect_sockets(overhead))
    return np.vstack((np.hstack((cv2.resize(rgb, (500, 500)), cv2.resize(angled.rgb, (500, 500)))),
                      np.hstack((cv2.resize(depth_visual(overhead.depth), (500, 250)), cv2.resize(depth_visual(angled.depth), (500, 250))))))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["simulation", "vision", "autonomy", "teleop"])
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--seconds", type=float, help="Stop simulation/vision/teleop after this many simulated seconds (headless default: 10)")
    args = parser.parse_args(argv)
    if args.mode == "teleop" and args.headless:
        parser.error("Teleoperation requires the interactive viewer; remove --headless")
    output = args.output or project_root() / "results/runs" / f"{args.mode}_seed{args.seed}_{time.time_ns()}"
    output.mkdir(parents=True, exist_ok=True)
    env = MujocoEnv(args.seed)
    cameras = CameraInterface(env)
    viewer = recorder = None
    teleop = KeyboardTeleop(env, cameras) if args.mode == "teleop" else None
    result = None
    try:
        if not args.headless:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(env.model, env.data, key_callback=teleop.enqueue if teleop else None)
            viewer.cam.lookat[:] = [.22, 0, .12]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.25, 135, -38
        if args.record:
            recorder = DemoRecorder(cameras, args.record)
        last_draw, wall_start = -1., time.perf_counter()

        def on_step(e):
            nonlocal last_draw
            if recorder:
                recorder.capture(e)
            if viewer and e.data.time - last_draw >= .05:
                if not viewer.is_running():
                    raise InterruptedError("Viewer closed")
                viewer.sync()
                if args.mode in ("vision", "teleop", "autonomy"):
                    panel = camera_panel(cameras, args.mode != "teleop")
                    if teleop:
                        cv2.putText(panel, "W/S A/D R/F | O/C | P pick | L place | SPACE stop | Q quit", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                    cv2.imshow("Tri-Wizard: overhead / angled RGB + metric depth", cv2.cvtColor(panel, cv2.COLOR_RGB2BGR))
                    code = cv2.waitKey(1) & 0xff
                    if code != 255 and teleop:
                        teleop.enqueue(code)
                last_draw = e.data.time
                remaining = e.data.time - (time.perf_counter() - wall_start)
                if remaining > 0:
                    time.sleep(min(remaining, .05))
        env.on_step = on_step
        if args.mode == "autonomy":
            result = AutonomousController(env, cameras, output).run(args.seed)
            print(f"Detected {result['detected_pegs']}/4 | Grasps {result['successful_grasps']} | Placements {result['successful_placements']}/4 | Sim {result['simulation_seconds']:.2f}s | Wall {result['wall_seconds']:.2f}s", flush=True)
            if recorder:
                for _ in range(recorder.fps * 2):
                    recorder.capture(env, f"{'SUCCESS' if result['success'] else 'FAILURE'} | {result['successful_placements']}/4 verified | sim {result['simulation_seconds']:.1f}s | wall {result['wall_seconds']:.1f}s")
            if not result["success"]:
                print(result.get("error") or next((p.get("error", str(p.get("verification"))) for p in result["placements"] if not p["success"]), "Verification failed"))
        else:
            if teleop:
                print(INSTRUCTIONS, flush=True)
            duration = args.seconds if args.seconds is not None else (None if viewer else 10.)
            while (viewer is None or viewer.is_running()) and (duration is None or env.data.time < duration):
                if teleop:
                    teleop.poll()
                    if not teleop.running:
                        break
                env.step(steps=1)
            if args.mode == "vision":
                for name in ("overhead", "angled"):
                    f = cameras.capture(name)
                    cv2.imwrite(str(output / f"{name}_rgb.png"), cv2.cvtColor(f.rgb, cv2.COLOR_RGB2BGR))
                    np.save(output / f"{name}_depth_m.npy", f.depth)
                f = cameras.capture()
                pegs, sockets = detect_pegs(f), detect_sockets(f)
                cv2.imwrite(str(output / "detections.png"), cv2.cvtColor(annotate(f.rgb, pegs, sockets), cv2.COLOR_RGB2BGR))
                print(f"Pegs: {list(pegs)}; sockets: {list(sockets)}")
    except InterruptedError as exc:
        env.hold()
        print(str(exc))
        return 130
    finally:
        if recorder:
            recorder.close()
        if viewer:
            viewer.close()
            cv2.destroyAllWindows()
        cameras.close()
    print(f"Artifacts: {output}")
    return 0 if result is None or result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
