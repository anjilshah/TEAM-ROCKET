# Troubleshooting

## Missing imports or environment

Use `.venv/bin/python` and the launch scripts, not Ubuntu's system Python. Run `.venv/bin/python -m pip check`. Restore dependencies using `requirements-lock.txt` and install the source with `--no-build-isolation --no-deps -e .`. If venv creation reports missing `ensurepip`, ask an Ubuntu administrator to install `python3-venv`.

## Rendering or GUI errors

First try `bash launch/run_simulation.sh --headless --seconds 1`. Launchers select EGL for `--headless` and GLFW for GUI unless `MUJOCO_GL` is explicitly set. For GUI, use the local desktop session and ensure `DISPLAY` or `WAYLAND_DISPLAY` is valid. A user override such as `MUJOCO_GL=egl` persists and can affect GUI behavior; unset it when diagnosing.

Mesa EGL worked in the validated environment without a GPU device. If EGL initialization fails, check host `libEGL`/Mesa installation. Do not reinstall Python libraries blindly. OSMesa is an alternative only on hosts that already have its native libraries. OpenCV's Qt/XCB errors concern display/native library setup. GUI native-library installation needs administrator approval; headless mode may still work.

A GLFW message saying Wayland cannot provide window position was observed during a successful GUI launch. It did not prevent simulation.

## Missing robot files or XML failures

The repository includes the small licensed mesh set. Run `scripts/fetch_robot.py` only to restore missing source files. It refuses to replace files whose content differs from the pinned upstream revision. Do not silently overwrite local model changes. Regenerate scene XML with `scripts/generate_demo_scene.py`; runtime construction uses the same generator in memory. Keep `simulation/scenes/` and `simulation/robots/` in their relative locations when opening the committed XML directly.

## Incorrect colors, shapes, or socket poses

Run `bash launch/run_vision.sh --headless --seconds 1 --output results/vision_debug`. Inspect the real RGB frames, `.npy` metric depth, and detection overlay. HSV thresholds and RGBA colors must agree in `config/colors.yaml`. Very bright illumination can reduce saturation. The board's magenta perimeter must be visible.

Peg classification operates on the depth-isolated top face. A tilted/occluded peg may be rejected. Socket detection needs the neutral rim surface and visible apertures; a robot covering the board can bias classification. The default parking pose points away from the board. Do not replace failed perception with saved object positions.

## Unreachable or rejected motion

The manufacturer reach number is not a guarantee that every downward-oriented tool pose is reachable. Tool length, joint limits, and height matter. Keep randomized cells in the tested region or validate new full-pose IK targets. The configured 0.22 m transfer height clears upright pegs while staying within this layout's reach.

A trajectory rejection includes the colliding geometry names and penetration depth. Resolve the workspace/layout/waypoints rather than disabling collision checks. The planner is conservative and may reject a feasible route. A failed held-object motion ends the autonomous episode and leaves the weld active; inspect and restart the simulation to recover safely. Restarting creates a new seeded scene. No external hardware is controlled.

## Failed grasp or verification

The gripper will not attach remotely. It requires proximity, upright alignment, low arm velocity, and a fitting aperture. Verify top-face pose/depth calibration and contact clearance. The weld is an intentional simulation aid, not friction-only grasp validation.

Review `episode.json` for separate camera and physics checks. A peg sitting on the rim fails bottom-height verification even if XY is close. Triangle/hexagon meshes can exhibit tiny contact vibrations; linear and angular velocity use separate units and thresholds, plus a 0.9 s position stability window. Do not relabel a failed trial as successful.

## Keyboard focus and stop

Click the viewer or camera panel. Commands are discrete 1 cm steps. P/L are vision-guided macros; L needs a peg picked using P. Space interrupts the active macro and holds joint positions. It does not act as a hardware emergency stop. Q or closing the viewer exits. Physical human keypress rehearsal is described in the demo guide; the automated suite exercises the command handlers against the simulator.

## Publication

Nothing has been uploaded. The target GitHub repository returned 404 through the connector; it may not exist or may be inaccessible. GitHub CLI is not installed here. After owner approval, create the public repository in GitHub, authenticate Git, and push the prepared local branch. Never include `.venv`, local credentials, or large temporary recordings.
