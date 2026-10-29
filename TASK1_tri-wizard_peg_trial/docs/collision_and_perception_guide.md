# Finding objects and recovering from collisions

This is the practical diagnosis guide for the current WidowX-250S simulation. Use it together with the [troubleshooting summary](troubleshooting.md), [validation record](validation.md), and [architecture](architecture.md). It explains what to inspect, which file controls each behavior, and how to prove a change helped.

All commands below run from the repository root using the existing `.venv`. Use a new directory under `results/runs/` for each investigation so the measured baseline in `results/evaluation/` and current results in `results/performance/` remain intact. Restart the Python process after changing YAML or source: an existing viewer retains its loaded model and controller.

## 1. Identify the failing stage first

The task order is camera capture → color masks → top-face shape → world pose → socket detection/matching → full-pose IK → collision-checked trajectory → grasp → transport → insertion → release → verification. A late collision can originate in an earlier pose error.

| Symptom | First evidence to inspect | Main implementation |
| --- | --- | --- |
| A peg is visible but not detected | RGB image and that color's binary mask | [color segmentation](../src/peg_sorting/vision/color_segmentation.py) |
| Color exists but shape is missing/wrong | Depth-isolated top contour and contour approximation | [pose estimation](../src/peg_sorting/vision/pose_estimation.py), [shape detection](../src/peg_sorting/vision/shape_detection.py) |
| Position is mirrored, scaled, or vertically wrong | Depth units, camera intrinsics/extrinsics, projection tests | [camera interface](../src/peg_sorting/simulation/camera_interface.py) |
| Sockets disappear or have biased centers | Magenta perimeter, rim plane, aperture contours, arm occlusion | [socket detection](../src/peg_sorting/vision/socket_detection.py) |
| `Unreachable pose` | Both IK residuals, tool orientation, limits, target height | [inverse kinematics](../src/peg_sorting/planning/inverse_kinematics.py) |
| `Trajectory collision rejected` | The two reported geometry/body names and phase | [trajectory planner](../src/peg_sorting/planning/trajectory_planner.py) |
| `Grasp rejected` | Tool/peg separation, uprightness, aperture, arm velocity | [gripper controller](../src/peg_sorting/control/gripper_controller.py) |
| Peg reaches a socket but verification fails | Separate `physics` and `camera` checks in `episode.json` | [autonomy](../src/peg_sorting/control/autonomous_controller.py), [physical evaluation](../src/peg_sorting/simulation/object_manager.py) |
| Movement is slow or display is choppy | Simulated time versus wall time, camera refresh, endpoint waits | [robot controller](../src/peg_sorting/control/robot_controller.py), [CLI](../src/peg_sorting/cli.py) |

Change one layer at a time. Save its before/after evidence and run the relevant tests before testing the complete task.

## 2. Capture a repeatable observation

Start with seed 0 and the default parking pose:

```bash
.venv/bin/python -m pip check
bash launch/run_simulation.sh --headless --seconds 1
bash launch/run_vision.sh --headless --seed 0 --seconds 1 \
  --output results/runs/vision_debug_seed0
```

The vision command saves:

- `overhead_rgb.png` and `angled_rgb.png`: real rendered color images.
- `overhead_depth_m.npy` and `angled_depth_m.npy`: metric optical-Z arrays.
- `detections.png`: overhead contour, bounding-box, shape, and position overlay.

Inspect both RGB views. Each peg top and the board perimeter should be visible. An object hidden by the robot is an occlusion issue before it is a threshold issue. The robot parks toward −Y to avoid covering the board; its initial arm configuration is set in `MujocoEnv.__init__` in [mujoco_env.py](../src/peg_sorting/simulation/mujoco_env.py).

For a live view, run `bash launch/run_vision.sh`. On a slower machine, use `--camera-fps 2`. Panel refresh does not change the physics timestep or the observations captured by autonomous verification.

### Save the individual color masks

After the capture above, this diagnostic reads its image and saves one mask per color:

```bash
.venv/bin/python - <<'PY'
from pathlib import Path
import cv2
import numpy as np
from peg_sorting.vision.color_segmentation import ColorSegmenter

out = Path("results/runs/vision_debug_seed0")
bgr = cv2.imread(str(out / "overhead_rgb.png"))
if bgr is None:
    raise SystemExit("Run the vision capture command first")
rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
segmenter = ColorSegmenter()
for color, mask in segmenter.masks(rgb).items():
    cv2.imwrite(str(out / f"mask_{color}.png"), mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(color, "pixels=", np.count_nonzero(mask),
          "contour_areas=", [round(cv2.contourArea(c), 1) for c in contours])
depth = np.load(out / "overhead_depth_m.npy")
valid = np.isfinite(depth) & (depth > 0) & (depth < 5)
print("depth shape:", depth.shape, "valid fraction:", float(valid.mean()))
if valid.any():
    print("metric depth percentiles:", np.quantile(depth[valid], [.05, .5, .95]))
PY
```

White pixels belong to a color mask; black pixels do not. These are color masks, not the final top-face contours. The pose pipeline additionally removes visible side walls using depth.

## 3. Recover missing colors and incorrect shapes

### If a color mask is empty

1. Confirm the peg appears in the saved RGB image. Resolve occlusion or camera framing first.
2. Check [colors.yaml](../config/colors.yaml). `hsv_ranges` controls detection; `rgba` controls the simulated peg color. Changing one may require checking the other.
3. Remember that OpenCV hue is 0–179; saturation and value are 0–255. Red uses two hue intervals around the hue wrap. Keep both intervals when tuning red.
4. Inspect whether the mask is too narrow, too broad, or washed out by lighting. Change only the relevant threshold range and recapture the same seed.
5. Run the color tests, then confirm all four rendered pegs still pass.

```bash
.venv/bin/pytest -q tests/test_color_detection.py
.venv/bin/pytest -q tests/test_pose_estimation.py::test_rendered_peg_position_and_yaw
```

Do not broaden every range until the whole scene becomes foreground. The opening/closing kernel is 3×3 pixels in `ColorSegmenter.masks`; minimum contour area for its general color detector is 90 pixels. The actual peg-pose pipeline separately requires at least 80 depth points, at least 80 pixels of top-contour area, and confidence ≥0.4. Those latter gates are source constants in `pose_estimation.py`, not YAML options.

### If color is correct but shape is missing or wrong

1. Check depth before changing polygon thresholds. The pose estimator selects the top surface within 1.5 mm of its estimated top height. A tilted, occluded, or badly projected peg may leave a broken top contour.
2. Inspect the final contour in `detections.png`, rather than classifying the complete colored silhouette including side walls.
3. The classifier uses polygon approximation at 0.022 times contour perimeter, vertex count, square aspect ratio, and circularity. Triangle, square, and hexagon orientation fitting uses the full uncompressed boundary.
4. Preserve `CHAIN_APPROX_NONE` for the boundary used by pose fitting. Compressing a long straight edge into two endpoints can underweight that edge and bias a triangle center.
5. If image resolution or lighting is deliberately changed, reevaluate contour area and morphology thresholds on several orientations. A lower resolution can make a valid small contour fail a pixel-area gate.

```bash
.venv/bin/pytest -q tests/test_shape_detection.py tests/test_pose_estimation.py
```

The verified scene has one peg of each color and shape. `detect_pegs` chooses the largest eligible top contour per color and stores observations by shape. Tracking multiple pegs with the same color/shape requires an additional multi-object representation and association step; a threshold change alone does not implement that feature.

## 4. Recover incorrect 3D positions

`ObservedPose.position` is the **peg top-face center**, not its rigid-body center or bottom. Grasp planning moves down from that top by `grasp_below_top` (currently 0.018 m). Confusing these points can produce a miss or table contact even when the printed XY position looks reasonable.

The current camera contract is:

| Quantity | Convention |
| --- | --- |
| RGB array | `uint8`, RGB channel order, top-left image origin |
| Depth array | Meters, optical camera Z; not Euclidean distance along a ray |
| Pixel camera axes | OpenCV: X right, Y down, Z forward |
| MuJoCo camera axes | X right, Y up, Z backward |
| Pixel principal point | `((width-1)/2, (height-1)/2)` |
| World units | Meters |

The pinned MuJoCo 3.3.5 high-level renderer already converts depth to meters and flips both arrays to top-left origin. Do not apply `raw_depth_to_metric` to `CameraInterface.capture().depth`, and do not flip the image a second time. That helper is only for low-level normalized OpenGL buffers.

For the default downward overhead camera at Z=1.25 m, the tabletop is approximately 1.25 m optical depth and a 0.10 m-high peg top approximately 1.15 m. The board rim is approximately 1.22 m. These are diagnostic expectations for the default camera, not replacement navigation targets.

Check the actual observations without commanding robot transport:

```bash
.venv/bin/python - <<'PY'
import numpy as np
from peg_sorting.simulation.mujoco_env import MujocoEnv
from peg_sorting.simulation.camera_interface import CameraInterface
from peg_sorting.vision.pose_estimation import detect_pegs
from peg_sorting.vision.socket_detection import detect_sockets

env = MujocoEnv(seed=0)
env.step(seconds=.6)
cameras = CameraInterface(env)
try:
    frame = cameras.capture("overhead")
    print("K:\n", frame.K)
    print("camera_to_world:\n", frame.camera_to_world)
    for shape, pose in detect_pegs(frame).items():
        print("peg", shape, pose.color, "top_m=", np.round(pose.position, 5),
              "yaw_rad=", round(pose.yaw, 5), "confidence=", round(pose.confidence, 3))
    for shape, pose in detect_sockets(frame).items():
        print("socket", shape, "rim_center_m=", np.round(pose.position, 5),
              "yaw_rad=", round(pose.yaw, 5))
finally:
    cameras.close()
PY
```

Camera positions, targets, FOV, and image dimensions are in [cameras.yaml](../config/cameras.yaml). Intrinsics and extrinsics come from the compiled model at runtime. [camera_calib.yaml](../config/camera_calib.yaml) documents conventions; editing that descriptive file does not recalibrate the camera interface.

Run `tests/test_pose_estimation.py` after a camera change. Its projection test reconstructs known visible tabletop points using actual depth from **both** cameras. Correct RGB overlays alone are insufficient evidence that world coordinates are correct.

## 5. Recover missing sockets or biased socket centers

Socket detection has three prerequisites: visible board perimeter, enough neutral rim surface to estimate the rim height, and aperture contours with recognizable shapes.

1. Inspect the magenta board border in the overhead RGB frame. The board detector builds a mask from it rather than using a fixed pixel rectangle.
2. If the parked gripper covers an aperture, restore the default parking pose before changing segmentation. The angled view helps identify the obstruction, but automatic multi-view occlusion recovery is not implemented.
3. Check errors such as `Board perimeter not visible` or `Insufficient board surface for plane estimate`. The magenta HSV range and neutral-surface test are in `socket_detection.py`; they are separate from peg thresholds in `colors.yaml`.
4. The aperture mask uses points below the observed rim plane. Its contours use `RETR_LIST`, because a dark outer board margin can enclose the holes and hide them from `RETR_EXTERNAL`.
5. Aperture boundaries are projected to the **rim plane**, not the recessed floor. Using floor depth for a visible rim boundary can bias XY alignment through perspective.
6. Check detected aperture shape, center, and yaw before allowing robot motion. The current socket contour-area window is 150–6000 pixels; deliberate camera-resolution changes may affect it.

```bash
.venv/bin/pytest -q tests/test_socket_matching.py
```

The tests include a translated board, so fixed pixel coordinates cannot satisfy them. Arbitrary board tilt/rotation is outside the verified fixture setup. Physical placement evaluation currently assumes the nominal upright socket orientations.

If moving the board in the simulated scene, update its layout coherently. `board_center` and `board_half_size` locate the floor and magenta perimeter, while `socket_centers` independently place the four rim tiles and sites. Changing only `board_center` does not move all four apertures. Each rim tile's outer half-size is defined in `scene_manager.py`; keep all tiles supported by the floor and inside the perimeter.

## 6. Separate IK reachability from collision freedom

IK answers whether the joint chain can achieve a tool **position and orientation** within limits. Collision checking answers whether a configuration or motion intersects the scene. Passing one does not pass the other.

The manufacturer reach number does not guarantee that a high, downward-oriented grasp is reachable. The grasp site's tool offset, shoulder/elbow/wrist limits, and required orientation all matter. An earlier 0.27 m lift height was unreachable in the original wider layout. The validated layout uses `clearance_z: 0.22` and narrower source/socket cells.

This diagnostic checks the observed square peg's pregrasp without executing a robot move:

```bash
.venv/bin/python - <<'PY'
from peg_sorting.simulation.mujoco_env import MujocoEnv
from peg_sorting.simulation.camera_interface import CameraInterface
from peg_sorting.vision.pose_estimation import detect_pegs
from peg_sorting.planning.grasp_planner import plan_grasp
from peg_sorting.control.robot_controller import RobotController

env = MujocoEnv(seed=0)
env.step(seconds=.6)
cameras = CameraInterface(env)
try:
    peg = detect_pegs(cameras.capture("overhead"))["square"]
    grasp = plan_grasp(peg)
    robot = RobotController(env)
    result = robot.ik.solve(grasp.pregrasp, grasp.rotation)
    print("reachable:", result.success, "position_error_m:", result.position_error,
          "orientation_error_rad:", result.orientation_error)
    if result.success:
        print("endpoint_collision:", robot.planner.collision(result.q, allowed_peg=peg.shape))
        path = robot.planner.plan(env.data.qpos[env.qids], result.q, allowed_peg=peg.shape)
        print("pregrasp trajectory accepted; duration_s:", path.duration)
finally:
    cameras.close()
PY
```

This is a preliminary pregrasp check, not proof that the entire pick/place route or a carried-object path will pass. Full execution also checks the actual interpolation and, after attachment, the held peg's geometry. The planner uses scratch `MjData`; its virtual held-object pose assignment does not move live simulation objects.

When IK fails, inspect both reported errors. Current tolerances are 0.0004 m position and 0.008 rad orientation in [planning.yaml](../config/planning.yaml). Resolve pose, layout, height, or branch/seed issues instead of accepting position-only convergence, changing robot link dimensions, or expanding manufacturer joint limits.

## 7. Read collision messages and choose the appropriate correction

An observed development error was:

```text
Trajectory collision rejected: peg_square_geom / rim_square_4: -0.0015 m
```

It means a checked configuration put the square peg into a square socket-wall piece with approximately 1.5 mm penetration. It is not evidence that color detection failed. In that development case, the gripper's occlusion had biased the detected socket pose; correcting perception was necessary before changing motion.

| Reported pair | Likely investigation | Correction to try after confirming the cause |
| --- | --- | --- |
| `peg_<shape>_geom / rim_<shape>_<index>` | Socket XY/yaw bias, grasp-induced offset, peg tilt, aperture geometry | Correct camera/contour/pose errors; check the grasp relation and intended orientation; inspect the real opening before considering a justified fixture-clearance redesign |
| Robot link or finger / `table` | Wrong Z reference, tool offset, approach height, or route | Recheck top-versus-center coordinates and full tool pose; adjust reachable waypoints or source cells |
| Robot link / board rim | Arm elbow/wrist sweep through the board | Try a reachable transfer height or an explicit lateral waypoint, then check the entire interpolated path |
| Robot link / another peg | Insufficient source spacing or a path sweeping across an upright peg | Reposition source cells within the validated reach region or add a checked approach route |
| Robot link / robot link | IK branch or interpolation self-intersection | Inspect another valid IK seed/branch and its whole route while retaining the official geometry and limits |
| `joint limit` | Endpoint outside range or cubic interpolation overshoot | Inspect endpoints and extrema; retain the exact spline-extrema checks |

`rim_square_4` is a generated convex prism, not a socket center coordinate. Named `peg_*_geom` objects are movable collision geometries. Unnamed robot geoms may appear using their `wx250s/...` body name.

The planner currently permits a 0.7 mm numerical penetration tolerance. It deliberately exempts the fixed base/table mounting pair and intended gripper/selected-target or gripper/held-object contacts. It still checks other relevant arm, table, board, self, and carried-object contacts. A free peg resting on the table is an expected support contact, not by itself an arm-trajectory failure.

Do not solve a rejection by disabling collision physics, exempting every robot/environment pair, or enlarging the penetration tolerance. The planner is conservative: rejection can mean that this selected route is unsuitable even if another route exists. Automatic global detour search is not implemented.

### A controlled way to change the route

1. Save the failing seed, phase, collision pair, and current camera evidence.
2. Verify the peg/socket poses before touching trajectory settings.
3. Identify whether the arm, held peg, table, board, or another peg is the obstruction.
4. Change one relevant source cell, socket layout, or waypoint. Raising `clearance_z` can clear an obstacle but can also make the downward pose unreachable; check both conditions.
5. Validate the new pregrasp, grasp, lift, transfer, and insertion poses using full-pose IK.
6. Validate entire motions. Endpoints alone cannot reveal an intermediate elbow sweep or cubic bulge.
7. If finer checks are needed, decrease `sample_joint_step` from its current 0.025 rad or decrease `approach_step` from 0.008 m, then measure the additional computation. These improve sampling resolution; they do not create a continuous collision-proof or globally complete planner.
8. Run the collision and physical-insertion tests, then repeat the whole seeded task and inspect its saved result.

Checked waypoints and insertion phases are defined in [pick_place_planner.py](../src/peg_sorting/planning/pick_place_planner.py); curve generation and collision sampling are in [trajectory_planner.py](../src/peg_sorting/planning/trajectory_planner.py). The live controller writes actuators rather than teleporting pegs or the arm.

## 8. Recover failed grasps and insertions

The gripper is an actuated finger model assisted by a disclosed local weld. Its capture conditions are explicit:

| Check | Current condition |
| --- | --- |
| Tool distance from grasp region | At most 9 mm |
| Tool downward / peg upright | Dot-product/uprightness checks ≥0.995 |
| Peg width along jaw travel | At most 0.058 m in the capture check |
| Arm joint-velocity norm before closing | At most 0.07 rad/s |
| Hold check | Relative peg/gripper translation stays within 6 mm of the stored attachment |

For a miss, inspect the measured top pose, tool offset, approach orientation, and settling before closing. For a moving arm, inspect `settle_seconds` (0.3 s before closure) and endpoint settling in `planning.yaml`. The current endpoint budget is 0.04–0.25 s and requires low arm velocity before continuing. Reducing these waits indiscriminately can make capture fail.

`grasp_below_top` is 0.018 m. The gripper's capture region is currently coded as 0.032 m above the peg body center, consistent with the fixed 0.05 m half-height and 0.018 m below-top grasp. Changing `grasp_below_top` alone does not update that gate; a different grasp geometry needs coordinated changes and validation. Similarly, the declared `peg_height` in `simulation.yaml` does not currently replace all 0.05/0.10 m geometry/planning/evaluation constants. Treat height changes as a model-and-control update rather than a single YAML tweak.

The socket must be a real aperture. In [scene_manager.py](../src/peg_sorting/simulation/scene_manager.py), convex prisms surround its opening and a floor supports the inserted peg. `socket_clearance` is 0.003 m normal to polygon edges. The nominal rim is at Z=0.03 m and floor at Z=0.002 m, giving 0.028 m insertion depth. Increasing clearance is a fixture-design change, not a substitute for accurate localization. Keep board height, floor, depth, marker height, and placement expectations consistent if redesigning the fixture.

Insertion is slow only near the opening: the final `socket_depth + insertion_entry_clearance` = 0.028 + 0.015 = 0.043 m uses `placement_speed: 0.035` m/s. General approaches use 0.09 m/s. Adjust approach/settling settings only after determining whether a failure is dynamic tracking, wrong alignment, or wrong geometry.

If the peg rocks, inspect collision geometry and contact parameters rather than labeling a rim contact as success. The implemented correction uses primitive box/cylinder geometry where suitable, multi-contact collision mode for polygon meshes, and softer peg contact parameters. Contact friction, `solref`, and `solimp` are currently in `scene_manager.py`, not YAML. Preserve a physically supported release and rerun the physical fixture tests after changes.

## 9. Distinguish a successful insertion from a nearby peg

Read `placements[].verification` in the episode JSON. `physics` and `camera` must both pass.

| Physical requirement | Current acceptance |
| --- | --- |
| Release | Grasp equality is inactive |
| XY position | Error <0.003 m from matching socket |
| Peg bottom height | Error <0.0025 m from socket floor |
| Uprightness | Rotation Z component >0.995 |
| Shape-symmetry yaw | Error <0.045 rad; circular yaw is irrelevant |
| Linear speed | <0.01 m/s |
| Angular speed | <0.10 rad/s |
| Position stability | <0.002 m drift across the sampled window |

The overhead camera must find the peg within 4 mm in XY and 3 mm of the expected seated top height. The angled camera's colored top-height estimate must agree within 5 mm. The nominal seated top is about 0.102 m. A peg standing on the rim can have correct XY but wrong bottom/top heights and must fail.

Verification observes six position samples, 0.15 s apart. A released, position-stable peg with camera agreement, acceptable XY/bottom-height/yaw, and a failed velocity limit can receive two further 0.9 s observation windows. Every window repeats the full checks, including uprightness. This can avoid an unnecessary re-grasp while contact motion settles; success still requires every original threshold.

`max_retries: 1` allows at most two grasp/place attempts. A failure while holding an object stops further autonomous actions instead of dropping it blindly. In teleoperation, Space holds commanded motion; it is a simulation stop. For an autonomous run, closing the viewer or interrupting the terminal stops the program. Restart a new seeded simulation for a clean recovery. Do not alter live peg `qpos` to manufacture an insertion result.

An error before the controller constructs its result, such as an initial camera/board failure, may leave no `episode.json`. Preserve the terminal traceback and use the separate vision capture for evidence. Later episode results store the outcome/error, attempt count, final verification, and timing; they are not a complete log of every intermediate trajectory or superseded retry verification.

## 10. Know which settings actually control each issue

| File | Effective settings or code | Use |
| --- | --- | --- |
| [colors.yaml](../config/colors.yaml) | `hsv_ranges`, `rgba` | Peg appearance and segmentation |
| [cameras.yaml](../config/cameras.yaml) | `width`, `height`, camera `position`, `target`, `fovy` | Framing/resolution; recheck projection and area gates |
| [simulation.yaml](../config/simulation.yaml) | `peg_centers`, `socket_centers`, `board_center`, `board_half_size`, `socket_clearance`, `board_height`, `peg_mass`, `timestep` | Scene layout, fixture geometry, dynamics; coupled changes need review |
| [planning.yaml](../config/planning.yaml) | `clearance_z`, `pregrasp_offset`, `approach_step`, `sample_joint_step` | Reachable clearances and sampling |
| [planning.yaml](../config/planning.yaml) | `max_joint_speed`, `max_joint_acceleration`, `approach_speed`, `placement_speed` | Motion command rates |
| [planning.yaml](../config/planning.yaml) | `tracking_settle_min_seconds`, `tracking_settle_max_seconds`, `settle_seconds`, `verification_settle_rechecks`, `max_retries` | Settling and bounded recovery |
| [pose_estimation.py](../src/peg_sorting/vision/pose_estimation.py) | Top-height band, area/point/confidence gates | Object localization filtering |
| [socket_detection.py](../src/peg_sorting/vision/socket_detection.py) | Magenta threshold, rim estimation, hole contour area | Board/aperture extraction |
| [scene_manager.py](../src/peg_sorting/simulation/scene_manager.py) | Peg dimensions, rim-tile size, floor/marker heights, contact properties | Coordinated fixture/model changes |

Use `--seed` for runtime randomization; the declared YAML seed is not the CLI's seed source. Nominal XML is not rewritten by a normal run: `MujocoEnv` builds a fresh scene in memory. If deliberately publishing updated standalone/nominal assets, run `scripts/generate_demo_scene.py --seed 0` after validation and review the generated file differences. Do not casually edit vendored robot meshes, joint ranges, or link dimensions to make a route pass.

## 11. Prove the correction at increasing scope

Run the narrow tests for the layer you changed:

```bash
.venv/bin/pytest -q tests/test_color_detection.py tests/test_shape_detection.py
.venv/bin/pytest -q tests/test_pose_estimation.py tests/test_socket_matching.py
.venv/bin/pytest -q tests/test_inverse_kinematics.py tests/test_planning.py
.venv/bin/pytest -q tests/test_placement.py
```

Then retain one run's console and artifacts together. Choose a new output directory for a new investigation:

```bash
mkdir -p results/runs/recovery_seed0
set -o pipefail
bash launch/run_autonomy.sh --headless --seed 0 \
  --output results/runs/recovery_seed0 2>&1 | tee results/runs/recovery_seed0/console.log
```

`pipefail` preserves an autonomous failure exit status instead of reporting only `tee`'s success. A complete successful run exits with 0; a failed sorting outcome exits nonzero. Inspect the saved initial/final images and JSON even when the console says success.

```bash
.venv/bin/python -m json.tool results/runs/recovery_seed0/episode.json
.venv/bin/pytest -q
.venv/bin/python scripts/evaluate_success.py --trials 5 --seed 0 \
  --output results/runs/recovery_trials
.venv/bin/python scripts/evaluate_timing.py results/runs/recovery_trials/trials.csv
```

The CSV separates detected pegs, correct colors/classifications, mean initial pose error, successful grasps, successful/failed/unattempted placements, complete-trial status, simulated time, wall time, and average simulated seconds per attempted peg. The summary reports complete-trial and placement rates. Do not count unattempted pegs as successful, or compare wall-time performance without accounting for GUI/recording load.

The saved current reference is **20/20 first-attempt placements in five trials**, mean **90.206 simulated seconds**, with **61 passing tests** in the latest recorded full suite. These are the results of the tested upright, narrow-layout simulation, not proof of arbitrary scene robustness. Consult the [current CSV](../results/performance/trials.csv) and [comparison record](../results/performance/comparison.json) rather than replacing a measured failure with an example target result.

## 12. Current recovery boundaries

The current implementation can reject low-confidence/missing perception, reject unreachable poses, reject sampled collisions, wait for a seated peg to settle, and attempt a bounded recovery when it has safely released the object. It does not implement a global obstacle-detour planner, active search for hidden objects, automatic scene rearrangement, learned object detection, multi-object tracking for repeated colors/shapes, or multi-view pose fusion that resolves arbitrary occlusion. The angled camera supplies observation and independent placement depth verification. Adding those broader behaviors is future implementation work, not an existing setting to enable.

The most reliable recovery order is therefore: **observe → diagnose the first incorrect stage → correct one cause → test that layer → validate the whole path → verify physical insertion → repeat seeded trials**.
