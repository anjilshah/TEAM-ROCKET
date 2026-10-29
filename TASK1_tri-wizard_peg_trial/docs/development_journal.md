# Development obstacles, fixes, and lessons learned

This is the development record for **Tri-Wizard Peg Trial**, not a list of hypothetical errors. It explains where implementation was blocked, how the cause was identified, what changed, and how to check the result. Use the companion [collision and object-finding recovery guide](collision_and_perception_guide.md) when diagnosing a new problem.

Three kinds of entries are distinguished:

- **Observed failure:** a simulator run, test, or diagnostic actually failed during development.
- **Integration decision or prevented risk:** a compatibility issue was investigated or handled before it caused a failed run. These are not presented as errors that happened.
- **Remaining limitation:** the implementation still has this boundary; suggestions for extending it are future work.

The code and measured results are local. Public GitHub/video publication has not occurred. No physical robot, physical RGB-D camera, ROS installation, or hardware experiment was involved.

## Quick issue index

| Obstacle | Evidence/status | Resolution and main location |
| --- | --- | --- |
| System Python lacked the scientific stack and a global `pip` command | Observed environment inspection | Project virtual environment and pinned dependencies: [setup](setup.md), [lockfile](../requirements-lock.txt) |
| Dependency/mesh downloads failed DNS resolution inside the restricted environment | Observed setup failure | Retry approved network access; keep installs local and mesh revision pinned: [fetcher](../scripts/fetch_robot.py) |
| Required WidowX model needed a verified source | Integration decision | Use the licensed Menagerie `wx250s` model derived from Interbotix; retain provenance: [robot notes](../simulation/robots/README.md) |
| Model keyframe dimensions would not match four added free bodies | Prevented integration risk | Remove upstream home keyframe when generating the expanded scene: [scene generator](../src/peg_sorting/simulation/scene_manager.py) |
| Official TCP validation was 0.25 mm short | Observed test failure | Correct the added TCP site's offset to 0.093575 m; keep the original robot links unchanged |
| A placed peg rocked and failed stability verification | Observed `debug0` failure | Primitive box/cylinder contacts, multi-contact mode for polygon meshes, softer contacts, and separate linear/angular velocity checks |
| Downward-oriented lift at Z=0.27 m was unreachable | Observed `debug1` failure | Narrow the workspace, use Z=0.22 m transfer height, validate lift IK before grasping |
| Square peg was predicted to collide with a socket rim | Observed `debug2` failure | Fix perception/occlusion and alignment; retain the collision rejection |
| Board visible but zero sockets detected | Observed `debug3` failure | Retrieve internal contours with `RETR_LIST`; filter actual aperture candidates |
| Triangle boundary fitting biased its center | Observed diagnostic/test failure | Use every boundary pixel with `CHAIN_APPROX_NONE` instead of unevenly weighted compressed contours |
| Camera depth/frame conventions could invert or distort poses | Prevented risk, then tested | Use the pinned renderer's metric, top-left output once; apply the OpenCV-to-MuJoCo axis conversion once |
| Autonomous sorting took about 229 simulated seconds | Observed timing problem | Continuous motion through IK nodes, adaptive endpoint settling, faster approaches, slower final insertion |
| Camera panels competed with physics/UI work | Source/performance diagnosis | Independent wall-clock rates: viewer up to 30 Hz, panels up to 4 Hz, keyboard polling up to 50 Hz |
| Cubic path could exceed a joint limit between valid nodes | Reproduced review finding | Check exact spline position extrema as well as collision samples |
| Settling timeout could allow residual arm velocity | Review finding | Reject motion if arm velocity is still too high at the end of the settling budget |
| A seated hexagon was re-grasped for residual contact velocity | Observed intermediate tuning failure | Re-observe through bounded settling windows using the same strict success criteria before re-grasping |
| Remote publication was unavailable/not authorized | Remaining external step | Local commits and upload-ready artifacts; request owner approval before publication |

## 1. Environment and dependencies

**What blocked progress:** Ubuntu's system Python did not have MuJoCo, OpenCV, NumPy, SciPy, or pytest, and a bare `pip` command was unavailable. The project directory initially contained a virtual environment but no source implementation.

**Steps taken:**

1. Inspected Ubuntu release, architecture, Python, pip, Git, display availability, and existing project files.
2. Used the project's `.venv` and its interpreter explicitly.
3. Installed the pinned scientific/simulation stack in that environment.
4. Added editable packaging so imports work from launchers and scripts.
5. Saved direct and transitive/build-tool versions in `requirements.txt` and `requirements-lock.txt`.
6. Ran the dependency consistency check.

The first network attempts reported name-resolution failures inside the restricted execution environment. Approved network access was used for package and licensed mesh downloads. This was an environment access problem, not a robot-model or Python-algorithm failure. No privileged system package installation was required.

**If it happens again:** follow [setup](setup.md) and run `.venv/bin/python -m pip check`. A missing import should first trigger an interpreter/environment check. Reinstalling the entire simulator stack is not the first diagnostic step.

## 2. Obtaining and integrating the required robot

**Question investigated:** could the specified Interbotix WidowX-250S be used reliably in MuJoCo without replacing it or installing ROS?

**Steps taken:** located Menagerie's licensed MJCF, checked its derivation from the Interbotix URDF, inspected the six-joint chain and finger coupling, downloaded its required meshes/texture at an immutable revision, preserved its license, and recorded SHA-256 hashes. The project reuses the upstream conversion; it did not perform a new local URDF conversion.

The scene generator retains the upstream link transforms, joint ranges, masses, inertias, and mesh geometry. It adds the task objects, cameras, tool sites, actuator tuning, and grasp equalities. The upstream home keyframe contains positions for the robot alone; adding four free bodies changes `nq`. Removing that keyframe was a deliberate compatibility step, not a claimed XML error from a failed episode.

**Verification:** all six manufacturer joint ranges and the published zero-pose TCP are checked in [the IK tests](../tests/test_inverse_kinematics.py). The nominal scene and all standalone object XMLs were also compiled by MuJoCo. See [robot integration notes](../simulation/robots/README.md) for the complete provenance.

## 3. Correcting the official TCP reference

**Observed symptom:** the added official TCP site reached X=0.458075 m at zero joint angles, while the official reference is X=0.458325 m. The 0.25 mm difference failed the FK test.

**Cause:** the added tool-site offset was entered as 0.093325 m. Inspecting the URDF fixed joints gave the correct sum:

```text
ee_arm + ee_bar + ee_gripper
0.043 + 0.023 + 0.027575 = 0.093575 m
```

**Change:** corrected the added site in [scene generation](../src/peg_sorting/simulation/scene_manager.py) and the reference in [robot configuration](../config/robot.yaml). The original link dimensions were not altered to make the test pass.

The **grasp site** remains a different point, X=0.108 m from `gripper_link`, derived from the upstream finger contact locations. Confusing these two sites would change the grasp height/position.

## 4. Contact rocking and misleading stability measurements

**Observed symptom:** the square was released inside its socket and both cameras confirmed its top height, but physical verification rejected it. The preserved `debug0` record reports XY error about 0.664 mm, bottom error about 0.469 mm, and a stable position window, alongside an old aggregate `speed` value of 0.1350.

That old aggregate mixed angular and linear velocity components, so it cannot be read as a speed in meters per second. Separate diagnostics also showed small alternating contact motion. Both the contact behavior and the metric needed attention.

**Steps taken:**

1. Inspected actual body positions, free-joint velocities, contact behavior, and camera verification.
2. Replaced square/circle mesh contacts with suitable box/cylinder primitives.
3. Enabled multiple-contact collision handling for polygon meshes.
4. Softened peg contact settings to reduce chatter at the 2 ms timestep.
5. Replaced the mixed-unit norm with separate linear and angular velocity checks.
6. Retained the insertion-depth, uprightness, yaw-alignment, release, and position-stability requirements in addition to camera agreement.

**Verification:** [physical fixture tests](../tests/test_placement.py) drop each matching peg into its opening, verify a peg resting above a socket fails, and show that a square is blocked by the triangular aperture. Successful autonomous episodes now contain the separate velocity fields in their verification results.

**What to change for a recurrence:** inspect contact geometry and timestep/contact parameters together. Check units before interpreting velocity. Keep a new contact configuration only after physical drop tests and released-object stability checks pass. The current values are recorded in the recovery guide.

## 5. Unreachable lift after a valid grasp

**Observed symptom:** `debug1` completed the square, then rejected a triangle lift target near `(0.3542, -0.1261, 0.27)` m. The best IK solution still had about 1.8 mm position error and 0.0149 rad orientation error, outside the required tolerances.

**Cause:** radial reach alone does not prove a pose is reachable. A downward tool, tool length, height, and all joint limits must be satisfied together. The original source/board layout and 0.27 m lift height pushed this constrained pose too far.

**Steps taken:** moved source cells and sockets into a conservative region, reduced `clearance_z` to 0.22 m, and added pre-grasp checks for the pregrasp, grasp, and lift targets. This avoids capturing an object before discovering that its lift is impossible.

**What to change:** coordinate `peg_centers`/`socket_centers` in [simulation configuration](../config/simulation.yaml) with the motion heights in [planning configuration](../config/planning.yaml). Check the full downward orientation with IK, then check collisions. Raising clearance indiscriminately can recreate this failure. Manufacturer joint limits and link dimensions should remain verified reference values.

## 6. A collision rejection exposed an upstream perception error

**Observed symptom:** `debug2` stopped with:

```text
Trajectory collision rejected: peg_square_geom / rim_square_4: -0.0015 m
```

The negative distance indicates a predicted penetration of about 1.5 mm at a checked configuration. It does not by itself prove that the board mesh is wrong.

**Diagnosis:** after the layout change, the parked gripper partially covered the lower board/socket region in the overhead image. The estimated square socket center and yaw were biased. A plausible but incorrect perception target therefore produced a real predicted rim collision.

**Steps taken:** inspected the rendered view and estimated socket pose against evaluation data, changed the parking pose to point toward −Y away from the board, and retained collision rejection. Further contour corrections described below were needed to make the updated view reliable.

**Lesson:** investigate target perception, coordinate conventions, and object orientation before changing clearance or collision exclusions. The planner correctly prevented a motion that should not have been executed.

## 7. Four pegs detected, zero sockets detected

**Observed symptom:** `debug3` detected all four pegs but no sockets and aborted before moving.

**Cause:** the recessed dark margin could surround the board interior in the depth mask. OpenCV's external-contour retrieval found the enclosing margin and hid nested aperture contours. Filtering out the large enclosing contour then left no sockets.

**Change:** socket detection now uses `RETR_LIST`, filters candidates by area/shape, and estimates the rim plane from visible neutral board pixels. The board is located through its visible magenta perimeter rather than a fixed image rectangle.

**Verification:** [socket tests](../tests/test_socket_matching.py) check all four observed centers and orientations and translate the board to check that detection follows the rendered board. Translation is tested; this is not a claim of arbitrary tilted-board reconstruction.

**For recurrence:** inspect the detected board mask and depth holes before changing classifier thresholds. A visible board does not guarantee visible or correctly retrieved apertures.

## 8. Triangle shape fitting gave a biased pose

**Observed symptom:** an apparently valid triangular contour produced a center far enough from the true socket to fail the socket-pose test and predict a rim collision.

**Cause:** `CHAIN_APPROX_SIMPLE` compresses straight contour edges. A long vertical edge may be represented by just its endpoints, while a rasterized sloping edge retains many points. Giving every retained point equal weight therefore gives different physical edges unequal weight during polygon fitting.

**Change:** peg-top and socket-boundary extraction use `CHAIN_APPROX_NONE` before regular-polygon fitting. Polygon approximation is still used for classification and an initial yaw estimate; the fitting stage receives the full boundary.

**Verification:** rendered peg position/yaw tests, socket pose tests, rotated synthetic shape tests, and the full sorting episode pass. If a triangle is classified correctly but its position is wrong, inspect the fitting input and weighting as well as the class label.

## 9. Depth conventions were checked before using them for motion

**Prevented risk:** interpreting the high-level renderer's output as an unconverted OpenGL depth buffer, flipping image rows twice, or mixing camera-axis conventions would create wrong world targets.

**Steps taken:** inspected the pinned MuJoCo 3.3.5 renderer implementation, confirmed that `Renderer.render()` already returns metric optical Z and top-left image orientation, derived intrinsics from the compiled camera FOV, and applied the OpenCV-to-MuJoCo axis conversion once. The separate low-level depth-buffer helper is not applied to these metric images.

**Verification:** [camera projection tests](../tests/test_pose_estimation.py) reconstruct known visible tabletop points from both actual RGB-D views. These are calibration/convention checks against real rendered depth, not just inverse algebra applied to synthetic numbers.

## 10. Slow movement and excessive intermediate stopping

**Observed symptom:** sorting took approximately 229 simulated seconds. A seed-0 profile counted **268 stopped joint segments** and **67 simulated seconds** of fixed 0.25 s waits alone.

**Steps taken:**

1. Measured the baseline instead of increasing all speeds immediately.
2. Kept 8 mm Cartesian IK nodes, but interpolated a continuous cubic joint curve through them.
3. Applied a single quintic progress function so each complete approach/lift/retreat starts and stops once.
4. Computed derivative bounds to respect 1.2 rad/s and 2.8 rad/s² joint command limits.
5. Replaced per-node waits with adaptive 0.04–0.25 s endpoint settling and a residual-velocity guard.
6. Used 0.09 m/s general approaches while retaining 0.035 m/s for the final socket depth plus 15 mm of lowering.
7. Separated viewer, camera-panel, and keyboard event scheduling. Camera panels default to at most 4 Hz; the viewer updates independently at up to 30 Hz.
8. Re-ran five seeded physical sorting trials, the full automated suite, a complete GUI run, and a new recorded run.

**Result:** the five-seed mean fell from **228.862 to 90.206 simulated seconds**, a **2.54× speedup / 60.6% shorter duration**. All **20 placements passed on their first attempt**. The GUI run took about 90.01 wall seconds. The current video is `results/demo_fast.mp4`; the older `demo_final.mp4` intentionally preserves the slower baseline.

The [comparison JSON](../results/performance/comparison.json) and [current CSV](../results/performance/trials.csv) contain the measurements. Wall time includes machine/rendering load and should be distinguished from simulated execution time.

**If the user still sees the old behavior:** close and restart the existing Python/viewer process. It retains previously imported code. On a slower rendering machine, use `--camera-fps 2` to reduce panel work without changing physics or motion criteria.

## 11. Continuous paths introduced two correctness checks

**Reproduced review finding:** four legal waist nodes at `upper_limit − [0.003, 0.0001, 0.0001, 0.003]` can produce a cubic curve that overshoots the upper limit between nodes. Checking only endpoints or sparsely sampled nodes is insufficient.

**Change:** the planner now includes all real position extrema from each cubic interval in its joint-limit checks, as well as sampling the actual curve for collisions. Exact derivative bounds determine the trajectory duration. The [overshoot regression test](../tests/test_planning.py) preserves this case.

**Second review finding:** an endpoint settling loop could exhaust its time budget and return successfully if position error was small, even while velocity was too high. The controller now explicitly rejects residual arm velocity after the settling budget.

These findings were caught while reviewing the speed improvement. They are not claimed as crashes in a published deployment. Stop responsiveness was also rechecked: the simulation stop flag is evaluated every 2 ms physics step, including settling.

## 12. Waiting for a seated peg before re-grasping it

**Observed intermediate tuning symptom:** the hexagon was seated and camera-confirmed, with XY error about 1.131 mm, bottom error about 0.193 mm, and a stable position window, but angular speed was **0.10248 rad/s**, just above the existing **0.10 rad/s** limit. Immediate re-grasping added unnecessary time.

**Change:** when a peg is released, position-stable, camera-confirmed, within the XY/bottom-height/yaw limits, and failing a residual-velocity check, verification can wait through up to two additional 0.9 s observation windows. Every window repeats the complete camera and physical checks, including uprightness. A new observation must satisfy all requirements before success is reported; the waiting rule does not relax them.

This is separate from the one bounded grasp retry. A held-object motion failure ends the episode for recovery; the controller does not blindly open the gripper while carrying it. The final optimized five-seed batch needed four grasps per complete trial.

## 13. What is implemented, and what remains limited

| Area | Current behavior | How to extend it, if needed |
| --- | --- | --- |
| Object finding | HSV masks, depth-isolated top faces, contours, regular-polygon pose fitting | Add calibrated noise tests, varied lighting, occlusion handling, and explicit multiple-instance association before expanding the operating domain |
| Camera use | Overhead perception drives targets; angled RGB-D supplies visibility and independent placement-height evidence | Add tested cross-view pose fusion if overhead targets must be recovered under occlusion; it is not currently an automatic second-view search system |
| Collision planning | Conservative waypoints, full-pose IK, actual-curve samples, carried-object collision prediction | Add a genuine obstacle-avoiding planner if automatic detours are required; current rejection does not automatically create a new route |
| Grasping | Actuated fingers plus proximity/alignment-gated weld assistance | Validate friction-only grasping separately before claiming it; do not remove assistance and assume identical performance |
| Scene geometry | Upright 10 cm pegs and upright sockets in a small tested region | Update all related dimensions/offsets and tests together for a new fixture; `peg_height` alone does not propagate everywhere |
| Configuration | Many live settings plus some documented nominal values | Audit where each key is consumed before changing it; the recovery guide maps the coupled settings |
| Evaluation | Five seeds, ±6 mm source-cell jitter, yaw within ±0.35 rad | Collect broader independent trials with unchanged success criteria before claiming general robustness |
| Teleoperation | GUI launch, command handlers, P/L macros, and stop behavior tested | Rehearse actual human keypresses/focus before the public demo |
| Publication | Local commits, documentation, measurements, actual recordings | Owner approval and a verified public upload are still needed |

## 14. How to document the next problem

Keep one record per experiment. Preserve the failing seed and output before editing settings. Use this template:

```text
Issue ID / date:
Exact launch command and seed:
Source commit and modified files:
Visible symptom / full exception:
RGB, depth, detection overlay, episode JSON paths:
Expected behavior:
Measured behavior (with units):
Suspected cause:
One change being tested (file/key/function):
Focused check and result:
Full sorting result and unchanged success criteria:
Remaining limitation / next step:
```

Change one subsystem at a time. For a collision, first establish that the observed target pose is correct; for a detection failure, first inspect images/masks before changing motion. After a focused check passes, run one complete seeded episode, then multiple seeded trials for changes that affect planning, geometry, grasping, or perception. Use fresh output directories so new runs cannot silently replace the evidence being compared.

## Evidence and source map

- [Condensed historical failure evidence](evidence/development_failures.json): extracts from preserved local development episodes and the timing profile. Raw debug directories are intentionally ignored by Git and may not exist in another checkout.
- [Current validation record](validation.md), [JUnit report](../results/validation/pytest.xml), and [machine-readable checks](../results/validation/checks.json).
- [Original measured baseline](../results/evaluation/summary.json) and [optimized measurements](../results/performance/summary.json).
- [Robot provenance and parameter validation](../simulation/robots/README.md).
- [Step-by-step recovery commands](collision_and_perception_guide.md) and [short troubleshooting reference](troubleshooting.md).

The measured results above are retained evidence, not new experiments run merely to write this document. Documentation checks and selected diagnostic examples are validated separately from historical simulation trials.

## Implementation steps across the whole assignment

| Phase | Steps actually taken | Evidence or resulting files |
| --- | --- | --- |
| Environment | Inspected Ubuntu/tooling and existing files; used project-local Python dependencies; checked imports/dependency compatibility | [Setup](setup.md), [dependency lock](../requirements-lock.txt), [validation checks](../results/validation/checks.json) |
| Repository | Created editable packaging, modular source, configuration, executable launchers, tests, and ignore rules | [File inventory](file_inventory.md), [package configuration](../pyproject.toml) |
| Robot integration | Selected the required licensed `wx250s`; preserved original model/meshes; verified FK/limits and added tool sites | [Robot notes](../simulation/robots/README.md), [provenance](../simulation/robots/wx250s/provenance.json) |
| Physical scene | Built the table, four free-body pegs, actual socket apertures and floors, lights, gravity, and contact settings | [Scene generator](../src/peg_sorting/simulation/scene_manager.py), [nominal XML](../simulation/scenes/peg_sorting.xml), [fixture tests](../tests/test_placement.py) |
| Cameras | Added overhead/angled RGB-D views and recording overview; implemented arrays, intrinsics, extrinsics, and depth/frame checks | [Camera interface](../src/peg_sorting/simulation/camera_interface.py), [projection tests](../tests/test_pose_estimation.py) |
| Vision and localization | Implemented HSV cleanup, shape classification, depth-isolated top faces, world positions, yaw fitting, board/socket detection, and matching | [Vision source](../src/peg_sorting/vision/pose_estimation.py), [socket source](../src/peg_sorting/vision/socket_detection.py) |
| Robot control | Implemented model-Jacobian full-pose IK, actuator tracking, finger commands, local assisted grasp capture, and release | [IK](../src/peg_sorting/planning/inverse_kinematics.py), [arm control](../src/peg_sorting/control/robot_controller.py), [gripper](../src/peg_sorting/control/gripper_controller.py) |
| Planning | Added reachable grasp/place targets, conservative transfer/approach phases, joint/curve checks, carried-object collision prediction, and continuous timing | [Pick/place planner](../src/peg_sorting/planning/pick_place_planner.py), [trajectory planner](../src/peg_sorting/planning/trajectory_planner.py) |
| Autonomy and recovery | Connected capture through insertion/release; required two-camera and physical verification; bounded settling/retry behavior | [Autonomous controller](../src/peg_sorting/control/autonomous_controller.py), [current measured episodes](../results/performance/summary.json) |
| Teleoperation | Added movement/gripper/P/L commands and simulation stop; checked command handlers, macro behavior, and GUI launch | [Keyboard control](../src/peg_sorting/control/teleop_keyboard.py), [teleoperation tests](../tests/test_teleop.py) |
| Evaluation and debugging | Ran failing diagnostics, corrected demonstrated causes, ran the recorded 61-test suite and five-seed trials, measured speed before/after, and completed a GUI run | [Failure extracts](evidence/development_failures.json), [comparison](../results/performance/comparison.json), [JUnit report](../results/validation/pytest.xml) |
| Documentation and demonstration | Wrote setup/architecture/run/recovery instructions and retained actual original/faster simulation videos with measurements and hashes | [Demo guide](demo_guide.md), [faster-video manifest](../results/performance/demo/video_manifest.json) |
| GitHub preparation | Created local commits, selected the requested remote, prepared CI and reviewable artifacts; retained the approval gate for public publication | [GitHub workflow](../.github/workflows/test.yml), [publication status](troubleshooting.md) |

Manual keyboard rehearsal, broader robustness trials, friction-only grasp validation, automatic global detours/active object search, and public upload remain separate follow-up work. They are not implied by the completed simulation pipeline.
