# Validation record

Validated on 29 October 2026, Ubuntu 22.04.4 LTS x86-64, Python 3.10.12. Simulator MuJoCo 3.3.5; direct dependencies and full dependency set are pinned. No privileged system package changes were required. The initial directory contained only a virtual environment and no project source to preserve.

## Executed checks

- Environment inspection: Ubuntu release, architecture, Python/pip/Git availability, installed module discovery, workspace files.
- Model: licensed Menagerie `wx250s` integrated; six joint limits checked against official values; official TCP FK checked against the manufacturer's home matrix. Source files are retained with hashes.
- XML: generated nominal scene, all four standalone peg files, and the board file independently compiled by MuJoCo. Scene has 36 generalized coordinates, 111 geoms, and three cameras (two RGB-D sensors plus recording overview).
- Renderer: actual RGB and metric depth captured from both sensors; projection reconstructed visible known tabletop points in both views.
- Perception: synthetic rotated/noisy silhouettes, hue wrapping, noise removal, actual rendered peg position/yaw and socket center/yaw, translated-board detection.
- Dynamics: each matching peg freely falls through its matching opening and seats; a square peg is physically blocked by the triangular opening; a peg merely near/above a socket fails verification.
- Planning: full-pose IK, unreachable/malformed targets, joint limits, velocity/acceleration limits, grasp confidence, and rejection of remote weld attachment.
- Integration: complete four-peg autonomy, keyboard Cartesian commands and simulation stop, keyboard P/L pick/place macros.
- Automated suite: **51 passed**, including explicit table-collision rejection and Space interruption during an active trajectory; no deleted or skipped failing tests. The checked-in JUnit report is `results/validation/pytest.xml`.
- Five randomized episodes, seeds 0–4: **20/20 placements, 5/5 complete trials**, all initial detections/colors/classifications correct. Actual CSV and JSON are in `results/evaluation/`.
- Headless simulation and vision launch scripts ran successfully. GUI simulation, vision (both RGB-D panels), and teleoperation launchers opened and closed successfully with harmless Wayland window-position/Qt warnings. Physical human keypress rehearsal remains separate from tested command handling.
- Recorded demo: `results/demo_final.mp4`, actual seed-0 execution with 4/4 verified placements, 1,375 frames at 1200×860 and 6 fps, 229.167 s playback. Its episode took 228.948 simulated seconds and 276.233 wall seconds including recording. The video manifest records its SHA-256 hash; the large MP4 remains local and ignored by Git.
- A GitHub Actions workflow is prepared but has not run remotely, since no repository has been published.

## Measured evaluation

| Seed | Placements | Simulated seconds | Mean initial pose error, mm |
| --- | --- | --- | --- |
| 0 | 4/4 | 228.948 | 0.273 |
| 1 | 4/4 | 228.050 | 0.305 |
| 2 | 4/4 | 229.380 | 0.297 |
| 3 | 4/4 | 228.988 | 0.290 |
| 4 | 4/4 | 228.942 | 0.243 |

Mean time: 228.862 simulated seconds; 57.215 simulated seconds per peg. Mean wall time in that batch: 10.149 s, including rendering for perception and competing test activity. Mean initial top-face position error: 0.282 mm. Source cells jitter by ±6 mm and yaw by ±0.35 rad; these results do not establish general robustness outside that domain.

## Debugging decisions

Initial flat mesh contacts rocked at the timestep scale. The implementation uses box/cylinder collision primitives where appropriate and multi-contact collision mode for polygon meshes. Peg contact parameters are softened to reduce chatter. Triangle/hexagon contact motion is evaluated with separate linear and angular velocity thresholds, avoiding a norm that mixes meters/second with radians/second. Autonomous verification also observes position drift across six samples, 0.15 s apart.

An early lift target was unreachable with a downward tool despite being inside a radial manufacturer reach number. Workspace cells were narrowed, transfer height reduced to 0.22 m, and lift IK is now validated before attachment. An early board pose was partially occluded by the parked gripper; the home pose now points away from the board. Socket extraction uses all contour levels so a dark board margin cannot hide apertures. Polygon fitting uses uncompressed boundary pixels so vertical triangle edges receive proportional weight.

Failed development runs were not counted in the final five-trial evaluation. Their debug artifacts remain under ignored `results/runs/`. Evaluation files represent successful reruns after these corrections; no example assignment values were substituted.

## Completion boundaries

Implemented and verified: simulator, required robot, physical pegs/sockets, both sensors, perception, localization, full-pose IK, conservative planning, assisted gripper, autonomous transport/insertion/release, physical/camera verification, keyboard command handlers, evaluation, tests, executable launchers, setup/docs, and local repository preparation.

Explicit limitations: assisted weld grasp; perfect simulated calibration/depth; small tested source region; upright fixtures; no global obstacle planner; no hardware validation; no guarantee every wrong shape is mechanically rejected; manual human keypress rehearsal pending. Public GitHub/video publication requires owner approval and has not occurred.
