# WidowX-250S integration

Robot: Interbotix WidowX-250 6DOF (`wx250s`). No alternative robot was substituted.

Source: Google DeepMind MuJoCo Menagerie, revision `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`, directory `trossen_wx250s`. Upstream license: BSD-3-Clause, retained in `wx250s/LICENSE`. The original README explains its URDF-to-MJCF conversion and requires MuJoCo ≥3.1.6. This project pins and tests 3.3.5.

Upstream describes the chain from the official Interbotix X-Series URDF. The vendor specification and source are linked in the top-level README. `scripts/fetch_robot.py` fetches only the pinned MJCF, license, notes, and required meshes/texture, records hashes, and rejects replacement of modified upstream files. No local URDF conversion or ROS installation was necessary.

## Chain and limits

| Joint | Axis in local link frame | Limits, degrees |
| --- | --- | --- |
| waist | Z | −180 to 180 |
| shoulder | Y | −108 to 114 |
| elbow | Y | −123 to 92 |
| forearm_roll | X | −180 to 180 |
| wrist_angle | Y | −100 to 123 |
| wrist_rotate | X | −180 to 180 |
| left_finger | Y slide | 0.015 to 0.037 m |
| right_finger | Y slide | −0.037 to −0.015 m |

The upstream equality makes the right finger mimic the left with a negative multiplier. There are seven position actuators: six arm joints and the gripper. Upstream simplified the gripper drive and added small spherical finger contacts and a base/shoulder exclusion.

The main kinematic translations are shoulder Z 0.072 m; upper-arm origin Z 0.03865 m; upper-forearm origin `(0.04975,0,0.25)` m; lower forearm X 0.175 m; wrist X 0.075 m; gripper X 0.065 m. These values come directly from the source. The official tool offset is `0.043 + 0.023 + 0.027575 = 0.093575 m`, from URDF fixed joints `ee_arm`, `ee_bar`, and `ee_gripper`. Zero-pose FK therefore reaches `(0.458325,0,0.36065)` m. A test verifies this against the manufacturer's published home matrix.

The manipulation grasp site is distinct: X 0.108 m, derived from finger body X 0.066 m plus contact X 0.042 m. Downward grasping constrains local tool X to world −Z and preserves a chosen in-plane yaw.

## Local integration changes

`scene_manager.py` parses the original XML without modifying the vendored files. It resolves mesh paths, removes the upstream home keyframe because four free bodies change `nq`, adds cameras/table/board/pegs, adds the two tool sites and four initially inactive welds, and uses a collision-safe parking configuration. It sets a 2 ms timestep, multi-contact collision mode, and servo proportional gain 450 with damping ratio inherited from upstream. Arm actuator force limits remain the upstream ±35 setting. Bias-force compensation is applied through `qfrc_applied`.

Joint definitions, ranges, link transformations, masses, inertias, visual meshes, and collision meshes are retained. The resulting model is an existing simplified simulation description, not a newly measured digital twin. Servo tuning and assisted weld grasping are not validated hardware behavior.

Validation includes all six manufacturer limits, zero-pose FK, three reachable orientation-constrained IK targets, an unreachable target, collision-aware trajectories, and complete sorting episodes. The committed scene and standalone robot load in MuJoCo. Verify file integrity against `wx250s/provenance.json` before changing vendor files.
