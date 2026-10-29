# TEAM ROCKET

Robotics assignment workspace. Each task lives in its own directory.

| Task | Project | Status |
| --- | --- | --- |
| [Task 1](TASK1_tri-wizard_peg_trial/) | [Tri-Wizard Peg Trial](TASK1_tri-wizard_peg_trial/README.md) | MuJoCo WidowX-250S autonomous sorting; 61 tests passed; 20/20 placements in five seeded trials |

## Task 1: autonomous peg sorting

The project uses two simulated RGB-D cameras and OpenCV to detect four colored peg shapes, estimate their poses, find matching sockets, and guide the required WidowX-250S arm through pick, transport, alignment, insertion, release, and verification. The measured four-peg cycle averages 90.206 simulated seconds. Grasping uses actuated fingers assisted by a disclosed proximity-gated weld.

Source, configuration, licensed robot assets, tests, documentation, and measured results are included. Local virtual environments, caches, credentials, and generated MP4 recordings are excluded by the existing publication rules. The demo guide explains how to reproduce a real recording.

## Clone and run

```bash
git clone https://github.com/anjilshah/TEAM-ROCKET.git "TEAM ROCKET"
cd "TEAM ROCKET/TASK1_tri-wizard_peg_trial"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-build-isolation --no-deps -e .
.venv/bin/pytest -q
bash launch/run_autonomy.sh --headless --seed 0
```

For the graphical simulator, run `bash launch/run_autonomy.sh` from the task directory. Quote `TEAM ROCKET` in shell paths because the folder name contains a space.

## Documentation

- [Task setup](TASK1_tri-wizard_peg_trial/docs/setup.md)
- [Architecture](TASK1_tri-wizard_peg_trial/docs/architecture.md)
- [Development obstacles, fixes, and lessons](TASK1_tri-wizard_peg_trial/docs/development_journal.md)
- [Collision and object-finding recovery](TASK1_tri-wizard_peg_trial/docs/collision_and_perception_guide.md)
- [Measured validation](TASK1_tri-wizard_peg_trial/docs/validation.md)
- [Demonstration guide](TASK1_tri-wizard_peg_trial/docs/demo_guide.md)
- [Current trial results](TASK1_tri-wizard_peg_trial/results/performance/trials.csv)

The GitHub Actions workflow at the workspace root runs the Task 1 tests and a headless autonomous episode. The source model's license and attribution remain with the task files.

For this whole-workspace repository, run Git commands from `TEAM ROCKET`. The original task history is integrated into the workspace history, and its original Git metadata is backed up locally under `.task1-git-backup/`. The backup is excluded from publication. Task 1 is published as ordinary files rather than a submodule.
