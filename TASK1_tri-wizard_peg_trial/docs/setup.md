# Setup

Validated platform: Ubuntu 22.04.4 LTS, x86-64, Python 3.10.12. No system Python packages were changed. The project uses a local virtual environment.

The Git repository and workspace root is `TEAM ROCKET/`; the Python project root is its `TASK1_tri-wizard_peg_trial/` subfolder. The workspace folder name contains a space, so quote that path in shell commands. For a fresh checkout, run from the directory that should contain the workspace:

```bash
git clone https://github.com/anjilshah/TEAM-ROCKET.git "TEAM ROCKET"
cd "TEAM ROCKET/TASK1_tri-wizard_peg_trial"
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-build-isolation --no-deps -e .
.venv/bin/python -m pip check
.venv/bin/pytest -q
bash launch/run_autonomy.sh --headless --seed 0
```

The public repository is [anjilshah/TEAM-ROCKET](https://github.com/anjilshah/TEAM-ROCKET), with this Python project as a normal subfolder. The source upload and original commit history have been verified. The existing local checkout additionally retains its repaired virtual environment and local demo videos; those generated files are excluded from Git. A fresh clone installs its own environment using the commands above.

For the existing checkout, enter it from its workspace parent and run directly:

```bash
cd "TEAM ROCKET/TASK1_tri-wizard_peg_trial"
bash launch/run_autonomy.sh --headless --seed 0
```

The virtual environment and editable-package paths have been repaired for this move. For a different copied/moved checkout, recreate a local environment from the lockfile rather than assuming another environment's absolute launcher paths remain valid. Do not copy `.venv/` as part of a fresh-machine installation.

Python venv support, Mesa/OpenGL/EGL libraries, and a working desktop display are host prerequisites. If absent, an Ubuntu administrator can install `python3-venv libgl1 libegl1-mesa libglfw3`. Installation of system packages is a separate privileged operation; it was not needed or performed here. OpenCV GUI mode additionally needs a working Qt/X11 or Wayland/XWayland environment. Headless rendering uses EGL and does not need a display server.

The robot is vendored, so normal setup does not contact Interbotix or install ROS. To restore missing robot files, run `.venv/bin/python scripts/fetch_robot.py`. The pinned revision and SHA-256 hashes are recorded in `simulation/robots/wx250s/provenance.json`. The fetcher refuses to overwrite local modifications to upstream files.

Run launchers from any directory by supplying their quoted path; they resolve the Python project root and its `.venv` themselves. From the directory containing the workspace, for example:

```bash
bash "TEAM ROCKET/TASK1_tri-wizard_peg_trial/launch/run_autonomy.sh" --headless --seed 0
```

From an activated environment, `peg-sort autonomy --headless --seed 0` is equivalent. This is an editable source installation; keep configuration and asset directories beside the source. Set `PEG_SORTING_ROOT` if using a different checkout location.

The six direct dependency versions are pinned in `requirements.txt`; `requirements-lock.txt` also pins transitive dependencies. The tested MuJoCo wheel includes its own simulator library. To inspect the environment:

```bash
.venv/bin/python --version
.venv/bin/python -m pip show mujoco opencv-python numpy scipy PyYAML pytest
```

Scene changes live in YAML and `scene_manager.py`. Runtime construction is in memory, so seeded trials can run without modifying the committed nominal scene. Regenerate the distributable XML with:

```bash
.venv/bin/python scripts/generate_demo_scene.py --seed 0
```
