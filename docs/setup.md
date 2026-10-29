# Setup

Validated platform: Ubuntu 22.04.4 LTS, x86-64, Python 3.10.12. No system Python packages were changed. The project uses a local virtual environment.

```bash
git clone https://github.com/anjilshah/tri-wizard-peg-trial.git
cd tri-wizard-peg-trial
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-build-isolation --no-deps -e .
.venv/bin/python -m pip check
.venv/bin/pytest -q
bash launch/run_autonomy.sh --headless --seed 0
```

The clone command becomes usable after the owner approves publication and the repository is created. Until then, use the prepared local checkout or copy the complete project directory excluding `.venv/` and generated logs.

Python venv support, Mesa/OpenGL/EGL libraries, and a working desktop display are host prerequisites. If absent, an Ubuntu administrator can install `python3-venv libgl1 libegl1-mesa libglfw3`. Installation of system packages is a separate privileged operation; it was not needed or performed here. OpenCV GUI mode additionally needs a working Qt/X11 or Wayland/XWayland environment. Headless rendering uses EGL and does not need a display server.

The robot is vendored, so normal setup does not contact Interbotix or install ROS. To restore missing robot files, run `.venv/bin/python scripts/fetch_robot.py`. The pinned revision and SHA-256 hashes are recorded in `simulation/robots/wx250s/provenance.json`. The fetcher refuses to overwrite local modifications to upstream files.

Run launchers from any directory by supplying their absolute path; they resolve the repository root and `.venv` themselves. From an activated environment, `peg-sort autonomy --headless --seed 0` is equivalent. This is an editable source installation; keep configuration and asset directories beside the source. Set `PEG_SORTING_ROOT` if using a different checkout location.

The six direct dependency versions are pinned in `requirements.txt`; `requirements-lock.txt` also pins transitive dependencies. The tested MuJoCo wheel includes its own simulator library. To inspect the environment:

```bash
.venv/bin/python --version
.venv/bin/python -m pip show mujoco opencv-python numpy scipy PyYAML pytest
```

Scene changes live in YAML and `scene_manager.py`. Runtime construction is in memory, so seeded trials can run without modifying the committed nominal scene. Regenerate the distributable XML with:

```bash
.venv/bin/python scripts/generate_demo_scene.py --seed 0
```
