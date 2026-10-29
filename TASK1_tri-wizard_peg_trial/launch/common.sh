#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
if [[ ! -x .venv/bin/python ]]; then
  echo 'Missing .venv. Run: python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt && .venv/bin/python -m pip install --no-build-isolation --no-deps -e .' >&2
  exit 2
fi
if ! .venv/bin/python -c 'import mujoco, cv2, peg_sorting' 2>/dev/null; then
  echo 'Dependencies missing. Follow docs/setup.md.' >&2
  exit 2
fi
if [[ ! -f simulation/robots/wx250s/wx250s.xml ]]; then
  echo 'Robot model missing. Run: .venv/bin/python scripts/fetch_robot.py' >&2
  exit 2
fi
case " $* " in
  *' --headless '*) export MUJOCO_GL="${MUJOCO_GL:-egl}" ;;
  *) export MUJOCO_GL="${MUJOCO_GL:-glfw}"
     if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
       echo 'No display. Pass --headless for simulation, vision, or autonomy.' >&2
       exit 2
     fi ;;
esac
