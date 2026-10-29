import os
# EGL works without an X server; GUI launchers explicitly select GLFW.
os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco
import numpy as np
from peg_sorting.utils.helpers import config, project_root
from peg_sorting.simulation.scene_manager import build_scene


class MujocoEnv:
    def __init__(self, seed=0, regenerate=True):
        if regenerate:
            self.model = mujoco.MjModel.from_xml_string(build_scene(seed, destination=False))
        else:
            self.model = mujoco.MjModel.from_xml_path(str(project_root() / "simulation/scenes/peg_sorting.xml"))
        self.data = mujoco.MjData(self.model)
        self.joints = config("robot")["arm_joints"]
        self.qids = np.array([self.model.joint(n).qposadr[0] for n in self.joints])
        self.dofs = np.array([self.model.joint(n).dofadr[0] for n in self.joints])
        self.actuators = np.array([self.model.actuator(n).id for n in self.joints])
        self.limits = np.array([self.model.joint(n).range for n in self.joints])
        self.site_id = self.model.site("grasp_site").id
        # Park toward -Y so the arm cannot cover the board during perception.
        self.data.qpos[self.qids] = [-np.pi / 2, -1.1, 1.3, 0, -.2, 0]
        self.data.qpos[self.model.joint("left_finger").qposadr] = .037
        self.data.qpos[self.model.joint("right_finger").qposadr] = -.037
        self.data.ctrl[self.actuators] = self.data.qpos[self.qids]
        self.data.ctrl[self.model.actuator("gripper").id] = .037
        mujoco.mj_forward(self.model, self.data)
        self.on_step = None
        self.stop_requested = False
        self.held_shape = None

    def step(self, seconds=None, steps=None):
        count = steps if steps is not None else max(1, round(seconds / self.model.opt.timestep))
        for _ in range(count):
            if self.stop_requested:
                raise InterruptedError("Simulation motion stopped by operator")
            # Feed-forward bias compensation; movement still uses force-limited servos.
            self.data.qfrc_applied[self.dofs] = self.data.qfrc_bias[self.dofs]
            mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all():
                raise RuntimeError("Nonfinite MuJoCo state")
            if self.on_step:
                self.on_step(self)

    def site_pose(self, data=None):
        data = data or self.data
        return data.site_xpos[self.site_id].copy(), data.site_xmat[self.site_id].reshape(3, 3).copy()

    def hold(self):
        self.data.ctrl[self.actuators] = self.data.qpos[self.qids]
