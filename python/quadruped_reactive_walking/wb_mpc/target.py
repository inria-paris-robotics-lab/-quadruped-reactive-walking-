import numpy as np
from scipy.interpolate import KroghInterpolator
from ..tools.utils import make_initial_footstep
import pinocchio as pin


class Target:
    """Utility class to compute references for base and footstep."""

    def __init__(self, params):
        self.params = params
        self.dt_wbc = params.dt_wbc
        self.k_per_step = 160
        self.initial_delay = 1000
        self.initial_footsteps = make_initial_footstep(self.params.q_init)
        self.base_vel_ref = pin.Motion()

        if params.movement == "base_circle":
            self.initial_base = params.pose_init[:3]
            self.A = np.array([0.02, 0.015, 0.0])
            self.offset = np.array([0.0, 0.0, 0.0])
            self.freq = np.array([0.5, 0.5, 0.0])
            self.phase = np.array([0.0, 0.0, 0.0])
        elif params.movement == "walk":
            self.velocity_lin_target = np.array([0.5, 0, 0])
            self.velocity_ang_target = np.array([0, 0, 0])
            # dim 6
            self.base_vel_ref = pin.Motion(self.velocity_lin_target, self.velocity_ang_target)
        elif params.movement == "circle":
            self.A = np.array([0.05, 0.0, 0.04])
            self.offset = np.array([0.05, 0, 0.05])
            self.freq = np.array([0.5, 0.0, 0.5])
            self.phase = np.array([-np.pi / 2, 0.0, -np.pi / 2])
        elif params.movement == "step":
            foot_pose = self.initial_footsteps[:, 1]
            self.p0 = foot_pose
            self.p1 = foot_pose + np.array([0.025, 0.0, 0.03])
            self.v1 = np.array([0.5, 0.0, 0.0])
            self.p2 = foot_pose + np.array([0.05, 0.0, 0.0])

            self.T = self.k_per_step * self.dt_wbc
            self.ts = np.repeat(np.linspace(0, self.T, 3), 2)

            self.update_time = -1
        else:
            self.ramp_length = 100
            self.target_ramp_x = np.linspace(0.0, 0.0, self.ramp_length)
            self.target_ramp_y = np.linspace(0.0, 0.0, self.ramp_length)
            self.target_ramp_z = np.linspace(0.0, 0.05, self.ramp_length)

    def compute(self, k):
        if k < self.initial_delay:
            if self.params.movement == "base_circle" or self.params.movement == "walk":
                return self.initial_footsteps

        k -= self.initial_delay
        out = np.empty((3, 4))

        if self.params.movement == "base_circle":
            out[:] = self._evaluate_circle(k, self.initial_base)
        elif self.params.movement == "walk":
            out[:] = self.initial_footsteps
        else:
            out[:] = self.initial_footsteps
            if self.params.movement == "circle":
                out[:, 1] = self._evaluate_circle(k, self.initial_footsteps[:, 1])
            elif self.params.movement == "step":
                out[:, 1] = self._evaluate_step(1, k)
                out[2, 1] += 0.015
            else:
                out[0, 1] = self.target_ramp_x[k] if k < self.ramp_length else self.target_ramp_x[-1]
                out[1, 1] = self.target_ramp_y[k] if k < self.ramp_length else self.target_ramp_y[-1]
                out[2, 1] = self.target_ramp_z[k] if k < self.ramp_length else self.target_ramp_z[-1]

        return out

    def _evaluate_circle(self, k, initial_position):
        return initial_position + self.offset + self.A * np.sin(2 * np.pi * self.freq * k * self.dt_wbc + self.phase)

    def _evaluate_step(self, j, k):
        n_step = k // self.k_per_step
        if n_step % 2 == 0:
            return self.p0.copy() if (n_step % 4 == 0) else self.p2.copy()

        if n_step % 4 == 1:
            initial = self.p0
            target = self.p2
            velocity = self.v1
        else:
            initial = self.p2
            target = self.p0
            velocity = -self.v1

        k_step = k % self.k_per_step
        if n_step != self.update_time:
            self.update_interpolator(initial, target, velocity)
            self.update_time = n_step

        t = k_step * self.dt_wbc
        return self.krog(t)

    def update_interpolator(self, initial, target, velocity):
        self.y = [initial, np.zeros(3), self.p1, velocity, target, np.zeros(3)]
        self.krog = KroghInterpolator(self.ts, np.array(self.y))


def make_footsteps_and_refs(params, target: Target):
    """
    Build a list of both footstep position and base velocity references.
    Footsteps is a list of 3,4-matrices
    Base_vel_refs is a list of pin.Motion (6D values)
    """
    footsteps = []
    base_refs = []
    for k in range(params.N_gait):
        target_base_vel = pin.Motion(np.zeros(6))
        kk = k * params.mpc_wbc_ratio
        target_footstep = target.compute(kk).copy()

        footsteps.append(target_footstep)
        base_refs.append(target_base_vel)

    return footsteps, base_refs
