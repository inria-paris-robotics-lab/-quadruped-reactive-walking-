"""
Construct an OCP for jumping.
"""
import crocoddyl
import numpy as np
import pinocchio as pin

from quadruped_reactive_walking import Params
from crocoddyl import CostModelSum
from . import walking


class JumpOCPBuilder:
    def __init__(self, params: Params, base_vel_refs):
        self.params = params
        self._base_builder = walking.WalkingOCPBuilder(params, base_vel_refs)
        self.task = self._base_builder.task
        self.state = self._base_builder.state
        self.rdata = self._base_builder.rdata

        self.x0 = self.task.x0
        self.jump_spec = params.task["jump"]
        self.ground_models_1 = self.create_ground_models()
        jump_vel = np.asarray(self.jump_spec["jump_velocity"])
        jump_vel = pin.Motion(jump_vel)
        self.jump_models = self.create_jump_model(jump_vel)
        self.landing_model = None
        self.problem = crocoddyl.ShootingProblem(self.x0, *self.build_timeline())

    def build_timeline(self):
        N = self.params.N_gait
        t_jump = self.jump_spec["t_jump"]
        t_land = self.jump_spec["t_land"]
        k0 = int(t_jump / self.params.dt_mpc)
        k1 = int(t_land / self.params.dt_mpc) + 1
        assert k1 > k0, "Landing time should be larger than jumping time"
        N_jump = k1 - k0
        ground_rms, ground_tm = self.ground_models_1
        assert k0 < len(ground_rms)
        rms = ground_rms[:k0].copy()
        rms += [self.jump_models] * N_jump
        N_land = N - k1

        rms += ground_rms[-N_land:].copy()
        assert len(rms) == N
        return rms, ground_tm

    def create_ground_models(self):
        rms = []
        support_feet = np.asarray(self.task.feet_ids)
        for k in range(self.params.N_gait):
            m = self._base_builder.make_running_model(support_feet, [], None)
            rms.append(m)
        return rms, self._base_builder.make_terminal_model(support_feet)

    def get_num_contacts(self, m):
        return len(m.differential.contacts.active_set)

    def create_jump_model(self, base_vel_ref: pin.Motion):
        support_feet = np.array([])
        m = self._base_builder._create_standard_model(support_feet)
        costs: CostModelSum = m.differential.costs
        self._base_builder._add_base_vel_cost(base_vel_ref, costs)
        for i in self.task.feet_ids:
            self._base_builder._add_fly_high_cost(i, costs)
            self._base_builder._add_vert_velocity_cost(i, costs)
        return m
