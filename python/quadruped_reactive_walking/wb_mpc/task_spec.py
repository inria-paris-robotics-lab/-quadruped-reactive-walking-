import numpy as np
import example_robot_data as erd
import pinocchio as pin
from quadruped_reactive_walking import Params


class TaskSpecBase:
    def __init__(self, params: Params, frozen_names=[]):
        self.robot = erd.load("solo12")
        self.q0 = self.robot.q0
        self.q0[:7] = np.array([0.0, 0.0, params.h_ref, 0, 0, 0, 1])
        self.q0[7:] = params.q_init

        self.model: pin.Model = self.robot.model
        self.collision_model = self.robot.collision_model
        self.visual_model = self.robot.visual_model

        self.robot_weight = (
            -sum([Y.mass for Y in self.model.inertias]) * self.model.gravity.linear[2]
        )

        self.frozen_names = frozen_names
        self.frozen_idxs = [self.model.getJointId(id) for id in frozen_names]
        self.freeze()
        nu_frozen = sum([self.model.joints[jid].nv for jid in self.frozen_idxs])

        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nx = self.nq + self.nv
        self.ndx = 2 * self.nv
        # -1 to take into account the freeflyer
        self.nu = 12 - nu_frozen
        if len(frozen_names) == 0:
            assert self.nu == 12

        self.effort_limit = np.ones(self.nu) * 2.5

        self.v0 = np.zeros(18)
        self.x0 = np.concatenate([self.q0, self.v0])

        pin.centerOfMass(self.robot.model, self.robot.data, self.q0, np.zeros((18, 1)))
        self.u0 = np.array([0.0, 0.0, 9.81 * self.robot.data.mass[0] / 4.0] * 4)

        self.baumgarte_gains = np.array([0.0, 100.0])

        self.feet_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
        self.feet_ids = [self.model.getFrameId(f) for f in self.feet_names]

        self.Rsurf = np.eye(3)

    def freeze(self):
        geom_models = [self.visual_model, self.collision_model]
        self.model, geometric_models_reduced = pin.buildReducedModel(
            self.model,
            list_of_geom_models=geom_models,
            list_of_joints_to_lock=self.frozen_idxs,
            reference_configuration=self.q0,
        )
        self.rdata = self.model.createData()
        self.visual_model = geometric_models_reduced[0]
        self.collision_model = geometric_models_reduced[1]


class TaskSpec(TaskSpecBase):
    def __init__(self, params: Params):
        super().__init__(params)

        self.useFixedBase = 0
        self.base_id = self.model.getFrameId("base_link")

        self.state_limit = np.concatenate(
            [np.full(18, np.inf), np.zeros(6), np.ones(12) * 800]
        )

        task_pms = params.task["walk"]

        # Cost function weights
        self.friction_mu = task_pms["friction_mu"]
        self.fly_high_sigma_height = task_pms["fly_high_sigma_height"]
        self.fly_high_w = task_pms["fly_high_w"]
        self.ground_collision_w = task_pms["ground_collision_w"]
        self.vertical_velocity_reg_w = task_pms["vertical_velocity_reg_w"]

        self.base_velocity_tracking_w = task_pms["base_velocity_tracking_w"]
        self.foot_tracking_w = task_pms["foot_tracking_w"]

        self.impact_altitude_w = task_pms["impact_altitude_w"]
        self.impact_velocity_w = task_pms["impact_velocity_w"]
        self.friction_cone_w = task_pms["friction_cone_w"]

        self.control_bound_w = task_pms["control_bound_w"]
        self.control_reg_w = task_pms["control_reg_w"]
        self.state_reg_w = np.array(
            [0] * 3 + [0] * 3 + [1e2 * 3] * 12 + [0] * 6 + [1e1 * 2] * 12
        )
        self.state_bound_w = np.array([0] * 18 + [0] * 6 + [0] * 12)
        self.terminal_velocity_w = np.zeros(2 * self.nv)
        self.terminal_velocity_w[self.nv :] = task_pms["terminal_velocity_w"]
        self.force_reg_w = task_pms["force_reg_w"]

        self.xref = self.x0
        self.uref = np.array(task_pms["uref"])


class TaskSpecFull(TaskSpecBase):
    def __init__(self, params: Params):
        frozen_names = ["root_joint"]

        super().__init__(params, frozen_names)

        self.useFixedBase = 1

        # Cost function weights
        self.mu = params.task["mu"]
        self.foot_tracking_w = 1e4
        self.friction_cone_w = 1e3
        self.control_bound_w = 1e3
        self.control_reg_w = 1e0
        self.state_reg_w = np.array(
            [1e2] * 3 + [1e-2] * 3 + [1e2] * 6 + [1e1] * 3 + [1e0] * 3 + [1e1] * 6
        )
        self.terminal_velocity_w = np.array([0] * 12 + [1e3] * 12)

        self.q0_reduced = self.q0[7:]
        self.v0_reduced = np.zeros(self.nv)
        self.x0_reduced = np.concatenate([self.q0_reduced, self.v0_reduced])

        self.xref = self.x0_reduced
        self.uref = self.u0
