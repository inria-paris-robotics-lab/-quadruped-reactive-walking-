import numpy as np
import pinocchio as pin
import crocoddyl

from typing import List, Optional
from quadruped_reactive_walking import Params, ResidualModelFlyHigh
from ..wb_mpc import task_spec
from ..tools.utils import no_copy_roll, no_copy_roll_insert
from .common import OCPBuilder
from crocoddyl import (
    ActivationBounds,
    ActivationModelWeightedQuad,
    StateMultibody,
    CostModelResidual,
    CostModelSum,
    DifferentialActionModelContactFwdDynamics,
    IntegratedActionModelEuler,
    ResidualModelState,
    ResidualModelControl,
)


class WalkingOCPBuilder(OCPBuilder):
    """Builder class to define the walking OCP."""

    def __init__(self, params: Params, base_vel_refs):
        super().__init__(params)
        self.task = task_spec.TaskSpec(params)
        self.state = StateMultibody(self.rmodel)
        self.rdata = self.rmodel.createData()

        self.life_gait = params.gait
        self.starting_gait = np.ones((params.starting_nodes, 4), dtype=np.int32)
        self.ending_gait = np.ones((params.ending_nodes, 4), dtype=np.int32)
        self.current_gait = np.append(
            self.starting_gait,
            self.ending_gait[0].reshape(1, -1),
            axis=0,
        )

        self.life_rm, self.life_tm = self.initialize_models_from_gait(self.life_gait, base_vel_refs)
        self.start_rm, self.start_tm = self.initialize_models_from_gait(self.starting_gait)
        self.end_rm, self.end_tm = self.initialize_models_from_gait(self.ending_gait)

        self.x0 = self.task.x0
        self.problem = crocoddyl.ShootingProblem(self.x0, self.start_rm, self.start_tm)

    def select_next_model(self, k, current_gait, base_vel_ref):
        feet_ids = np.asarray(self.task.feet_ids)

        t = int(k / self.params.mpc_wbc_ratio) - 1

        if t < len(self.start_rm):
            mask = self.life_gait[t] == 1
            support_feet = feet_ids[mask]
            model = self.life_rm[t]
            no_copy_roll_insert(current_gait, self.life_gait[t])

        elif t < len(self.start_rm) + len(self.life_rm) * self.params.gait_repetitions:
            no_copy_roll(self.life_gait)
            mask = self.life_gait[-1] == 1
            support_feet = feet_ids[mask]
            model = self.problem.runningModels[0]
            no_copy_roll_insert(current_gait, self.life_gait[-1])

        else:
            i = 0 if t == len(self.start_rm) + len(self.life_rm) * self.params.gait_repetitions else 1
            # choose to pich the node with impact or not
            support_feet = feet_ids[self.ending_gait[i] == 1]
            model = self.end_rm[i]
            no_copy_roll_insert(current_gait, self.ending_gait[i])
            base_vel_ref = None

        if base_vel_ref is not None:
            base_vel_ref = pin.Motion(base_vel_ref)
        return model, support_feet, base_vel_ref

    @property
    def rmodel(self):
        return self.task.model

    def initialize_models_from_gait(self, gait, base_vel_refs=None):
        """Create action models (problem stages) from a gait matrix and other optional data."""
        # both or neither must be none
        running_models = []
        feet_ids = np.asarray(self.task.feet_ids)
        for t in range(gait.shape[0]):
            support_feet_ids = feet_ids[gait[t] == 1]
            base_vel_ref = base_vel_refs[t] if base_vel_refs is not None else None
            has_switched = np.any(gait[t] != gait[t - 1])
            switch_matrix = gait[t] if has_switched else np.array([])
            switch_feet = feet_ids[switch_matrix == 1]
            running_models.append(self.make_running_model(support_feet_ids, switch_feet, base_vel_ref))

        support_feet_ids = feet_ids[gait[-1] == 1]
        terminal_model = self.make_terminal_model(support_feet_ids)

        return running_models, terminal_model

    def _create_standard_model(self, support_feet) -> crocoddyl.IntegratedActionModelAbstract:
        """
        Create a standard integrated action model, to be modified by the callee.

        :param state: swinging foot task
        :param support_feet: list of support feet ids
        :return action model for a swing foot phase
        """
        pin.forwardKinematics(self.rmodel, self.rdata, self.task.q0)
        pin.updateFramePlacements(self.rmodel, self.rdata)
        actuation = crocoddyl.ActuationModelFloatingBase(self.state)
        nu = actuation.nu

        zero_vec = np.zeros(3)

        contacts = crocoddyl.ContactModelMultiple(self.state, nu)
        for i in self.task.feet_ids:
            name = self.rmodel.frames[i].name + "_contact"
            contact = crocoddyl.ContactModel3D(
                self.state,
                i,
                zero_vec,
                pin.LOCAL_WORLD_ALIGNED,
                nu,
                self.task.baumgarte_gains,
            )
            contacts.addContact(name, contact)
            contacts.changeContactStatus(name, i in support_feet)

        costs = CostModelSum(self.state, nu)
        residual = ResidualModelState(self.state, self.task.xref, nu)
        activation = ActivationModelWeightedQuad(self.task.state_reg_w**2)
        state_cost = CostModelResidual(self.state, activation, residual)
        costs.addCost("state_reg", state_cost, 1)

        state_bound_residual = ResidualModelState(self.state, self.task.xref, nu)
        activation = crocoddyl.ActivationModelWeightedQuadraticBarrier(
            ActivationBounds(-self.task.state_limit, self.task.state_limit),
            self.task.state_bound_w**2,
        )
        state_bound_cost = CostModelResidual(self.state, activation, state_bound_residual)
        costs.addCost("state_limitBound", state_bound_cost, 1)

        diff = DifferentialActionModelContactFwdDynamics(self.state, actuation, contacts, costs, 0.0, True)
        return IntegratedActionModelEuler(diff, self.params.dt_mpc)

    def make_running_model(
        self,
        support_feet,
        switch_feet,
        base_vel_ref: Optional[pin.Motion],
    ):
        """
        Add all the costs to the running models
        """
        model = self._create_standard_model(support_feet)
        costs = model.differential.costs
        for i in self.task.feet_ids:
            start_pos = self.rdata.oMf[i].translation

            self._add_friction_cost(i, support_feet, costs)
            self._add_force_reg(i, model)
            if self.has_ground_collision:
                self._add_ground_coll_penalty(i, costs, start_pos)
            if self.has_fly_high:
                self._add_fly_high_cost(i, costs)
            self._add_vert_velocity_cost(i, costs)

            # Fake impact
            if i in switch_feet and i in support_feet:
                self._add_impact_costs(i, costs, start_pos)

        if self.has_base_vel_cost:
            if base_vel_ref is not None:
                ref = pin.Motion(base_vel_ref)
            else:
                ref = pin.Motion.Zero()
            self._add_base_vel_cost(ref, costs)

        self._add_control_costs(costs)

        self.update_tracking_costs(costs, base_vel_ref, support_feet)
        return model

    def _add_control_costs(self, costs: CostModelSum):
        nu = costs.nu
        control_reg = CostModelResidual(self.state, ResidualModelControl(self.state, self.task.uref))
        costs.addCost("control_reg", control_reg, self.task.control_reg_w)

        control_bound_activation = crocoddyl.ActivationModelQuadraticBarrier(
            ActivationBounds(-self.task.effort_limit, self.task.effort_limit)
        )
        control_bound = CostModelResidual(self.state, control_bound_activation, ResidualModelControl(self.state, nu))
        costs.addCost("control_bound", control_bound, self.task.control_bound_w)

    def make_terminal_model(self, support_feet):
        """
        Add the final velocity cost to the terminal model
        """
        model = self._create_standard_model(support_feet)
        nu = model.differential.actuation.nu
        residual = ResidualModelState(self.state, self.task.xref, nu)
        activation = ActivationModelWeightedQuad(self.task.terminal_velocity_w**2)
        state_cost = CostModelResidual(self.state, activation, residual)
        model.differential.costs.addCost("terminal_velocity", state_cost, 1)
        return model

    def _add_friction_cost(self, i: int, support_feet, costs: CostModelSum):
        nu = costs.nu
        # Contact forces
        cone = crocoddyl.FrictionCone(self.task.Rsurf, self.task.friction_mu, 4, False, 3)
        residual = crocoddyl.ResidualModelContactFrictionCone(self.state, i, cone, nu)
        activation = crocoddyl.ActivationModelQuadraticBarrier(ActivationBounds(cone.lb, cone.ub))
        friction_cost = CostModelResidual(self.state, activation, residual)
        friction_name = self.rmodel.frames[i].name + "_friction_cost"
        costs.addCost(friction_name, friction_cost, self.task.friction_cone_w)
        costs.changeCostStatus(friction_name, i in support_feet)

    def _add_force_reg(self, i: int, m: DifferentialActionModelContactFwdDynamics):
        costs = m.differential.costs
        nu = costs.nu
        name = "{}_forceReg".format(self.rmodel.frames[i].name)
        nc = len(m.differential.contacts.active_set)
        ref_force = np.array([0, 0, self.task.robot_weight / nc])
        ref_force = pin.Force(ref_force, ref_force * 0.0)
        force_reg = CostModelResidual(
            self.state,
            crocoddyl.ResidualModelContactForce(self.state, i, ref_force, 3, nu),
        )
        costs.addCost(name, force_reg, self.task.force_reg_w)
        costs.changeCostStatus(name, False)

    def _add_ground_coll_penalty(self, i: int, costs: CostModelSum, start_pos):
        nu = costs.nu

        # Swing foot
        ground_coll_res = crocoddyl.ResidualModelFrameTranslation(self.state, i, start_pos, nu)

        bounds = ActivationBounds(np.array([-1000, -1000, start_pos[2]]), np.array([1000, 1000, 1000]))
        ground_coll_activ = crocoddyl.ActivationModelQuadraticBarrier(bounds)
        ground_coll_cost = CostModelResidual(self.state, ground_coll_activ, ground_coll_res)

        name = "{}_groundCol".format(self.rmodel.frames[i].name)
        costs.addCost(
            name,
            ground_coll_cost,
            self.task.ground_collision_w,
        )
        costs.changeCostStatus(name, False)

    @property
    def has_ground_collision(self):
        return self.task.ground_collision_w > 0

    def _add_fly_high_cost(self, i: int, costs: CostModelSum):
        nu = costs.nu
        fly_high_cost = CostModelResidual(
            self.state,
            ResidualModelFlyHigh(self.state, i, self.task.fly_high_sigma_height, nu),
        )
        name = "{}_flyHigh".format(self.rmodel.frames[i].name)
        costs.addCost(
            name,
            fly_high_cost,
            self.task.fly_high_w,
        )
        costs.changeCostStatus(name, False)

    @property
    def has_fly_high(self):
        return self.task.fly_high_w > 0

    def _add_vert_velocity_cost(self, i: int, costs: CostModelSum):
        nu = costs.nu
        vertical_velocity_reg_residual = crocoddyl.ResidualModelFrameVelocity(
            self.state,
            i,
            pin.Motion.Zero(),
            pin.ReferenceFrame.WORLD,
            nu,
        )
        vertical_velocity_activation = ActivationModelWeightedQuad(np.array([0, 0, 1, 0, 0, 0]))

        name = "{}_vel_zReg".format(self.rmodel.frames[i].name)
        vertical_velocity_reg_cost = CostModelResidual(
            self.state,
            vertical_velocity_activation,
            vertical_velocity_reg_residual,
        )
        costs.addCost(
            name,
            vertical_velocity_reg_cost,
            self.task.vertical_velocity_reg_w,
        )

    @property
    def has_impact_alt_cost(self):
        return self.task.impact_altitude_w > 0

    @property
    def has_impact_vel_cost(self):
        return self.task.impact_velocity_w > 0

    def _add_impact_costs(self, i: int, costs: CostModelSum, start_pos):
        nu = costs.nu
        if self.has_impact_alt_cost:
            impact_activ = ActivationModelWeightedQuad(np.array([0, 0, 1]))
            impact_alt_cost = CostModelResidual(
                self.state,
                impact_activ,
                crocoddyl.ResidualModelFrameTranslation(self.state, i, start_pos, nu),
            )
            costs.addCost(
                "{}_altitudeimpact".format(self.rmodel.frames[i].name),
                impact_alt_cost,
                self.task.impact_altitude_w / self.params.dt_mpc,
            )

        if self.has_impact_vel_cost:
            impact_vel_cost = CostModelResidual(
                self.state,
                crocoddyl.ResidualModelFrameVelocity(
                    self.state,
                    i,
                    pin.Motion.Zero(),
                    pin.ReferenceFrame.WORLD,
                    nu,
                ),
            )
            costs.addCost(
                "{}_velimpact".format(self.rmodel.frames[i].name),
                impact_vel_cost,
                self.task.impact_velocity_w / self.params.dt_mpc,
            )

    @property
    def has_base_vel_cost(self):
        return self.task.base_velocity_tracking_w > 0

    def _add_base_vel_cost(self, vel_ref: pin.Motion, costs: CostModelSum):
        nu = costs.nu
        name = "base_velocity_tracking"

        residual_base_velocity = crocoddyl.ResidualModelFrameVelocity(
            self.state, self.task.base_id, vel_ref, pin.LOCAL, nu
        )
        base_velocity = CostModelResidual(self.state, residual_base_velocity)
        costs.addCost(name, base_velocity, self.task.base_velocity_tracking_w)

    def update_model(
        self,
        model,
        base_vel_ref: Optional[pin.Motion],
        support_feet,
        is_terminal=False,
    ):
        """Update each stage of the OCP: the contact status of the dynamics and cost functions."""
        for i in self.task.feet_ids:
            name = self.rmodel.frames[i].name + "_contact"
            model.differential.contacts.changeContactStatus(name, i in support_feet)
        if not is_terminal:
            self.update_tracking_costs(model.differential.costs, base_vel_ref, support_feet)

    def update_tracking_costs(self, costs, base_vel_ref: pin.Motion, support_feet):
        index = 0
        for i in self.task.feet_ids:
            name = "{}_forceReg".format(self.rmodel.frames[i].name)
            costs.changeCostStatus(name, i in support_feet)

            if self.has_ground_collision:
                name = "{}_groundCol".format(self.rmodel.frames[i].name)
                costs.changeCostStatus(name, i not in support_feet)

            if self.has_fly_high:
                name = "{}_flyHigh".format(self.rmodel.frames[i].name)
                costs.changeCostStatus(name, i not in support_feet)

            name = "{}_vel_zReg".format(self.rmodel.frames[i].name)
            costs.changeCostStatus(name, i not in support_feet)

        if base_vel_ref is not None and self.has_base_vel_cost:
            name = "base_velocity_tracking"
            costs.costs[name].cost.residual.reference = base_vel_ref