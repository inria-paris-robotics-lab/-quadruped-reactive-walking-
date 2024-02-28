import crocoddyl
import pinocchio as pin
import numpy as np
import time

from colorama import Fore
from .ocp_abstract import OCPAbstract
from typing import Optional
from quadruped_reactive_walking import Params
from . import task_spec
from ..ocp_defs.walking import WalkingOCPBuilder


class CrocOCP(OCPAbstract):
    """
    Generate a Crocoddyl OCP for the control task.
    """

    def __init__(self, params: Params, base_vel_refs):
        super().__init__(params)
        self.task = task_spec.TaskSpec(params)

        self._builder = WalkingOCPBuilder(params, base_vel_refs)
        self.rdata = self._builder.rdata
        self.current_gait = self._builder.current_gait

        # Set the problem parameters
        self.t_problem_update = 0
        self.t_update_last_model = 0.0
        self.t_shift = 0.0

        self.x0 = self.task.x0

        self.croc_problem = self._builder.problem
        self.ddp = crocoddyl.SolverFDDP(self.croc_problem)
        if params.ocp.verbose:
            self.ddp.setCallbacks([crocoddyl.CallbackVerbose()])
        if self.warm_start_empty():
            print(Fore.CYAN + "No warm-start found, initializing..." + Fore.RESET)
            self.xs_init = [self.x0] * (self.ddp.problem.T + 1)
            self.us_init = self.ddp.problem.quasiStatic([self.x0] * self.ddp.problem.T)

    @property
    def rmodel(self):
        return self.task.model

    def get_type_str():
        return "croc"

    def solve(self, k):
        t_start = time.time()

        t_update = time.time()
        self.t_update = t_update - t_start

        self._check_ws_dim()

        t_warm_start = time.time()
        self.t_warm_start = t_warm_start - t_update

        self.ddp.solve(
            self.xs_init,
            self.us_init,
            self.max_iter if k > 0 else self.init_max_iters,
            False,
        )

        t_ddp = time.time()
        self.t_ddp = t_ddp - t_warm_start

        self.t_solve = time.time() - t_start
        self.num_iters = self.ddp.iter

    def push_node(self, k, x0, base_vel_ref: Optional[pin.Motion]):
        """
        Create a shooting problem for a simple walking gait.

        :param k: current MPC iteration
        :param x0: initial condition
        :param base_pose: 1D array
        """
        self.x0 = x0
        pin.forwardKinematics(self.rmodel, self.rdata, self.x0[: self.task.nq])
        pin.updateFramePlacements(self.rmodel, self.rdata)

        self.croc_problem.x0 = self.x0

        if k == 0:
            return

        model, support_feet, base_vel_ref = self._builder.select_next_model(k, self.current_gait, base_vel_ref)
        self._builder.update_model(model, base_vel_ref, support_feet)
        self.circular_append(model)
        self.cycle_warm_start()

    def circular_append(self, m):
        d = m.createData()
        self.croc_problem.circularAppend(m, d)

    def get_results(self, window_size=None):
        self.xs_init = self.ddp.xs
        self.us_init = self.ddp.us
        if window_size is None:
            window_size = len(self.ddp.us)
        return (
            self.current_gait,
            self.ddp.xs[: window_size + 1],
            self.ddp.us[:window_size],
            self.ddp.K[:window_size].tolist(),
            self.t_ddp,
        )

    def get_croco_forces(self):
        d = self.ddp.problem.runningDatas[0]
        cnames = d.differential.multibody.contacts.contacts.todict().keys()
        forces = {n: [] for n in cnames}

        for m in self.ddp.problem.runningDatas:
            mdict = m.differential.multibody.contacts.contacts.todict()
            for n in cnames:
                if n in mdict:
                    forces[n] += [(mdict[n].jMf.inverse() * mdict[n].f).linear]
                else:
                    forces[n] += [np.array([0, 0, 0])]
        for f in forces:
            forces[f] = np.array(forces[f])
        return forces

    def get_croco_forces_ws(self):
        forces = []

        for m in self.ddp.problem.runningDatas:
            mdict = m.differential.multibody.contacts.contacts.todict()
            f_tmp = []
            for n in mdict:
                f_tmp += [(mdict[n].jMf.inverse() * mdict[n].f).linear]
            forces += [np.concatenate(f_tmp)]
        return forces

    def get_croco_acc(self):
        return [m.differential.xout for m in self.ddp.problem.runningDatas]
