"""
Solve the OCP using the PROXDDP package.

Author:
    Wilson Jallet
"""
import time
import crocoddyl

import proxddp
from colorama import Fore

from .ocp_crocoddyl import CrocOCP
from quadruped_reactive_walking import Params
import abc
import numpy as np


def infNorm(x):
    return np.linalg.norm(x, ord=np.inf)


class AlgtrOCPAbstract(CrocOCP):
    """Solve the OCP using proxddp."""

    # Must be set by child class
    prox_ddp = None  # Solver instance

    @abc.abstractclassmethod
    def get_type_str():
        pass

    def __init__(
        self,
        params: Params,
        footsteps,
        base_refs,
    ):
        super().__init__(params, footsteps, base_refs)

        self.my_problem: proxddp.TrajOptProblem = proxddp.croc.convertCrocoddylProblem(
            self.problem
        )

        self.num_threads = params.ocp.num_threads
        if hasattr(self.problem, "num_threads"):
            self.problem.num_threads = self.num_threads
        self.my_problem.setNumThreads(self.num_threads)

        self.verbose = proxddp.QUIET
        if params.ocp.verbose:
            self.verbose = proxddp.VERBOSE
            self.ddp.setCallbacks([crocoddyl.CallbackVerbose()])

        self.prox_ddp.verbose = self.verbose
        self.prox_ddp.max_iters = self.max_iter
        self.prox_ddp.setup(self.my_problem)

        self.x_solver_errs = []
        self.u_solver_errs = []
        self.ff_errs = []
        self.fb_errs = []
        self.prox_stops_2 = []
        self.croc_stops_2 = []
        self.prox_stops = []
        self.croc_stops = []
        self.prox_iters = []
        self.croc_iters = []

    def solve(self, k, xs_init=None, us_init=None):
        t_start = time.time()
        self.my_problem.x0_init = self.x0

        t_update = time.time()
        self.t_update = t_update - t_start
        nsteps = self.my_problem.num_steps

        if xs_init is None or us_init is None:
            xs = [self.x0] * (nsteps + 1)
            us = self.problem.quasiStatic([self.x0] * nsteps)
        else:
            assert len(xs) == nsteps + 1
            assert len(us) == nsteps
            xs = xs_init
            us = us_init

        t_warm_start = time.time()
        self.t_warm_start = t_warm_start - t_update

        maxiter = self.max_iter if k > 0 else self.init_max_iters
        self.prox_ddp.max_iters = maxiter
        self.prox_ddp.run(self.my_problem, xs, us)

        # compute proxddp's criteria
        ws = self.prox_ddp.workspace
        if hasattr(ws, "Qus_ddp"):
            Qus = ws.Qus_ddp
        else:
            Qus = [q.Qu for q in ws.q_params]
        prox_norm_2 = sum(q.dot(q) for q in Qus)

        res = self.prox_ddp.results
        prox_norm_inf = max(res.primal_infeas, res.dual_infeas)
        self.prox_stops.append(prox_norm_inf)
        self.prox_stops_2.append(prox_norm_2)
        self.prox_iters.append(res.num_iters)

        t_ddp = time.time()
        self.t_ddp = t_ddp - t_warm_start

        self.t_solve = time.time() - t_start
        self.num_iters = res.num_iters

    def circular_append(self, action_model: crocoddyl.ActionModelAbstract):
        d = action_model.createData()
        self.problem.circularAppend(action_model, d)

        sm = proxddp.croc.ActionModelWrapper(action_model)
        self.my_problem.replaceStageCircular(sm)
        ws = self.prox_ddp.workspace
        ws.cycleAppend(sm.createData())

    def get_results(self):
        res = self.prox_ddp.results
        ws = self.prox_ddp.workspace  # noqa
        feedbacks = [-K.copy() for K in res.controlFeedbacks()]
        xs = res.xs.tolist()
        us = res.us.tolist()
        # if COMPARE:
        #     ffwds = [-k.copy() for k in res.controlFeedforwards()]
        #     xerr = [infNorm(self.state.diff(x1, x2)) for x1, x2 in zip(xs, self.ddp.xs)]
        #     uerr = [infNorm(u1 - u2) for u1, u2 in zip(us, self.ddp.us)]
        #     fberr = [infNorm(fb1 - fb2) for fb1, fb2 in zip(feedbacks, self.ddp.K)]
        #     fferr = [infNorm(f1 - f2) for f1, f2 in zip(ffwds, self.ddp.k)]
        #     self.x_solver_errs.extend(xerr)
        #     self.u_solver_errs.extend(uerr)
        #     self.ff_errs.extend(fferr)
        #     self.fb_errs.extend(fberr)

        return (
            self.current_gait.copy(),
            xs,
            us,
            feedbacks,
            self.t_ddp,
        )

    def clear(self):
        self.x_solver_errs.clear()
        self.u_solver_errs.clear()
        self.ff_errs.clear()
        self.fb_errs.clear()

        self.prox_stops.clear()
        self.prox_stops_2.clear()
        self.prox_iters.clear()

        self.croc_stops.clear()
        self.croc_stops_2.clear()
        self.croc_iters.clear()


class AlgtrOCPFDDP(AlgtrOCPAbstract):
    """Solve the OCP using fddp."""

    def __init__(
        self,
        params: Params,
        footsteps,
        base_refs,
    ):
        print(Fore.BLUE + "[using SolverFDDP]")
        self.prox_ddp = proxddp.SolverFDDP(params.ocp.tol)
        print(Fore.RESET)
        super().__init__(params, footsteps, base_refs)

    def get_type_str():
        return "algtr-fddp"


class AlgtrOCPProx(AlgtrOCPAbstract):
    """Solve the OCP using proxddp."""

    def __init__(
        self,
        params: Params,
        footsteps,
        base_refs,
    ):
        print(Fore.GREEN + "[using SolverProxDDP]")
        mu_init = 1e-10
        self.prox_ddp = proxddp.SolverProxDDP(params.ocp.tol, mu_init, 0.0)
        self.prox_ddp.mu_min = 1e-12
        self.prox_ddp.reg_init = 1e-9
        self.prox_ddp.ldlt_algo_choice = proxddp.LDLT_DENSE
        self.prox_ddp.max_refinement_steps = 0
        print(Fore.RESET)
        super().__init__(params, footsteps, base_refs)

    def get_type_str():
        return "algtr-prox"
