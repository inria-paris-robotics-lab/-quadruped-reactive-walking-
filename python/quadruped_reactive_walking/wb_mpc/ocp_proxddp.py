"""
Solve the OCP using the PROXDDP package.

Author:
    Wilson Jallet
"""
import time
import aligator
import crocoddyl
import numpy as np

from abc import abstractclassmethod
from colorama import Fore
from .ocp_crocoddyl import CrocOCP
from quadruped_reactive_walking import Params


def infNorm(x):
    return np.linalg.norm(x, ord=np.inf)


class AlgtrOCPAbstract(CrocOCP):
    """Solve the OCP using aligator."""

    # Must be set by child class
    solver = None  # Solver instance

    @abstractclassmethod
    def get_type_str():
        pass

    def __init__(
        self,
        params: Params,
        base_vel_refs,
    ):
        super().__init__(params, base_vel_refs)

        self.algtr_problem: aligator.TrajOptProblem = aligator.croc.convertCrocoddylProblem(self.croc_problem)

        self.num_threads = params.ocp.num_threads
        if hasattr(self.croc_problem, "num_threads"):
            self.croc_problem.num_threads = self.num_threads
        self.solver.setNumThreads(self.num_threads)

        self.verbose = aligator.QUIET
        if params.ocp.verbose:
            self.verbose = aligator.VERBOSE

        self.solver.verbose = self.verbose
        self.solver.max_iters = self.max_iter
        self.solver.setup(self.algtr_problem)

    def solve(self, k):
        t_start = time.time()
        self.algtr_problem.x0_init = self.x0

        t_update = time.time()
        self.t_update = t_update - t_start

        self._check_ws_dim()

        t_warm_start = time.time()
        self.t_warm_start = t_warm_start - t_update

        maxiter = self.max_iter if k > 0 else self.init_max_iters
        self.solver.max_iters = maxiter
        self.solver.run(self.algtr_problem, self.xs_init, self.us_init)
        # compute aligator's criteria
        res = self.solver.results

        t_ddp = time.time()
        self.t_ddp = t_ddp - t_warm_start

        self.t_solve = time.time() - t_start
        self.num_iters = res.num_iters

    def circular_append(self, action_model: crocoddyl.ActionModelAbstract):
        d = action_model.createData()
        self.croc_problem.circularAppend(action_model, d)

        sm = aligator.croc.ActionModelWrapper(action_model)
        self.algtr_problem.replaceStageCircular(sm)
        ws = self.solver.workspace
        ws.cycleAppend(sm.createData())

    def get_results(self, window_size=None):
        res = self.solver.results
        self.xs_init = res.xs
        self.us_init = res.us
        if window_size is None:
            window_size = len(res.us)
        feedbacks = res.controlFeedbacks()[:window_size]
        # flip sign because controller expects Crocoddyl's convention
        feedbacks = [-K.copy() for K in feedbacks]

        return (
            self.current_gait.copy(),
            res.xs[: window_size + 1],
            res.us[:window_size],
            feedbacks,
            self.t_ddp,
        )


class AlgtrOCPFDDP(AlgtrOCPAbstract):
    """Solve the OCP using fddp."""

    def __init__(
        self,
        params: Params,
        base_vel_refs,
    ):
        print(Fore.BLUE + "[using SolverFDDP]" + Fore.RESET)
        self.solver = aligator.SolverFDDP(params.ocp.tol)
        super().__init__(params, base_vel_refs)

    def get_type_str():
        return "algtr-fddp"


class AlgtrOCPProx(AlgtrOCPAbstract):
    """Solve the OCP using aligator."""

    def __init__(self, params: Params, base_vel_refs, lsc=aligator.LinearSolverChoice.LQ_SOLVER_SERIAL):
        print(Fore.GREEN + "[using SolverProxDDP]" + Fore.RESET)
        mu_init = 1e-11
        self.solver = aligator.SolverProxDDP(params.ocp.tol, mu_init)
        self.solver.force_initial_condition = True
        self.solver.linear_solver_choice = lsc
        self.solver.rollout_type = aligator.ROLLOUT_LINEAR
        super().__init__(params, base_vel_refs)

    def get_type_str():
        return "algtr-prox"
