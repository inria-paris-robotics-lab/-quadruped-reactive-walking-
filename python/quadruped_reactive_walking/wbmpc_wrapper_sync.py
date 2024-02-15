from .wb_mpc.ocp_abstract import OCPAbstract

from typing import Type

from .wbmpc_wrapper_abstract import MPCWrapperAbstract, MPCResult
from quadruped_reactive_walking import Params


class SyncMPCWrapper(MPCWrapperAbstract):
    """
    Wrapper to run both types of MPC (OQSP or Crocoddyl) in a synchronous manner in the main thread.
    """

    def __init__(self, params: Params, footsteps, base_refs, solver_cls: Type[OCPAbstract]):
        super().__init__(params)
        self.solver_cls = solver_cls

        self.ocp = solver_cls(params, footsteps, base_refs)

        self.last_available_result: MPCResult = MPCResult(
            self.params.N_gait, self.pd.nx, self.pd.nu, self.pd.ndx, self.WINDOW_SIZE
        )
        self.new_result = False

    def solve(self, k, x0, footstep, base_vel_ref):
        self.ocp.push_node(k, x0, footstep, base_vel_ref)
        self.ocp.solve(k)

        gait, xs, us, K, solving_duration = self.ocp.get_results(self.WINDOW_SIZE)
        self.last_available_result.gait = gait
        self.last_available_result.xs = xs
        self.last_available_result.us = us
        self.last_available_result.K = K
        self.last_available_result.solving_duration = solving_duration
        self.new_result = True

    def get_latest_result(self):
        """
        Return the desired contact forces that have been computed by the last iteration
        of the MPC.
        If a new result is available, return the new result.
        Otherwise return the old result again.
        """
        if self.new_result:
            self.last_available_result.new_result = True
            self.new_result = False
        else:
            self.last_available_result.new_result = False

        return self.last_available_result

    def stop_parallel_loop(self):
        pass  # Do nothing since it is single threaded
