from .wbmpc_wrapper_ros import ROSMPCWrapperClient
from .wbmpc_wrapper_multiprocess import MultiprocessMPCWrapper

import numpy as np

from .wb_mpc.ocp_abstract import OCPAbstract
from typing import Type

from .wbmpc_wrapper_abstract import MPCResult
from quadruped_reactive_walking import Params


class ROSMPCAsyncClient(MultiprocessMPCWrapper):
    """
    Wrapper to run both types of MPC (OQSP or Crocoddyl) asynchronously in a new process
    """

    def __init__(self, params: Params, base_vel_refs, solver_cls: Type[OCPAbstract]):
        self.ros_client = ROSMPCWrapperClient(params, base_vel_refs, solver_cls, synchronous=True)

        super().__init__(params, base_vel_refs, solver_cls)

    def _mpc_asynchronous(self):
        x0 = np.zeros_like(self.x0_shared)
        base_vel_ref = np.zeros_like(self.base_vel_ref_shared)
        while self.running.value:
            if not self.new_data.value:
                continue

            self.new_data.value = False

            with self.mutex:
                k, x0[:], base_vel_ref[:] = self._get_shared_data_in()

            self.ros_client.solve(k, x0, base_vel_ref)
            res: MPCResult = self.ros_client.get_latest_result()
            self._put_shared_data_out(res.gait, res.xs, res.us, res.K, res.num_iters, res.solving_duration)
            self.new_result.value = True

    def stop_parallel_loop(self):
        self.ros_client.stop_parallel_loop()
        super().stop_parallel_loop()
