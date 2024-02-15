try:
    from multiprocess import Process, Value, Lock
    from multiprocess.managers import SharedMemoryManager
except ImportError:
    from multiprocessing import Process, Value, Lock
    from multiprocessing.managers import SharedMemoryManager


import numpy as np
import pinocchio as pin

from .wb_mpc.ocp_abstract import OCPAbstract
from .tools.utils import create_shared_ndarray

from typing import Type

from .wbmpc_wrapper_abstract import MPCWrapperAbstract
from quadruped_reactive_walking import Params, MPCResult


class MultiprocessMPCWrapper(MPCWrapperAbstract):
    """
    Wrapper to run both types of MPC (OQSP or Crocoddyl) asynchronously in a new process
    """

    def __init__(self, params: Params, footsteps, base_refs, solver_cls: Type[OCPAbstract]):
        super().__init__(params)
        self.solver_cls = solver_cls

        self.footsteps_plan = footsteps
        self.base_refs = base_refs

        self.last_available_result: MPCResult = MPCResult(self.N_gait, self.nx, self.nu, self.ndx, self.WINDOW_SIZE)

        # Shared memory used for multiprocessing
        self.smm = SharedMemoryManager()
        self.smm.start()

        self.new_data = Value("b", False)
        self.running = Value("b", True)
        self.in_k = Value("i", 0, lock=False)
        self.in_warm_start = Value("b", False, lock=False)
        self.out_num_iters = Value("i", 0, lock=False)
        self.out_solving_time = Value("d", 0.0, lock=False)
        self.new_result = Value("b", False)

        self._shms = set()
        self.mutex = Lock()

        self.x0_shared = self.create_shared_ndarray(self.nx)
        self.gait_shared = self.create_shared_ndarray((self.N_gait + 1, 4), np.int32)
        self.xs_shared = self.create_shared_ndarray((self.WINDOW_SIZE + 1, self.nx))
        self.us_shared = self.create_shared_ndarray((self.WINDOW_SIZE, self.nu))
        self.Ks_shared = self.create_shared_ndarray((self.WINDOW_SIZE, self.nu, self.ndx))
        self.footstep_shared = self.create_shared_ndarray((3, 4))
        self.base_ref_shared = self.create_shared_ndarray(6)

        self.p = Process(target=self._mpc_asynchronous)
        self.p.start()

    def create_shared_ndarray(self, shape, dtype=np.float64):
        """Use current smm to create a shared array."""
        dtype = np.dtype(dtype)
        itemsize = dtype.itemsize
        nbytes = np.prod(shape) * itemsize
        shm = self.smm.SharedMemory(nbytes)
        self._shms.add(shm)
        return create_shared_ndarray(shape, dtype, shm)

    def solve(self, k, x0, footstep, base_vel_ref: pin.Motion):
        self._put_shared_data_in(k, x0, footstep, base_vel_ref.np)
        self.new_data.value = True

    def get_latest_result(self):
        """
        Return the desired contact forces that have been computed by the last iteration
        of the MPC.
        If a new result is available, return the new result.
        Otherwise return the old result again.
        """
        if self.new_result.value:
            (
                self.last_available_result.gait[:, :],
                self.last_available_result.xs,
                self.last_available_result.us,
                self.last_available_result.K,
                self.last_available_result.solving_duration,
                self.last_available_result.num_iters,
            ) = self._get_shared_data_out()

            self.last_available_result.new_result = True
            self.new_result.value = False
        else:
            self.last_available_result.new_result = False

        return self.last_available_result

    def _mpc_asynchronous(self):
        """
        Parallel process with an infinite loop that run the asynchronous MPC
        """
        # Thread-local data
        x0 = np.zeros_like(self.x0_shared)
        footstep = np.zeros_like(self.footstep_shared)
        base_ref = np.zeros_like(self.base_ref_shared)
        while self.running.value:
            if not self.new_data.value:
                continue

            self.new_data.value = False

            with self.mutex:
                k, x0[:], footstep[:], base_ref[:] = self._get_shared_data_in()

            if k == 0:
                loop_ocp = self.solver_cls(self.params, self.footsteps_plan, self.base_refs)

            loop_ocp.push_node(k, x0, footstep, base_ref)
            loop_ocp.solve(k)
            gait, xs, us, K, solving_time = loop_ocp.get_results(self.WINDOW_SIZE)
            self._put_shared_data_out(gait, xs, us, K, loop_ocp.num_iters, solving_time)
            self.new_result.value = True

    def _put_shared_data_in(self, k, x0, footstep, base_ref):
        """
        Put data in shared memory (input to the asynchronous MPC).
        """
        with self.mutex:
            self.in_k.value = k
            self.x0_shared[:] = x0
            self.footstep_shared[:] = footstep
            self.base_ref_shared[:] = base_ref

    def _get_shared_data_in(self):
        """
        Retrieve the input data for the MPC from the shared memory buffers.
        """
        k = self.in_k.value
        x0 = self.x0_shared
        footstep = self.footstep_shared
        base_ref = self.base_ref_shared
        return k, x0, footstep, base_ref

    def _put_shared_data_out(self, gait, xs, us, K, num_iters, solving_time):
        """Put data in shared memory (output of the asynchronous MPC to be retrieved)."""

        with self.mutex:
            self.gait_shared[:] = gait
            self.xs_shared[:] = np.stack(xs)
            self.us_shared[:] = np.stack(us)
            self.Ks_shared[:] = np.stack(K)
            self.out_num_iters = num_iters
            self.out_solving_time.value = solving_time

    def _get_shared_data_out(self):
        """Retrieve the MPC output data from the shared memory buffers."""
        gait = self.gait_shared
        xs = list(self.xs_shared)
        us = list(self.us_shared)
        K = list(self.Ks_shared)
        num_iters = self.out_num_iters.value
        solving_time = self.out_solving_time.value

        return gait, xs, us, K, solving_time, num_iters

    def stop_parallel_loop(self):
        """
        Stop the infinite loop in the parallel process to properly close the simulation
        """

        self.running.value = False
        self.p.join()
        self.smm.shutdown()
