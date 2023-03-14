from multiprocessing import Process, Value, Array

import numpy as np

from .wb_mpc.ocp_abstract import OCPAbstract
from .wb_mpc.problem_data import TaskSpec

from typing import Type

from .wbmpc_wrapper_abstract import MPCWrapperAbstract, Result


class MultiprocessMPCWrapper(MPCWrapperAbstract):
    """
    Wrapper to run both types of MPC (OQSP or Crocoddyl) asynchronously in a new process
    """

    def __init__(
        self, params, footsteps, base_refs, solver_cls: Type[OCPAbstract], **kwargs
    ):
        self.params = params
        self.pd = TaskSpec(params)
        self.T = params.T
        self.nu = self.pd.nu
        self.nx = self.pd.nx
        self.ndx = self.pd.ndx
        self.solver_cls = solver_cls
        self._solver_kwargs = kwargs

        self.footsteps_plan = footsteps
        self.base_refs = base_refs

        self.new_data = Value("b", False)
        self.running = Value("b", True)
        self.in_k = Value("i", 0)
        self.in_x0 = Array("d", [0] * self.nx)
        self.in_warm_start = Value("b", False)
        self.in_xs = Array("d", [0] * ((self.T + 1) * self.nx))
        self.in_us = Array("d", [0] * (self.T * self.nu))
        self.in_footstep = Array("d", [0] * 12)
        self.in_base_ref = Array("d", [0] * 6)
        self.out_gait = Array("d", [0] * ((self.T + 1) * 4))
        self.out_xs = Array("d", [0] * ((self.T + 1) * self.nx))
        self.out_us = Array("d", [0] * (self.T * self.nu))
        self.out_k = Array("d", [0] * (self.T * self.nu * self.ndx))
        self.out_num_iters = Value("i", 0)
        self.out_solving_time = Value("d", 0.0)

        self.last_available_result: Result = Result(params)
        self.new_result = Value("b", False)

    def solve(self, k, x0, footstep, base_ref, xs=None, us=None):
        if k == 0:
            if xs is not None:
                self.last_available_result.xs = xs
                self.last_available_result.us = us
            else:
                self.last_available_result.xs = [x0 for _ in range(self.T + 1)]
            p = Process(target=self._mpc_asynchronous)
            p.start()

        self.add_new_data(k, x0, footstep, base_ref, xs, us)

    def get_latest_result(self):
        """
        Return the desired contact forces that have been computed by the last iteration
        of the MPC.
        If a new result is available, return the new result.
        Otherwise return the old result again.
        """
        if self.new_result.value:
            (
                self.last_available_result.gait,
                self.last_available_result.xs,
                self.last_available_result.us,
                self.last_available_result.K,
                self.last_available_result.solving_duration,
                self.last_available_result.num_iters,
            ) = self._decompress_dataOut()

            self.last_available_result.new_result = True
            self.new_result.value = False
        else:
            self.last_available_result.new_result = False

        return self.last_available_result

    def _mpc_asynchronous(self):
        """
        Parallel process with an infinite loop that run the asynchronous MPC
        """
        while self.running.value:
            if not self.new_data.value:
                continue

            self.new_data.value = False

            k, x0, footstep, base_ref, xs, us = self._decompress_dataIn()

            if k == 0:
                loop_ocp = self.solver_cls(
                    self.params,
                    self.footsteps_plan,
                    self.base_refs,
                    **self._solver_kwargs
                )

            loop_ocp.make_ocp(k, x0, footstep, base_ref)
            loop_ocp.solve(k, xs, us)
            gait, xs, us, K, solving_time = loop_ocp.get_results()
            self._compress_dataOut(gait, xs, us, K, loop_ocp.num_iters, solving_time)
            self.new_result.value = True

    def add_new_data(self, k, x0, footstep, base_ref, xs, us):
        """
        Compress data in a C-type structure that belongs to the shared memory to send
        data from the main control loop to the asynchronous MPC and notify the process
        that there is a new data
        """

        self._compress_dataIn(k, x0, footstep, base_ref, xs, us)
        self.new_data.value = True

    def _compress_dataIn(self, k, x0, footstep, base_ref, xs, us):
        """
        Decompress data from a C-type structure that belongs to the shared memory to
        retrieve data from the main control loop in the asynchronous MPC
            dataIn (Array): shared C-type structure that contains the input data
        """
        with self.in_k.get_lock():
            self.in_k.value = k
        with self.in_x0.get_lock():
            np.frombuffer(self.in_x0.get_obj()).reshape(self.nx)[:] = x0
        with self.in_footstep.get_lock():
            np.frombuffer(self.in_footstep.get_obj()).reshape((3, 4))[:, :] = footstep
        with self.in_base_ref.get_lock():
            np.frombuffer(self.in_base_ref.get_obj())[:] = base_ref

        if xs is None or us is None:
            self.in_warm_start.value = False
            return
        self.in_warm_start.value = True

        with self.in_xs.get_lock():
            np.frombuffer(self.in_xs.get_obj()).reshape((self.T + 1, self.nx))[
                :, :
            ] = np.array(xs)
        with self.in_us.get_lock():
            np.frombuffer(self.in_us.get_obj()).reshape((self.T, self.nu))[
                :, :
            ] = np.array(us)

    def _decompress_dataIn(self):
        """
        Decompress data from a C-type structure that belongs to the shared memory to
        retrieve data from the main control loop in the asynchronous MPC
        """
        with self.in_k.get_lock():
            k = self.in_k.value
        with self.in_x0.get_lock():
            x0 = np.frombuffer(self.in_x0.get_obj()).reshape(self.nx)
        with self.in_footstep.get_lock():
            footstep = np.frombuffer(self.in_footstep.get_obj()).reshape((3, 4))
        with self.in_base_ref.get_lock():
            base_ref = np.frombuffer(self.in_base_ref.get_obj()).reshape(6)

        if not self.in_warm_start.value:
            return k, x0, footstep, base_ref, None, None

        with self.in_xs.get_lock():
            xs = list(
                np.frombuffer(self.in_xs.get_obj()).reshape((self.T + 1, self.nx))
            )
        with self.in_us.get_lock():
            us = list(np.frombuffer(self.in_us.get_obj()).reshape((self.T, self.nu)))

        return k, x0, footstep, base_ref, xs, us

    def _compress_dataOut(self, gait, xs, us, K, num_iters, solving_time):
        """
        Compress data to a C-type structure that belongs to the shared memory to
        retrieve data in the main control loop from the asynchronous MPC
        """
        with self.out_gait.get_lock():
            np.frombuffer(self.out_gait.get_obj()).reshape((self.T + 1, 4))[
                :, :
            ] = np.array(gait)

        with self.out_xs.get_lock():
            np.frombuffer(self.out_xs.get_obj()).reshape((self.T + 1, self.nx))[
                :, :
            ] = np.array(xs)
        with self.out_us.get_lock():
            np.frombuffer(self.out_us.get_obj()).reshape((self.T, self.nu))[
                :, :
            ] = np.array(us)
        with self.out_k.get_lock():
            np.frombuffer(self.out_k.get_obj()).reshape([self.T, self.nu, self.ndx])[
                :, :, :
            ] = np.array(K)
        self.out_num_iters = num_iters
        self.out_solving_time.value = solving_time

    def _decompress_dataOut(self):
        """
        Return the result of the asynchronous MPC (desired contact forces) that is
        stored in the shared memory
        """
        gait = np.frombuffer(self.out_gait.get_obj()).reshape((self.T + 1, 4))
        xs = list(np.frombuffer(self.out_xs.get_obj()).reshape((self.T + 1, self.nx)))
        us = list(np.frombuffer(self.out_us.get_obj()).reshape((self.T, self.nu)))
        K = list(
            np.frombuffer(self.out_k.get_obj()).reshape([self.T, self.nu, self.ndx])
        )
        num_iters = self.out_num_iters.value
        solving_time = self.out_solving_time.value

        return gait, xs, us, K, solving_time, num_iters

    def stop_parallel_loop(self):
        """
        Stop the infinite loop in the parallel process to properly close the simulation
        """

        self.running.value = False
