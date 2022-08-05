import ctypes
from ctypes import Structure
from enum import Enum
from multiprocessing import Process, Value, Array
from time import time, sleep

import numpy as np

from .WB_MPC.CrocoddylOCP import OCP

import quadruped_reactive_walking as qrw


class Result:
    def __init__(self, pd):
        self.xs = list(np.zeros((pd.T + 1, pd.nx)))
        self.us = list(np.zeros((pd.T, pd.nu)))
        self.K = list(np.zeros([pd.T, pd.nu, pd.nx]))
        self.solving_duration = 0.0
        self.new_result = False


class MPC_Wrapper:
    """
    Wrapper to run both types of MPC (OQSP or Crocoddyl) with the possibility to run OSQP in
    a parallel process
    """

    def __init__(self, pd, params, footsteps, gait):
        self.params = params
        self.pd = pd

        self.footsteps_plan = footsteps
        self.initial_gait = gait

        self.multiprocessing = params.enable_multiprocessing

        if self.multiprocessing:
            self.new_data = Value("b", False)
            self.running = Value("b", True)
            self.in_k = Value("i", 0)
            self.in_x0 = Array("d", [0] * pd.nx)
            self.in_warm_start = Value("b", False)
            self.in_xs = Array("d", [0] * ((pd.T + 1) * pd.nx))
            self.in_us = Array("d", [0] * (pd.T * pd.nu))
            self.in_footstep = Array("d", [0] * 12)
            self.in_gait = Array("d", [0] * (pd.T * 4))
            self.out_xs = Array("d", [0] * ((pd.T + 1) * pd.nx))
            self.out_us = Array("d", [0] * (pd.T * pd.nu))
            self.out_k = Array("d", [0] * (pd.T * pd.nu * pd.nx))
            self.out_solving_time = Value("d", 0.0)
        else:
            self.ocp = OCP(pd, footsteps, gait)

        self.last_available_result = Result(pd)
        self.new_result = Value("b", False)

    def solve(self, k, x0, footstep, gait, xs=None, us=None):
        """
        Call either the asynchronous MPC or the synchronous MPC depending on the value
        of multiprocessing during the creation of the wrapper

        Args:
            k (int): Number of inv dynamics iterations since the start of the simulation
        """
        if self.multiprocessing:
            self.run_MPC_asynchronous(k, x0, footstep, gait, xs, us)
        else:
            self.run_MPC_synchronous(x0, footstep, gait, xs, us)

    def get_latest_result(self):
        """
        Return the desired contact forces that have been computed by the last iteration
        of the MPC.
        If a new result is available, return the new result.
        Otherwise return the old result again.
        """
        if self.new_result.value:
            if self.multiprocessing:
                (
                    self.last_available_result.xs,
                    self.last_available_result.us,
                    self.last_available_result.K,
                    self.last_available_result.solving_duration,
                ) = self.decompress_dataOut()

            self.last_available_result.new_result = True
            self.new_result.value = False
        else:
            self.last_available_result.new_result = False

        return self.last_available_result

    def run_MPC_synchronous(self, x0, footstep, gait, xs, us):
        """
        Run the MPC (synchronous version)
        """
        self.ocp.solve(x0, footstep, gait, xs, us)
        (
            self.last_available_result.xs,
            self.last_available_result.us,
            self.last_available_result.K,
            self.last_available_result.solving_duration,
        ) = self.ocp.get_results()
        self.new_result.value = True

    def run_MPC_asynchronous(self, k, x0, footstep, gait, xs, us):
        """
        Run the MPC (asynchronous version)
        """
        if k == 0:
            self.last_available_result.xs = [x0 for _ in range(self.pd.T + 1)]
            p = Process(target=self.MPC_asynchronous)
            p.start()
        self.add_new_data(k, x0, footstep, gait, xs, us)

    def MPC_asynchronous(self):
        """
        Parallel process with an infinite loop that run the asynchronous MPC
        """
        while self.running.value:
            if not self.new_data.value:
                continue

            self.new_data.value = False

            k, x0, footstep, gait, xs, us = self.decompress_dataIn()

            if k == 0:
                loop_ocp = OCP(self.pd, self.footsteps_plan, self.initial_gait)

            loop_ocp.solve(x0, footstep, gait, xs, us)
            xs, us, K, solving_time = loop_ocp.get_results()
            self.compress_dataOut(xs, us, K, solving_time)
            self.new_result.value = True

    def add_new_data(self, k, x0, footstep, gait, xs, us):
        """
        Compress data in a C-type structure that belongs to the shared memory to send
        data from the main control loop to the asynchronous MPC and notify the process
        that there is a new data
        """

        self.compress_dataIn(k, x0, footstep, gait, xs, us)
        self.new_data.value = True

    def compress_dataIn(self, k, x0, footstep, gait, xs, us):
        """
        Decompress data from a C-type structure that belongs to the shared memory to
        retrieve data from the main control loop in the asynchronous MPC
            dataIn (Array): shared C-type structure that contains the input data
        """
        with self.in_k.get_lock():
            self.in_k.value = k
        with self.in_x0.get_lock():
            np.frombuffer(self.in_x0.get_obj()).reshape(self.pd.nx)[:] = x0
        with self.in_footstep.get_lock():
            np.frombuffer(self.in_footstep.get_obj()).reshape((3, 4))[:, :] = footstep
        with self.in_gait.get_lock():
            np.frombuffer(self.in_gait.get_obj()).reshape((self.pd.T, 4))[:, :] = gait

        if xs is None or us is None:
            self.in_warm_start.value = False
            return

        with self.in_xs.get_lock():
            np.frombuffer(self.in_xs.get_obj()).reshape((self.pd.T + 1, self.pd.nx))[
                :, :
            ] = np.array(xs)
        with self.in_us.get_lock():
            np.frombuffer(self.in_us.get_obj()).reshape((self.pd.T, self.pd.nu))[
                :, :
            ] = np.array(us)

    def decompress_dataIn(self):
        """
        Decompress data from a C-type structure that belongs to the shared memory to
        retrieve data from the main control loop in the asynchronous MPC
        """
        with self.in_k.get_lock():
            k = self.in_k.value
        with self.in_x0.get_lock():
            x0 = np.frombuffer(self.in_x0.get_obj()).reshape(self.pd.nx)
        with self.in_footstep.get_lock():
            footstep = np.frombuffer(self.in_footstep.get_obj()).reshape((3, 4))
        with self.in_gait.get_lock():
            gait = np.frombuffer(self.in_gait.get_obj()).reshape((self.pd.T, 4))

        if not self.in_warm_start.value:
            return k, x0, footstep, gait, None, None

        with self.in_xs.get_lock():
            xs = list(
                np.frombuffer(self.in_xs.get_obj()).reshape((self.pd.T + 1, self.pd.nx))
            )
        with self.in_us.get_lock():
            us = list(
                np.frombuffer(self.in_us.get_obj()).reshape((self.pd.T, self.pd.nu))
            )

        return k, x0, footstep, gait, xs, us

    def compress_dataOut(self, xs, us, K, solving_time):
        """
        Compress data to a C-type structure that belongs to the shared memory to
        retrieve data in the main control loop from the asynchronous MPC
        """
        with self.out_xs.get_lock():
            np.frombuffer(self.out_xs.get_obj()).reshape((self.pd.T + 1, self.pd.nx))[
                :, :
            ] = np.array(xs)
        with self.out_us.get_lock():
            np.frombuffer(self.out_us.get_obj()).reshape((self.pd.T, self.pd.nu))[
                :, :
            ] = np.array(us)
        with self.out_k.get_lock():
            np.frombuffer(self.out_k.get_obj()).reshape(
                [self.pd.T, self.pd.nu, self.pd.nx]
            )[:, :, :] = np.array(K)
        self.out_solving_time.value = solving_time

    def decompress_dataOut(self):
        """
        Return the result of the asynchronous MPC (desired contact forces) that is
        stored in the shared memory
        """
        xs = list(
            np.frombuffer(self.out_xs.get_obj()).reshape((self.pd.T + 1, self.pd.nx))
        )
        us = list(np.frombuffer(self.out_us.get_obj()).reshape((self.pd.T, self.pd.nu)))
        K = list(
            np.frombuffer(self.out_k.get_obj()).reshape(
                [self.pd.T, self.pd.nu, self.pd.nx]
            )
        )
        solving_time = self.out_solving_time.value

        return xs, us, K, solving_time

    def stop_parallel_loop(self):
        """
        Stop the infinite loop in the parallel process to properly close the simulation
        """

        self.running.value = False
