from quadruped_reactive_walking import Params

from .wb_mpc import get_ocp_from_str
from .wb_mpc.ocp_abstract import OCPAbstract
from .wb_mpc.task_spec import TaskSpec

from .wbmpc_wrapper_abstract import MPCWrapperAbstract, MPCResult

from typing import Type
from threading import Lock

import rospy
from quadruped_reactive_walking.srv import (
    MPCInit,
    MPCInitResponse,
    MPCSolve,
    MPCSolveResponse,
    MPCStop,
    MPCStopResponse,
)
from .tools.ros_tools import (
    numpy_to_multiarray_float64,
    multiarray_to_numpy_float64,
    listof_numpy_to_multiarray_float64,
    multiarray_to_listof_numpy_float64,
    AsyncServiceProxy,
)


class ROSMPCWrapperClient(MPCWrapperAbstract):
    """
    Wrapper to run both types of MPC (OQSP or Crocoddyl) on a seperate node/machine using ROS as communication interface.
    """

    def __init__(
        self,
        params,
        footsteps,
        base_refs,
        solver_cls: Type[OCPAbstract],
        synchronous=False,
    ):
        self.synchronous = synchronous

        self._result_lock = Lock()
        self.new_result: bool = False
        self.pd = TaskSpec(params)
        self.last_available_result: MPCResult = MPCResult(
            params.N_gait, self.pd.nx, self.pd.nu, self.pd.ndx, self.WINDOW_SIZE
        )

        base_refs_multiarray = listof_numpy_to_multiarray_float64(base_refs)
        footsteps_multiarray = listof_numpy_to_multiarray_float64(footsteps)

        init_solver_srv = rospy.ServiceProxy("qrw_wbmpc/init", MPCInit)
        res = init_solver_srv(
            solver_type=solver_cls.get_type_str(),
            params=params.raw_str,
            footsteps=footsteps_multiarray,
            base_refs=base_refs_multiarray,
        )
        assert res.success, "Error while initializing mpc on server"

        self.solve_solver_srv = None
        if self.synchronous:
            self.solve_solver_srv = rospy.ServiceProxy(
                "qrw_wbmpc/solve", MPCSolve, persistent=True
            )
        else:
            self.solve_solver_srv = AsyncServiceProxy(
                "qrw_wbmpc/solve", MPCSolve, callback=self._result_cb, persistent=True
            )

    def solve(self, k, x0, footstep, base_ref, xs=None, us=None):
        res = self.solve_solver_srv(
            k=k,
            x0=numpy_to_multiarray_float64(x0),
            footstep=numpy_to_multiarray_float64(footstep),
            base_ref=numpy_to_multiarray_float64(base_ref),
            xs=listof_numpy_to_multiarray_float64(xs if xs is not None else []),
            us=listof_numpy_to_multiarray_float64(us if us is not None else []),
        )
        if self.synchronous:
            self._parse_result(res)

    def _result_cb(self, fut):
        msg = fut.result()
        self._parse_result(msg)

    def _parse_result(self, msg):
        assert msg.run_success, "Error while runnning solver on server"
        with self._result_lock:
            self.new_result = True
            self.last_available_result.gait = multiarray_to_numpy_float64(msg.gait)
            self.last_available_result.xs = multiarray_to_listof_numpy_float64(msg.xs)
            self.last_available_result.us = multiarray_to_listof_numpy_float64(msg.us)
            self.last_available_result.K = multiarray_to_listof_numpy_float64(msg.K)
            self.last_available_result.solving_duration = msg.solving_duration
            self.last_available_result.num_iters = msg.num_iters

    def get_latest_result(self):
        """
        If a new result is available, return the new result.
        Otherwise return the old result again.
        """
        with self._result_lock:
            self.last_available_result.new_result = self.new_result
            self.new_result = False

        return self.last_available_result

    def stop_parallel_loop(self):
        stop_solver_srv = rospy.ServiceProxy("qrw_wbmpc/stop", MPCStop)
        res = stop_solver_srv()
        assert (
            res.success
        ), "Unable to stop the MPC server. (Most probably stopped already)"


class ROSMPCWrapperServer:
    WINDOW_SIZE = 2

    def __init__(self):
        self.is_init = False
        self._init_service = rospy.Service(
            "qrw_wbmpc/init", MPCInit, self._trigger_init
        )
        self._solve_service = rospy.Service(
            "qrw_wbmpc/solve", MPCSolve, self._trigger_solve
        )
        self._stop_service = rospy.Service(
            "qrw_wbmpc/stop", MPCStop, self._trigger_stop
        )
        rospy.loginfo("Initializing MPC server.")

    def _trigger_init(self, msg):
        if self.is_init:
            rospy.logerr("MPC already initialized.")
            return MPCInitResponse(False)

        self.params = Params.create_from_str(msg.params)
        self.pd = TaskSpec(self.params)
        self.T = self.params.N_gait
        self.nu = self.pd.nu
        self.nx = self.pd.nx
        self.ndx = self.pd.ndx
        self.solver_cls = get_ocp_from_str(msg.solver_type)

        footsteps = multiarray_to_numpy_float64(msg.footsteps)
        base_refs = multiarray_to_numpy_float64(msg.base_refs)

        self.ocp = self.solver_cls(self.params, footsteps, base_refs)

        self.last_available_result: MPCResult = MPCResult(
            self.params.N_gait, self.pd.nx, self.pd.nu, self.pd.ndx, self.WINDOW_SIZE
        )

        rospy.loginfo("Initializing MPC.")
        self.is_init = True
        return MPCInitResponse(True)

    def _trigger_solve(self, msg):
        if not self.is_init:
            return MPCSolveResponse(run_success=False)

        self.ocp.make_ocp(
            msg.k,
            multiarray_to_numpy_float64(msg.x0),
            multiarray_to_numpy_float64(msg.footstep),
            multiarray_to_numpy_float64(msg.base_ref),
        )

        xs = multiarray_to_listof_numpy_float64(msg.xs)
        us = multiarray_to_listof_numpy_float64(msg.us)

        if len(xs) == 0:
            xs = None
        if len(us) == 0:
            us = None

        self.ocp.solve(msg.k)

        gait, xs, us, Ks, solving_duration = self.ocp.get_results(self.WINDOW_SIZE)

        return MPCSolveResponse(
            run_success=True,
            gait=numpy_to_multiarray_float64(gait),
            xs=listof_numpy_to_multiarray_float64(xs),
            us=listof_numpy_to_multiarray_float64(us),
            K=listof_numpy_to_multiarray_float64(Ks),
            solving_duration=solving_duration,
        )

    def _trigger_stop(self, msg):
        if not self.is_init:
            rospy.logwarn("[MPCStop] MPC was not initialized.")
            return MPCStopResponse(False)
        self.is_init = False
        rospy.loginfo("Shutting down MPC.")
        return MPCStopResponse(True)


if __name__ == "__main__":
    rospy.init_node("qrw_wbmpc")
    server = ROSMPCWrapperServer()
    rospy.spin()
