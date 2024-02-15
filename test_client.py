import rospy
import numpy as np
from quadruped_reactive_walking.srv import (
    MPCSolve,
    MPCSolveRequest,
    MPCSolveResponse,
)
from quadruped_reactive_walking.tools.ros_tools import (
    numpy_to_multiarray_float64,
    listof_numpy_to_multiarray_float64,
)


class TestClient:
    """
    Wrapper to run both types of MPC (OQSP or Crocoddyl) on a seperate node/machine using ROS as communication interface.
    """

    def __init__(
        self,
    ):
        self.solve_solver_srv = rospy.ServiceProxy("qrw_wbmpc/test", MPCSolve, persistent=True)

    def solve(self):
        return self.solve_solver_srv(MPCSolveRequest())


class TestServer:
    def __init__(self):
        self._solve_service = rospy.Service("qrw_wbmpc/test", MPCSolve, self._trigger_solve)
        from quadruped_reactive_walking import MPCResult, Params

        p = Params.create_from_file()
        self.res = MPCResult(p.N_gait, 37, 12, 36, p.window_size)

    def _trigger_solve(self, msg):
        return fake_out_data(self.res)
        # return MPCSolveResponse(run_success=True)


def fake_in_data():
    k = 0
    x0 = np.random.randn(37)
    footstep = np.random.randn(3, 4)
    base_ref = np.array([0.5, 0, 0, 0, 0, 0])
    req = MPCSolveRequest(
        k=k,
        x0=numpy_to_multiarray_float64(x0),
        footstep=numpy_to_multiarray_float64(footstep),
        base_ref=numpy_to_multiarray_float64(base_ref),
    )
    return req


def fake_out_data(res):
    return MPCSolveResponse(
        run_success=True,
        gait=numpy_to_multiarray_float64(res.gait),
        xs=listof_numpy_to_multiarray_float64(res.xs.tolist()),
        us=listof_numpy_to_multiarray_float64(res.us.tolist()),
        K=listof_numpy_to_multiarray_float64(res.K.tolist()),
    )


if __name__ == "__main__":
    rospy.init_node("qrw_wbmpc")
    server = TestServer()
    rospy.spin()
