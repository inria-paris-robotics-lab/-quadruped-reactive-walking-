import argparse
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
    def __init__(self, service_name):
        rospy.wait_for_service(service_name, timeout=10)
        self.solve_solver_srv = rospy.ServiceProxy(service_name, MPCSolve, persistent=True)

    def fake_request_data():
        k = 0
        x0 = np.random.randn(37)
        base_vel_ref = np.array([0.5, 0, 0, 0, 0, 0])
        req = MPCSolveRequest(
            k=k,
            x0=numpy_to_multiarray_float64(x0),
            base_vel_ref=numpy_to_multiarray_float64(base_vel_ref),
        )
        return req

    def solve(self):
        return self.solve_solver_srv(TestClient.fake_request_data())


class TestServer:
    def __init__(self, service_name):
        self._solve_service = rospy.Service(service_name, MPCSolve, self._trigger_solve)
        from quadruped_reactive_walking import MPCResult, Params

        p = Params.create_from_file()
        self.res = MPCResult(p.N_gait, 37, 12, 36, p.window_size)

    def _trigger_solve(self, msg):
        return MPCSolveResponse(
            run_success=True,
            gait=numpy_to_multiarray_float64(self.res.gait),
            xs=listof_numpy_to_multiarray_float64(self.res.xs.tolist()),
            us=listof_numpy_to_multiarray_float64(self.res.us.tolist()),
            K=listof_numpy_to_multiarray_float64(self.res.K.tolist()),
        )


if __name__ == "__main__":
    # Parsing arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--client", help="Only run client side code", action="store_true")
    parser.add_argument("--server", help="Only run server side code", action="store_true")
    parser.add_argument("--service", help="Name of the service to test", default="qrw_wbmpc/test")
    args = parser.parse_args()

    start_client = args.client or not args.server
    start_server = args.server or not args.client

    # Starintg ros node
    rospy.init_node("qrw_wbmpc" + ("_client" if start_client else "") + ("_server" if start_server else ""))

    # Running client and/or server
    if start_server:
        server = TestServer(args.service)

    if start_client:
        client = TestClient(args.service)
        client.solve()

    # Simple test that everything went fine
    assert not rospy.is_shutdown()

    # If the client is not started, let the server running
    if not start_client:
        print(f"Server running ({args.service})...")
        rospy.spin()
