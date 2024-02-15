import time

import ndcurves
import numpy as np
import pinocchio as pin

import quadruped_reactive_walking as qrw

from crocoddyl import StateMultibody
from . import wb_mpc
from .wb_mpc.target import Target, make_footsteps_and_refs
from .wb_mpc.task_spec import TaskSpec
from .wbmpc_wrapper_abstract import MPCResult
from .tools.utils import quaternionToRPY, make_initial_footstep
from typing import Type


class ControllerResult:
    """
    Object to store the result of the control loop
    It contains what is sent to the robot (gains, desired positions and velocities,
    feedforward torques)
    """

    def __init__(self, params):
        self.P = np.array(params.Kp_main.tolist() * 4)
        self.D = np.array(params.Kd_main.tolist() * 4)
        self.FF_weight = params.Kff_main * np.ones(12)
        self.q_des = np.zeros(12)
        self.v_des = np.zeros(12)
        self.tau_ff = np.zeros(12)


class DummyDevice:
    def __init__(self, h):
        self.imu = self.IMU()
        self.joints = self.Joints()
        self.base_position = np.zeros(3)
        self.base_position[2] = 0.1944
        self.b_base_velocity = np.zeros(3)
        self.baseState = ((0.0, 0.0, h), (0.0, 0.0, 0.0, 1.0))
        self.baseVel = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    class IMU:
        def __init__(self):
            self.linear_acceleration = np.zeros(3)
            self.gyroscope = np.zeros(3)
            self.attitude_euler = np.zeros(3)
            self.attitude_quaternion = np.zeros(4)

    class Joints:
        def __init__(self):
            self.positions = np.zeros(12)
            self.velocities = np.zeros(12)


def get_x_arr_no_base(xs):
    arr = np.stack(xs[:3]).T
    return np.concatenate([arr[7:19], arr[25:]])


class Controller:
    t_mpc = 0.0
    q_security = np.array([1.2, 2.1, 3.14] * 4)

    def __init__(self, params: qrw.Params, q_init, solver_cls: Type[wb_mpc.OCPAbstract]):
        """
        Function that computes the reference control (tau, q_des, v_des and gains)

        Args:
            params (Params object): store parameters
            q_init (array): initial position of actuators
        """

        self.params = params
        self.task = TaskSpec(params)
        self.rdata = self.task.model.createData()
        self.state = StateMultibody(self.task.model)

        self.k = 0
        self.error = False
        self.initialized = False

        if params.predefined_vel:
            self.joystick = qrw.AnimatorBase(params)
        else:
            self.joystick = qrw.Joystick(params)

        self.estimator = qrw.Estimator()
        self.estimator.initialize(params)
        self.q = np.zeros(18)
        self.mpc_result: MPCResult = None

        self.result = ControllerResult(params)
        self.result.q_des = self.task.q0[7:].copy()
        self.result.v_des = self.task.v0[6:].copy()

        self.target = Target(params)
        self.footsteps, self.base_refs = make_footsteps_and_refs(self.params, self.target)

        self.default_footstep = make_initial_footstep(params.q_init)
        self.target_base = pin.Motion.Zero()
        self.target_footstep = np.zeros((3, 4))

        self.mpc = self._create_mpc(solver_cls=solver_cls)
        assert self.mpc is not None, "Error while instanciating MPCWrapper"

        self.gait = np.array([[1, 1, 1, 1]] * (params.N_gait + 1))
        self.mpc_solved = False
        self.k_result = 0
        self.k_solve = 0
        if self.params.interpolate_mpc:
            x_arr_nobase = get_x_arr_no_base([self.task.x0, self.task.x0, self.task.x0])
            q_arr_nobase = x_arr_nobase[:12]
            self.interpolator_ = ndcurves.exact_cubic(
                q_arr_nobase,
                np.array([0.0, params.dt_mpc, 2 * params.dt_mpc]),
            )
        # TODO: reload warm starts here

        self.filter_q = qrw.LowPassFilter(params)
        self.filter_v = qrw.LowPassFilter(params)

        device = DummyDevice(params.h_ref)
        device.joints.positions = q_init
        self.compute(device)

    def _create_mpc(self, solver_cls):
        if self.params.mpc_in_rosnode:
            if self.params.asynchronous_mpc:
                from .wbmpc_wrapper_ros_mp import ROSMPCAsyncClient

                return ROSMPCAsyncClient(self.params, self.footsteps, self.base_refs, solver_cls)
            else:
                from .wbmpc_wrapper_ros import ROSMPCWrapperClient

                return ROSMPCWrapperClient(self.params, self.footsteps, self.base_refs, solver_cls, True)
        else:
            if self.params.asynchronous_mpc:
                from .wbmpc_wrapper_multiprocess import (
                    MultiprocessMPCWrapper as MPCWrapper,
                )
            else:
                from .wbmpc_wrapper_sync import SyncMPCWrapper as MPCWrapper
            return MPCWrapper(
                self.params,
                self.footsteps,
                self.base_refs,
                solver_cls=solver_cls,
            )

    def warmup(self):
        pass

    def compute(self, device, qc=None):
        """
        Run one iteration of the main control loop

        Args:
            device (object): Interface with the masterboard or the simulation
        """
        t_start = time.time()

        self.joystick.update_v_ref(self.k, False)

        oRh, hRb, oTh = self.run_estimator(device)

        t_measures = time.time()
        self.t_measures = t_measures - t_start

        if self.params.movement == "base_circle" or self.params.movement == "walk":
            self.target_base.np[:] = self.v_ref
            self.target_footstep[:] = 0.0
        else:
            self.target_base.np[:] = 0.0
            self.target_footstep[:] = self.target.compute(self.k + self.params.N_gait * self.params.mpc_wbc_ratio)

        if self.k % self.params.mpc_wbc_ratio == 0:
            if self.mpc_solved:
                self.k_solve = self.k
                self.mpc_solved = False

            if self.params.closed_loop or not self.initialized:
                x = self.x_estim
            else:
                x = self.mpc_result.xs[1]

            try:
                self.t_mpc_start = time.time()
                self.mpc.solve(self.k, x, self.target_footstep.copy(), self.target_base.copy())
            except ValueError:
                import traceback

                self.error = True
                traceback.print_exc()

        t_mpc = time.time()

        if not self.error:
            self.mpc_result: MPCResult = self.mpc.get_latest_result()
            self.gait[:, :] = self.mpc_result.gait
            xs = self.mpc_result.xs
            if self.mpc_result.new_result:
                self.mpc_solved = True
                self.k_new = self.k
                self.t_mpc = t_mpc - self.t_mpc_start

            if not self.initialized and self.params.save_guess:
                self.save_guess()

            # Compute feedforward torque
            self.result.tau_ff[:] = self.compute_torque()

            self.result.q_des[:], self.result.v_des[:] = self.interpolate_solution(xs)

        t_send = time.time()
        self.t_send = t_send - t_mpc

        self.clamp_result(device)
        self.security_check()

        if self.error:
            self.set_null_control()

        self.t_loop = time.time() - t_start
        self.k += 1
        self.initialized = True

        return self.error

    def interpolate_solution(self, xs):
        t_wp = np.linspace(0.0, 2 * self.params.dt_mpc, 3)
        if self.params.interpolate_mpc:
            # Use interpolation
            if self.mpc_result.new_result:
                if self.params.interpolation_type == qrw.INTERP_CUBIC:
                    x_arr_nobase = get_x_arr_no_base(xs)
                    q_arr_nobase = x_arr_nobase[:12]
                    self.interpolator_ = ndcurves.exact_cubic(q_arr_nobase, t_wp)
            t = (self.k - self.k_solve + 1) * self.params.dt_wbc
            q = self.interpolator_(t)
            v = self.interpolator_.derivate(t, 1)
        else:
            # use integration
            q, v = self.integrate_x()
        return q, v

    def security_check(self):
        """
        Check if the command is fine and set the command to zero in case of error
        """

        if not self.error:
            if (np.abs(self.q_estimate[7:]) > self.q_security).any():
                print("-- POSITION LIMIT ERROR --")
                print(self.q_estimate[7:])
                print(np.abs(self.q_estimate[7:]) > self.q_security)
                self.error = True
            elif (np.abs(self.v_estimate[6:]) > 100.0).any():
                print("-- VELOCITY TOO HIGH ERROR --")
                print(self.v_estimate[6:])
                print(np.abs(self.v_estimate[6:]) > 100.0)
                self.error = True
            elif (np.abs(self.result.FF_weight) > 5.0).any():
                print("-- FEEDFORWARD TORQUES TOO HIGH ERROR --")
                print(self.result.FF_weight)
                print(np.abs(self.result.FF_weight) > 5.0)
                self.error = True

    def clamp(self, num, min_value=None, max_value=None):
        clamped = False
        if min_value is not None and num <= min_value:
            num = min_value
            clamped = True
        if max_value is not None and num >= max_value:
            num = max_value
            clamped = True
        return clamped

    def clamp_result(self, device, set_error=False):
        """
        Clamp the result
        """
        hip_max = 120.0 * np.pi / 180.0
        knee_min = 5.0 * np.pi / 180.0
        hip_ids = []
        knee_ids = []
        for i in range(4):
            if self.clamp(self.result.q_des[3 * i + 1], -hip_max, hip_max):
                hip_ids.append(i)
                self.error = set_error
            if self.task.q0[7 + 3 * i + 2] >= 0.0 and self.clamp(self.result.q_des[3 * i + 2], knee_min):
                knee_ids.append(i)
                self.error = set_error
            elif self.task.q0[7 + 3 * i + 2] <= 0.0 and self.clamp(self.result.q_des[3 * i + 2], max_value=-knee_min):
                knee_ids.append(i)
                self.error = set_error
        if len(hip_ids) > 0:
            print("Clamping hips {}".format(hip_ids))
        if len(knee_ids) > 0:
            print("Clamping knees {}".format(knee_ids))

        clamped_tau = []
        clamped_pos = []
        clamped_vel = []
        for i in range(12):
            if self.clamp(
                self.result.q_des[i],
                device.joints.positions[i] - 4.0,
                device.joints.positions[i] + 4.0,
            ):
                clamped_pos.append(i)
                self.error = set_error

            if self.clamp(
                self.result.v_des[i],
                device.joints.velocities[i] - 100.0,
                device.joints.velocities[i] + 100.0,
            ):
                clamped_vel.append(i)
                self.error = set_error

            if self.clamp(self.result.tau_ff[i], -3.2, 3.2):
                clamped_tau.append(i)
                self.error = set_error
        if len(clamped_tau) > 0:
            print("Clamping torque of motors {}".format(clamped_tau))
        if len(clamped_pos) > 0:
            print("Clamping position of motors {}".format(clamped_pos))
        if len(clamped_vel) > 0:
            print("Clamping velocity of motors {}".format(clamped_vel))

    def set_null_control(self):
        """
        Send default null values to the robot
        """
        self.result.FF_weight[:] = 0.0
        self.result.q_des[:] = 0.0
        self.result.v_des[:] = 0.0
        self.result.tau_ff[:] = 0.0

    def save_guess(self, filename="/tmp/init_guess.npy"):
        """
        Save the result of the MPC in a file called /tmp/init_guess.npy
        """
        np.save(
            open(filename, "wb"),
            {"xs": self.mpc_result.xs, "us": self.mpc_result.us},
        )
        print("Initial guess saved")

    def run_estimator(self, device, q_perfect=np.zeros(6), b_baseVel_perfect=np.zeros(3)):
        """
        Call the estimator and retrieve the reference and estimated quantities.
        Run a filter on q, h_v and v_ref.

        @param device device structure holding simulation data
        @param q_perfect 6D perfect position of the base in world frame
        @param v_baseVel_perfect 3D perfect linear velocity of the base in base frame
        """

        if self.k < 2:
            self.estimator.initialize_IMU_Yaw()

        self.estimator.run(
            self.gait,
            self.default_footstep,
            device.imu.linear_acceleration,
            device.imu.gyroscope,
            device.imu.attitude_euler,
            device.joints.positions,
            device.joints.velocities,
            q_perfect,
            b_baseVel_perfect,
        )

        self.estimator.update_reference_state(self.joystick.v_ref)

        oRh = self.estimator.get_oRh()
        hRb = self.estimator.get_hRb()
        oTh = self.estimator.get_oTh().reshape((3, 1))

        self.v_ref = self.estimator.get_base_vel_ref()
        self.h_v = self.estimator.get_h_v()
        self.h_v_windowed = self.estimator.get_h_v_filtered()
        self.v_windowed = self.estimator.get_v_filtered()

        self.q_estimate = self.estimator.get_q_estimate()
        self.v_estimate = self.estimator.get_v_estimate()

        # bp_m = np.array([e for tup in device.baseState for e in tup])
        # bv_m = np.array([e for tup in device.baseVel for e in tup])
        self.q[:3] = self.q_estimate[:3]
        self.q[3:6] = quaternionToRPY(self.q_estimate[3:7])
        self.q[6:] = self.q_estimate[7:]

        self.v = self.estimator.get_v_reference()

        self.base_position_filtered = self.filter_q.filter(self.q[:6], True)

        self.q_filtered = self.q_estimate.copy()
        self.q_filtered[:3] = self.base_position_filtered[:3]
        self.q_filtered[3:7] = pin.Quaternion(pin.rpy.rpyToMatrix(self.base_position_filtered[3:])).coeffs()
        self.v_filtered = self.v_estimate.copy()
        # self.v_filtered[:6] = np.zeros(6)
        # self.v_filtered[:6] = self.filter_v.filter(self.v_windowed, False)
        self.v_filtered[:6] = self.filter_v.filter(self.v_estimate[:6], False)

        self.x_estim = np.concatenate([self.q_filtered, self.v_filtered])
        return oRh, hRb, oTh

    def compute_torque(self):
        """
        Compute the feedforward torque using ricatti gains
        """
        x_diff = self.state.diff(self.x_estim, self.mpc_result.xs[0])
        tau = self.mpc_result.us[0] + np.dot(self.mpc_result.K[0], x_diff)
        return tau

    def integrate_x(self):
        """
        Integrate the position and velocity using the acceleration computed from the
        feedforward torque
        """
        q0 = self.q_estimate.copy()
        v0 = self.v_estimate.copy()
        tau = np.concatenate([np.zeros(6), self.result.tau_ff])

        a = pin.aba(self.task.model, self.rdata, q0, v0, tau)

        v = v0 + a * self.params.dt_wbc
        q = pin.integrate(self.task.model, q0, v * self.params.dt_wbc)

        return q[7:], v[6:]
