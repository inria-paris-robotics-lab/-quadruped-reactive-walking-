from datetime import datetime
from time import time
import numpy as np
import pathlib
from .kinematics_utils import get_translation_array
from ..controller import Controller
from ..wb_mpc.task_spec import TaskSpec


FIG_DPI = 100
TEMP_DIRNAME = pathlib.Path.home() / ".tmp"
DATE_STRFORMAT = "%Y_%m_%d_%H_%M_%S"


class LoggerControl:
    def __init__(
        self,
        params,
        log_size=60e3,
        loop_buffer=False,
        filename=None,
        solver_cls_name=None,
    ):
        if filename is not None:
            self.data = np.load(filename, allow_pickle=True)

        self.log_size = int(log_size)
        self.i = 0
        self.loop_buffer = loop_buffer
        self.params = params
        self.solver_cls = solver_cls_name

        size = self.log_size
        self.pd = TaskSpec(params)

        # IMU and actuators:
        self.q_mes = np.zeros([size, 12])
        self.v_mes = np.zeros([size, 12])
        self.torquesFromCurrentMeasurment = np.zeros([size, 12])
        self.baseOrientation = np.zeros([size, 3])
        self.baseOrientationQuat = np.zeros([size, 4])
        self.baseAngularVelocity = np.zeros([size, 3])
        self.baseLinearAcceleration = np.zeros([size, 3])
        self.baseAccelerometer = np.zeros([size, 3])
        self.current = np.zeros(size)
        self.voltage = np.zeros(size)
        self.energy = np.zeros(size)

        # Motion capture:
        self.mocapPosition = np.zeros([size, 3])
        self.mocapVelocity = np.zeros([size, 3])
        self.mocapAngularVelocity = np.zeros([size, 3])
        self.mocapOrientationMat9 = np.zeros([size, 3, 3])
        self.mocapOrientationQuat = np.zeros([size, 4])

        # Timestamps
        self.tstamps = np.zeros(size)

        # TODO: ADD WHAT YOU WANT TO LOG

        # Controller timings: MPC time, ...
        self.t_measures = np.full(size, np.nan)
        self.t_mpc = np.full(size, np.nan)  # solver time #measurement time
        self.t_send = np.full(size, np.nan)  #
        self.t_loop = np.full(size, np.nan)  # controller time loop
        self.t_whole = np.full(size, np.nan)  # controller time loop

        self.t_ocp_update = np.zeros(size)
        self.t_ocp_warm_start = np.full(size, np.nan)
        self.t_ocp_ddp = np.full(size, np.nan)
        self.t_ocp_solve = np.zeros(size)

        # MPC
        self.q_estimate_rpy = np.zeros([size, self.pd.nq - 1])
        self.q_estimate = np.zeros([size, self.pd.nq])
        self.v_estimate = np.zeros([size, self.pd.nv])
        self.q_filtered = np.zeros([size, self.pd.nq])
        self.v_filtered = np.zeros([size, self.pd.nv])
        MPC_WINDOW = params.window_size
        self.ocp_xs = np.zeros([size, MPC_WINDOW + 1, self.pd.nx])
        self.ocp_us = np.zeros([size, MPC_WINDOW, self.pd.nu])
        # "spot" feedback gain
        self.ocp_K = np.zeros([size, self.pd.nu, self.pd.ndx])
        self.ocp_num_iters = np.zeros([size], dtype=int)
        self.MPC_equivalent_Kp = np.zeros([size, self.pd.nu])
        self.MPC_equivalent_Kd = np.zeros([size, self.pd.nu])

        self.target = np.zeros([size, 3])
        self.target_base_linear = np.zeros([size, 3])
        self.target_base_angular = np.zeros([size, 3])

        # Whole body control
        self.wbc_P = np.zeros([size, 12])  # proportionnal gains of the PD+
        self.wbc_D = np.zeros([size, 12])  # derivative gains of the PD+
        self.wbc_q_des = np.zeros([size, 12])  # desired position of actuators
        self.wbc_v_des = np.zeros([size, 12])  # desired velocity of actuators
        self.wbc_FF = np.zeros([size, 12])  # gains for the feedforward torques
        self.wbc_tau_ff = np.zeros([size, 12])  # feedforward torques

    def sample(self, controller: Controller, device, qualisys=None):
        # Logging from the device (data coming from the robot)
        params: qrw.Params = controller.params
        if self.solver_cls is None:
            self.solver_cls = controller.mpc.ocp.__class__.get_type_str()
        self.q_mes[self.i] = device.joints.positions
        self.v_mes[self.i] = device.joints.velocities
        self.baseOrientation[self.i] = device.imu.attitude_euler
        self.baseOrientationQuat[self.i] = device.imu.attitude_quaternion
        self.baseAngularVelocity[self.i] = device.imu.gyroscope
        self.baseLinearAcceleration[self.i] = device.imu.linear_acceleration
        self.baseAccelerometer[self.i] = device.imu.accelerometer
        self.torquesFromCurrentMeasurment[self.i] = device.joints.measured_torques
        if hasattr(device, "powerboard"):
            self.current[self.i] = device.powerboard.current
            self.voltage[self.i] = device.powerboard.voltage
            self.energy[self.i] = device.powerboard.energy

        # Logging from qualisys (motion capture)
        if params.use_qualisys:
            assert qualisys is not None
            self.mocapPosition[self.i] = qualisys.getPosition()
            self.mocapVelocity[self.i] = qualisys.getVelocity()
            self.mocapAngularVelocity[self.i] = qualisys.getAngularVelocity()
            self.mocapOrientationMat9[self.i] = qualisys.getOrientationMat9()
            self.mocapOrientationQuat[self.i] = qualisys.getOrientationQuat()
        elif params.SIMULATION:  # Logging from PyBullet simulator through fake device
            self.mocapPosition[self.i] = device.baseState[0]
            self.mocapVelocity[self.i] = device.baseVel[0]
            self.mocapAngularVelocity[self.i] = device.baseVel[1]
            self.mocapOrientationMat9[self.i] = device.rot_oMb
            self.mocapOrientationQuat[self.i] = device.baseState[1]
        else:
            pass

        # Controller timings: MPC time, ...
        self.t_mpc[self.i] = controller.t_mpc
        self.t_send[self.i] = controller.t_send
        self.t_loop[self.i] = controller.t_loop
        self.t_measures[self.i] = controller.t_measures

        # Logging from model predictive control
        self.q_estimate_rpy[self.i] = np.array(controller.q)
        self.q_estimate[self.i] = np.array(controller.q_estimate)
        self.v_estimate[self.i] = np.array(controller.v_estimate)
        self.q_filtered[self.i] = np.array(controller.q_filtered)
        self.v_filtered[self.i] = np.array(controller.v_filtered)
        self.ocp_xs[self.i] = np.array(controller.mpc_result.xs)
        self.ocp_us[self.i] = np.array(controller.mpc_result.us)
        self.ocp_K[self.i] = controller.mpc_result.K[0]
        self.ocp_num_iters[self.i] = controller.mpc_result.num_iters
        self.MPC_equivalent_Kp[self.i] = controller.mpc_result.K[0].diagonal()
        self.MPC_equivalent_Kd[self.i] = controller.mpc_result.K[0].diagonal(3)

        self.t_measures[self.i] = controller.t_measures
        self.t_mpc[self.i] = controller.t_mpc
        self.t_send[self.i] = controller.t_send
        self.t_loop[self.i] = controller.t_loop

        self.t_ocp_ddp[self.i] = controller.mpc_result.solving_duration

        if self.i == 0:
            for i in range(self.params.N_gait * self.params.mpc_wbc_ratio):
                self.target[i] = controller.footsteps[i // self.params.mpc_wbc_ratio][:, 1]
                self.target_base_linear[i] = controller.base_refs[i // self.params.mpc_wbc_ratio].linear
                self.target_base_angular[i] = controller.base_refs[i // self.params.mpc_wbc_ratio].angular
        if self.i + self.params.N_gait * self.params.mpc_wbc_ratio < self.log_size:
            self.target[self.i + self.params.N_gait * self.params.mpc_wbc_ratio] = controller.target_footstep[:, 1]
            self.target_base_linear[self.i + self.params.N_gait * self.params.mpc_wbc_ratio] = controller.v_ref[:][:3]

            self.target_base_angular[self.i + self.params.N_gait * self.params.mpc_wbc_ratio] = controller.v_ref[:][3:]

        if not self.params.asynchronous_mpc and not self.params.mpc_in_rosnode:
            self.t_ocp_update[self.i] = controller.mpc.ocp.t_update
            self.t_ocp_warm_start[self.i] = controller.mpc.ocp.t_warm_start
            self.t_ocp_solve[self.i] = controller.mpc.ocp.t_solve

        # Logging from whole body control
        self.wbc_P[self.i] = controller.result.P
        self.wbc_D[self.i] = controller.result.D
        self.wbc_q_des[self.i] = controller.result.q_des
        self.wbc_v_des[self.i] = controller.result.v_des
        self.wbc_FF[self.i] = controller.result.FF_weight
        self.wbc_tau_ff[self.i] = controller.result.tau_ff

        # Logging timestamp
        self.tstamps[self.i] = time()

        self.i += 1

    def plot(self, save=False, filename=TEMP_DIRNAME):
        self.plot_states(save, filename)
        # self.plot_torques(save, filename)
        # self.plot_target(save, filename)
        # self.plot_riccati_gains(0, save, filename)
        self.plot_controller_times(save, filename)
        # if not self.params.asynchronous_mpc:
        #     self.plot_ocp_times()

    def plot_states(self, save=False, filename=TEMP_DIRNAME):
        import matplotlib.pyplot as plt
        from itertools import product

        legend = ["Hip", "Shoulder", "Knee"]
        figsize = (18, 6)
        fig: plt.Figure = plt.figure(figsize=figsize, dpi=FIG_DPI, constrained_layout=True)
        gridspec = fig.add_gridspec(1, 2)
        gs0 = gridspec[0].subgridspec(2, 2)
        gs1 = gridspec[1].subgridspec(2, 2)
        q_mes = np.array(self.q_mes)
        v_mes = np.array(self.v_mes)
        for ii, j in product(range(2), range(2)):
            plt.subplot(gs0[ii, j])
            i = ii * 2 + j
            plt.title("Joint position of " + str(i))
            for jj in range(3):
                plt.plot(q_mes[:, 3 * i + jj] * 180 / np.pi)
            plt.ylabel("Joint position [deg]")
            plt.xlabel("t[s]")
            plt.legend(legend)
        # plt.draw()
        # if save:
        #     plt.savefig(filename + "/joint_positions")

        for ii, j in product(range(2), range(2)):
            plt.subplot(gs1[ii, j])
            i = ii * 2 + j
            plt.title("Joint velocity of " + str(i))
            for jj in range(3):
                plt.plot(v_mes[:, (3 * i + jj)] * 180 / np.pi)
            plt.ylabel("Joint velocity [deg/s]")
            plt.xlabel("$t$ [s]")
            plt.legend(legend)
        plt.draw()
        # if save:
        #     plt.savefig(filename + "/joint_velocities")
        if save:
            plt.savefig(filename + "/joint_states")

    def plot_torques(self, save=False, filename=TEMP_DIRNAME):
        import matplotlib.pyplot as plt

        legend = ["Hip", "Shoulder", "Knee"]
        plt.figure(figsize=(12, 6), dpi=FIG_DPI)
        i = 0
        for i in range(4):
            plt.subplot(2, 2, i + 1)
            plt.title("Joint torques of " + str(i))
            [plt.plot(np.array(self.torquesFromCurrentMeasurment)[:, (3 * i + jj)]) for jj in range(3)]
            plt.ylabel("Torque [Nm]")
            plt.xlabel("$t$ [s]")
            plt.legend(legend)
        plt.draw()
        if save:
            plt.savefig(filename + "/joint_torques")

    def plot_target(self, save=False, filename=TEMP_DIRNAME):
        import matplotlib.pyplot as plt

        t_range = np.array([k * self.params.dt_wbc for k in range(self.tstamps.shape[0])])
        x = np.concatenate([self.q_filtered, self.v_filtered], axis=1)
        m_feet_p_log = {idx: get_translation_array(self.pd.model, x, idx)[0] for idx in self.pd.feet_ids}

        x_mpc = [self.ocp_xs[0][0, :]]
        [x_mpc.append(x[1, :]) for x in self.ocp_xs[:-1]]
        x_mpc = np.array(x_mpc)

        feet_p_log = {idx: get_translation_array(self.pd.model, x_mpc, idx)[0] for idx in self.pd.feet_ids}

        # Target plot
        _, axs = plt.subplots(3, 2, sharex=True)
        legend = ["x", "y", "z"]
        for p in range(3):
            axs[p, 0].set_title("Base position on " + legend[p])
            axs[p, 0].plot(self.q_estimate[:, p])
            axs[p, 0].plot(self.q_filtered[:, p])
            axs[p, 0].legend(["Estimated", "Filtered"])

            axs[p, 1].set_title("Base rotation on " + legend[p])
            axs[p, 1].plot(self.q_estimate_rpy[:, 3 + p])
            axs[p, 1].legend(["Estimated"])

        if save:
            plt.savefig(filename + "/base_position_target")

        _, axs = plt.subplots(3, 2, sharex=True)
        legend = ["x", "y", "z"]
        for p in range(3):
            axs[p, 0].set_title("Base velocity on " + legend[p])
            axs[p, 0].plot(self.target_base_linear[:, p])
            axs[p, 0].plot(self.v_estimate[:, p])
            axs[p, 0].plot(self.v_filtered[:, p])
            axs[p, 0].legend(["Target", "Estimated", "Filtered"])

            axs[p, 1].set_title("Base angular velocity on " + legend[p])
            axs[p, 1].plot(self.target_base_angular[:, p])
            axs[p, 1].plot(self.v_estimate[:, 3 + p])
            axs[p, 1].plot(self.v_filtered[:, 3 + p])
            axs[p, 1].legend(["Target", "Estimated", "Filtered"])
        if save:
            plt.savefig(filename + "/base_velocity_target")

        _, axs = plt.subplots(3, sharex=True)
        legend = ["x", "y", "z"]
        for p in range(3):
            axs[p].set_title("Free foot on " + legend[p])
            [axs[p].plot(m_feet_p_log[foot_id][:, p]) for foot_id in self.pd.feet_ids]
            axs[p].legend(self.pd.feet_names)
            # "Predicted"])
        if save:
            plt.savefig(filename + "/target")

        _, axs = plt.subplots(3, sharex=True)
        legend = ["x", "y", "z"]
        for p in range(3):
            axs[p].set_title("Predicted free foot on z over " + legend[p])
            [axs[p].plot(t_range, feet_p_log[foot_id][:, p]) for foot_id in self.pd.feet_ids]
            axs[p].legend(self.pd.feet_names)

        if save:
            plt.savefig(filename + "/target")

    def plot_riccati_gains(self, n, save=False, filename=TEMP_DIRNAME):
        import matplotlib.pyplot as plt

        # Equivalent Stiffness Damping plots
        legend = ["Hip", "Shoulder", "Knee"]
        plt.figure(figsize=(12, 18), dpi=FIG_DPI)
        for p in range(3):
            plt.subplot(3, 1, p + 1)
            plt.title("Joint:  " + legend[p])
            plt.plot(self.MPC_equivalent_Kp[:, p])
            plt.plot(self.MPC_equivalent_Kd[:, p])
            plt.legend(["Stiffness", "Damping"])
            plt.ylabel("Gains")
            plt.xlabel("t")

        if save:
            plt.savefig(filename + "/diagonal_Riccati_gains")

        # Riccati gains
        plt.figure(figsize=(12, 18), dpi=FIG_DPI)
        plt.title("Riccati gains at step: " + str(n))
        plt.imshow(self.ocp_K[n])
        plt.colorbar()
        if save:
            plt.savefig(filename + "/Riccati_gains")

    def plot_controller_times(self, save=False, filename=TEMP_DIRNAME):
        import matplotlib.pyplot as plt

        t_range = np.array([k * self.params.dt_mpc for k in range(self.tstamps.shape[0])])

        alpha = 0.7
        plt.figure(figsize=(9, 6), dpi=FIG_DPI)
        plt.plot(t_range, self.t_measures, "r+", alpha=alpha, label="Estimation")
        plt.plot(t_range, self.t_mpc, "g+", alpha=alpha, label="MPC (total)")
        # plt.plot(t_range, self.t_send, c="pink", marker="+", alpha=alpha, label="Sending command")
        plt.plot(t_range, self.t_loop, "+", c="violet", alpha=alpha, label="Entire loop")
        plt.plot(
            t_range,
            self.t_ocp_ddp,
            "1",
            c="blue",
            alpha=alpha,
            label="MPC (OCP solve)",
        )
        plt.axhline(
            y=self.params.dt_mpc,
            color="darkorange",
            linestyle="-",
            lw=1.2,
            label="mpc $\\Delta t$",
        )
        plt.legend()
        ylims = plt.ylim()
        plt.ylim(-0.004, min(ylims[1], 0.02))
        plt.xlabel("Clock  [s]")
        plt.ylabel("Timing [s]")
        plt.tight_layout()

        if save:
            plt.savefig(filename + "/timings")

    def plot_ocp_times(self):
        import matplotlib.pyplot as plt

        t_range = np.array([k * self.params.dt_mpc for k in range(self.tstamps.shape[0])])

        plt.figure()
        plt.plot(t_range, self.t_ocp_update, "r+")
        plt.plot(t_range, self.t_ocp_warm_start, "g+")
        plt.plot(t_range, self.t_ocp_ddp, "b+")
        plt.plot(t_range, self.t_ocp_solve, "+", color="violet")
        plt.axhline(y=self.params.dt_mpc, color="grey", linestyle=":", lw=1.0)
        lgd = ["t_ocp_update", "t_ocp_warm_start", "t_ocp_ddp", "t_ocp_solve"]
        plt.legend(lgd)
        plt.xlabel("Time [s]")
        plt.ylabel("Time [s]")

    def save(self, filename="data"):
        name = filename + "/data.npz"

        np.savez_compressed(
            name,
            solver_cls=self.solver_cls,
            target=self.target,
            target_base_linear=self.target_base_linear,
            target_base_angular=self.target_base_angular,
            q_estimate_rpy=self.q_estimate_rpy,
            q_estimate=self.q_estimate,
            v_estimate=self.v_estimate,
            q_filtered=self.q_filtered,
            v_filtered=self.v_filtered,
            ocp_xs=self.ocp_xs,
            ocp_us=self.ocp_us,
            ocp_K=self.ocp_K,
            MPC_equivalent_Kp=self.MPC_equivalent_Kp,
            MPC_equivalent_Kd=self.MPC_equivalent_Kd,
            t_measures=self.t_measures,
            t_mpc=self.t_mpc,
            t_send=self.t_send,
            t_loop=self.t_loop,
            t_ocp_update=self.t_ocp_update,
            t_ocp_warm_start=self.t_ocp_warm_start,
            t_ocp_ddp=self.t_ocp_ddp,
            t_ocp_solve=self.t_ocp_solve,
            wbc_P=self.wbc_P,
            wbc_D=self.wbc_D,
            wbc_q_des=self.wbc_q_des,
            wbc_v_des=self.wbc_v_des,
            wbc_FF=self.wbc_FF,
            wbc_tau_ff=self.wbc_tau_ff,
            tstamps=self.tstamps,
            q_mes=self.q_mes,
            v_mes=self.v_mes,
            baseOrientation=self.baseOrientation,
            baseOrientationQuat=self.baseOrientationQuat,
            baseAngularVelocity=self.baseAngularVelocity,
            baseLinearAcceleration=self.baseLinearAcceleration,
            baseAccelerometer=self.baseAccelerometer,
            torquesFromCurrentMeasurment=self.torquesFromCurrentMeasurment,
            mocapPosition=self.mocapPosition,
            mocapVelocity=self.mocapVelocity,
            mocapAngularVelocity=self.mocapAngularVelocity,
            mocapOrientationMat9=self.mocapOrientationMat9,
            mocapOrientationQuat=self.mocapOrientationQuat,
            current=self.current,
            voltage=self.voltage,
            energy=self.energy,
        )
        print("Logs saved in " + name)

    def load(self):
        if self.data is None:
            print("No data file loaded. Need one in the constructor.")
            return

        self.solver_cls = self.data["solver_cls"]
        # Load sensors arrays
        self.target = self.data["target"]
        self.target_base_linear = self.data["target_base_linear"]
        self.target_base_angular = self.data["target_base_angular"]
        self.q_estimate_rpy = self.data["q_estimate_rpy"]
        self.q_estimate = self.data["q_estimate"]
        self.v_estimate = self.data["v_estimate"]
        self.q_filtered = self.data["q_filtered"]
        self.v_filtered = self.data["v_filtered"]
        self.q_mes = self.data["q_mes"]
        self.v_mes = self.data["v_mes"]
        self.baseOrientation = self.data["baseOrientation"]
        self.baseOrientationQuat = self.data["baseOrientationQuat"]
        self.baseAngularVelocity = self.data["baseAngularVelocity"]
        self.baseLinearAcceleration = self.data["baseLinearAcceleration"]
        self.baseAccelerometer = self.data["baseAccelerometer"]
        self.torquesFromCurrentMeasurment = self.data["torquesFromCurrentMeasurment"]

        self.mocapPosition = self.data["mocapPosition"]
        self.mocapVelocity = self.data["mocapVelocity"]
        self.mocapAngularVelocity = self.data["mocapAngularVelocity"]
        self.mocapOrientationMat9 = self.data["mocapOrientationMat9"]
        self.mocapOrientationQuat = self.data["mocapOrientationQuat"]
        self.size = self.q_mes.shape[0]
        self.current = self.data["current"]
        self.voltage = self.data["voltage"]
        self.energy = self.data["energy"]

        # TODO: load your new data
        self.tstamps = self.data["tstamps"]
        self.t_mpc = self.data["t_mpc"]
        self.t_send = self.data["t_send"]
        self.t_ocp_ddp = self.data["t_ocp_ddp"]
        self.t_measures = self.data["t_measures"]
        self.v_estimate = self.data["v_estimate"]
        self.q_filtered = self.data["q_filtered"]
        self.v_filtered = self.data["v_filtered"]
        self.ocp_xs = self.data["ocp_xs"]
        self.ocp_us = self.data["ocp_us"]
        self.ocp_K = self.data["ocp_K"]
        self.t_mpc = self.data["t_mpc"]
        self.t_send = self.data["t_send"]
        self.t_loop = self.data["t_loop"]
        self.wbc_P = self.data["wbc_P"]
        self.wbc_D = self.data["wbc_D"]
        self.wbc_q_des = self.data["wbc_q_des"]


if __name__ == "__main__":
    import sys
    import os
    import argparse
    import quadruped_reactive_walking as qrw

    sys.path.insert(0, os.getcwd())

    parser = argparse.ArgumentParser(description="Process logs.")
    parser.add_argument("--file", type=str, help="A valid log file path")
    args = parser.parse_args()
    params = qrw.Params.create_from_file()
    pd = TaskSpec(params)

    today = datetime.now()
    today = today.strftime(DATE_STRFORMAT)
    logger = LoggerControl(params, filename=str(TEMP_DIRNAME / today / "data.npz"))

    logger.load()
    logger.plot()
