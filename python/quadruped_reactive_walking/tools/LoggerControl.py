from datetime import datetime
from time import time
import numpy as np
from .kinematics_utils import get_translation, get_translation_array
from ..Controller import Controller


class LoggerControl:
    def __init__(self, pd, params, log_size=60e3, loop_buffer=False, file=None):
        if file is not None:
            self.data = np.load(file, allow_pickle=True)

        self.log_size = np.int(log_size)
        self.i = 0
        self.loop_buffer = loop_buffer
        self.params = params

        size = self.log_size
        self.pd = pd

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
        self.t_measures = np.zeros(size)
        self.t_mpc = np.zeros(size)  # solver time #measurement time
        self.t_send = np.zeros(size)  #
        self.t_loop = np.zeros(size)  # controller time loop
        self.t_whole = np.zeros(size)  # controller time loop

        self.t_ocp_update = np.zeros(size)
        self.t_ocp_warm_start = np.zeros(size)
        self.t_ocp_ddp = np.zeros(size)
        self.t_ocp_solve = np.zeros(size)

        # MPC
        self.q_estimate_rpy = np.zeros([size, pd.nq - 1])
        self.q_estimate = np.zeros([size, pd.nq])
        self.v_estimate = np.zeros([size, pd.nv])
        self.q_filtered = np.zeros([size, pd.nq])
        self.v_filtered = np.zeros([size, pd.nv])
        self.ocp_xs = np.zeros([size, params.T + 1, pd.nx])
        self.ocp_us = np.zeros([size, params.T, pd.nu])
        self.ocp_K = np.zeros([size, self.pd.nu, self.pd.ndx])
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
        self.q_mes[self.i] = device.joints.positions
        self.v_mes[self.i] = device.joints.velocities
        self.baseOrientation[self.i] = device.imu.attitude_euler
        self.baseOrientationQuat[self.i] = device.imu.attitude_quaternion
        self.baseAngularVelocity[self.i] = device.imu.gyroscope
        self.baseLinearAcceleration[self.i] = device.imu.linear_acceleration
        self.baseAccelerometer[self.i] = device.imu.accelerometer
        self.torquesFromCurrentMeasurment[self.i] = device.joints.measured_torques
        self.current[self.i] = device.powerboard.current
        self.voltage[self.i] = device.powerboard.voltage
        self.energy[self.i] = device.powerboard.energy

        # Logging from qualisys (motion capture)
        if qualisys is not None:
            self.mocapPosition[self.i] = qualisys.getPosition()
            self.mocapVelocity[self.i] = qualisys.getVelocity()
            self.mocapAngularVelocity[self.i] = qualisys.getAngularVelocity()
            self.mocapOrientationMat9[self.i] = qualisys.getOrientationMat9()
            self.mocapOrientationQuat[self.i] = qualisys.getOrientationQuat()
        else:  # Logging from PyBullet simulator through fake device
            self.mocapPosition[self.i] = device.baseState[0]
            self.mocapVelocity[self.i] = device.baseVel[0]
            self.mocapAngularVelocity[self.i] = device.baseVel[1]
            self.mocapOrientationMat9[self.i] = device.rot_oMb
            self.mocapOrientationQuat[self.i] = device.baseState[1]

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
        self.MPC_equivalent_Kp[self.i] = controller.mpc_result.K[0].diagonal()
        self.MPC_equivalent_Kd[self.i] = controller.mpc_result.K[0].diagonal(3)

        self.t_measures[self.i] = controller.t_measures
        self.t_mpc[self.i] = controller.t_mpc
        self.t_send[self.i] = controller.t_send
        self.t_loop[self.i] = controller.t_loop

        self.t_ocp_ddp[self.i] = controller.mpc_result.solving_duration

        if self.i == 0:
            for i in range(self.params.T * self.params.mpc_wbc_ratio):
                self.target[i] = controller.footsteps[i // self.params.mpc_wbc_ratio][
                    :, 1
                ]
                self.target_base_linear[i] = controller.base_refs[
                    i // self.params.mpc_wbc_ratio
                ][:3]
                self.target_base_angular[i] = controller.base_refs[
                    i // self.params.mpc_wbc_ratio
                ][3:]
        if self.i + self.params.T * self.params.mpc_wbc_ratio < self.log_size:
            self.target[
                self.i + self.params.T * self.params.mpc_wbc_ratio
            ] = controller.target_footstep[:, 1]
            self.target_base_linear[
                self.i + self.params.T * self.params.mpc_wbc_ratio
            ] = controller.v_ref[:][:3]

            self.target_base_angular[
                self.i + self.params.T * self.params.mpc_wbc_ratio
            ] = controller.v_ref[:][3:]

        if not self.params.enable_multiprocessing:
            self.t_ocp_update[self.i] = controller.mpc.ocp.t_update
            self.t_ocp_warm_start[self.i] = controller.mpc.ocp.t_warm_start
            self.t_ocp_solve[self.i] = controller.mpc.ocp.t_solve

        # Logging from whole body control
        self.wbc_P[self.i] = controller.result.P
        self.wbc_D[self.i] = controller.result.D
        self.wbc_q_des[self.i] = controller.result.q_des
        self.wbc_v_des[self.i] = controller.result.v_des
        self.wbc_FF[self.i] = controller.result.FF
        self.wbc_tau_ff[self.i] = controller.result.tau_ff

        # Logging timestamp
        self.tstamps[self.i] = time()

        self.i += 1

    def plot(self, save=False, fileName="tmp/"):
        import matplotlib.pyplot as plt

        self.plot_states(save, fileName)
        self.plot_torques(save, fileName)
        self.plot_target(save, fileName)
        # self.plot_riccati_gains(0, save, fileName)
        self.plot_controller_times(save, fileName)
        # if not self.params.enable_multiprocessing:
        #     self.plot_OCP_times()

    def plot_states(self, save=False, fileName="/tmp"):
        import matplotlib.pyplot as plt

        legend = ["Hip", "Shoulder", "Knee"]
        plt.figure(figsize=(12, 6), dpi=90)
        i = 0
        for i in range(4):
            plt.subplot(2, 2, i + 1)
            plt.title("Joint position of " + str(i))
            [
                plt.plot(np.array(self.q_mes)[:, (3 * i + jj)] * 180 / np.pi)
                for jj in range(3)
            ]
            plt.ylabel("Joint position [deg]")
            plt.xlabel("t[s]")
            plt.legend(legend)
        plt.draw()
        if save:
            plt.savefig(fileName + "/joint_positions")

        plt.figure(figsize=(12, 6), dpi=90)
        i = 0
        for i in range(4):
            plt.subplot(2, 2, i + 1)
            plt.title("Joint velocity of " + str(i))
            [
                plt.plot(np.array(self.v_mes)[:, (3 * i + jj)] * 180 / np.pi)
                for jj in range(3)
            ]
            plt.ylabel("Joint velocity [deg/s]")
            plt.xlabel("t[s]")
            plt.legend(legend)
        plt.draw()
        if save:
            plt.savefig(fileName + "/joint_velocities")

    def plot_torques(self, save=False, fileName="/tmp"):
        import matplotlib.pyplot as plt

        legend = ["Hip", "Shoulder", "Knee"]
        plt.figure(figsize=(12, 6), dpi=90)
        i = 0
        for i in range(4):
            plt.subplot(2, 2, i + 1)
            plt.title("Joint torques of " + str(i))
            [
                plt.plot(np.array(self.torquesFromCurrentMeasurment)[:, (3 * i + jj)])
                for jj in range(3)
            ]
            plt.ylabel("Torque [Nm]")
            plt.xlabel("t[s]")
            plt.legend(legend)
        plt.draw()
        if save:
            plt.savefig(fileName + "/joint_torques")

    def plot_target(self, save=False, fileName="/tmp"):
        import matplotlib.pyplot as plt

        t_range = np.array(
            [k * self.params.dt_wbc for k in range(self.tstamps.shape[0])]
        )
        x = np.concatenate([self.q_filtered, self.v_filtered], axis=1)
        m_feet_p_log = {
            idx: get_translation_array(self.pd, x, idx)[0] for idx in self.pd.feet_ids
        }

        x_mpc = [self.ocp_xs[0][0, :]]
        [x_mpc.append(x[1, :]) for x in self.ocp_xs[:-1]]
        x_mpc = np.array(x_mpc)

        feet_p_log = {
            idx: get_translation_array(self.pd, x_mpc, idx)[0]
            for idx in self.pd.feet_ids
        }

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
            plt.savefig(fileName + "/base_position_target")

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
            plt.savefig(fileName + "/base_velocity_target")

        _, axs = plt.subplots(3, sharex=True)
        legend = ["x", "y", "z"]
        for p in range(3):
            axs[p].set_title("Free foot on " + legend[p])
            [axs[p].plot(m_feet_p_log[foot_id][:, p]) for foot_id in self.pd.feet_ids]
            axs[p].legend(self.pd.feet_names)
            # "Predicted"])
        if save:
            plt.savefig(fileName + "/target")

        _, axs = plt.subplots(3, sharex=True)
        legend = ["x", "y", "z"]
        for p in range(3):
            axs[p].set_title("Predicted free foot on z over " + legend[p])
            [
                axs[p].plot(t_range, feet_p_log[foot_id][:, p])
                for foot_id in self.pd.feet_ids
            ]
            axs[p].legend(self.pd.feet_names)

        if save:
            plt.savefig(fileName + "/target")

    def plot_riccati_gains(self, n, save=False, fileName="/tmp"):
        import matplotlib.pyplot as plt

        # Equivalent Stiffness Damping plots
        legend = ["Hip", "Shoulder", "Knee"]
        plt.figure(figsize=(12, 18), dpi=90)
        for p in range(3):
            plt.subplot(3, 1, p + 1)
            plt.title("Joint:  " + legend[p])
            plt.plot(self.MPC_equivalent_Kp[:, p])
            plt.plot(self.MPC_equivalent_Kd[:, p])
            plt.legend(["Stiffness", "Damping"])
            plt.ylabel("Gains")
            plt.xlabel("t")

        if save:
            plt.savefig(fileName + "/diagonal_Riccati_gains")

        # Riccati gains
        plt.figure(figsize=(12, 18), dpi=90)
        plt.title("Riccati gains at step: " + str(n))
        plt.imshow(self.ocp_K[n])
        plt.colorbar()
        if save:
            plt.savefig(fileName + "/Riccati_gains")

    def plot_controller_times(self, save=False, fileName="/tmp"):
        import matplotlib.pyplot as plt

        t_range = np.array(
            [k * self.params.dt_mpc for k in range(self.tstamps.shape[0])]
        )

        plt.figure()
        plt.plot(t_range, self.t_measures, "r+")
        plt.plot(t_range, self.t_mpc, "g+")
        plt.plot(t_range, self.t_send, "b+")
        plt.plot(t_range, self.t_loop, "+", color="violet")
        plt.plot(t_range, self.t_ocp_ddp, "+", color="royalblue")
        plt.axhline(y=self.params.dt_wbc, color="grey", linestyle=":", lw=1.0)
        lgd = ["Measures", "MPC", "Send", "Whole-loop", "MPC solve"]
        plt.legend(lgd)
        plt.xlabel("Time [s]")
        plt.ylabel("Time [s]")

        if save:
            plt.savefig(fileName + "/timings")

    def plot_OCP_times(self):
        import matplotlib.pyplot as plt

        t_range = np.array(
            [k * self.params.dt_mpc for k in range(self.tstamps.shape[0])]
        )

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

    def save(self, fileName="data"):
        name = fileName + "/data.npz"

        np.savez_compressed(
            name,
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
        self.t_loop = self.data["t_loop"]
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
    from ..WB_MPC.problem_data import ProblemData
    from .Utils import init_robot

    sys.path.insert(0, os.getcwd())

    parser = argparse.ArgumentParser(description="Process logs.")
    parser.add_argument("--file", type=str, help="A valid log file path")
    args = parser.parse_args()
    params = qrw.Params()
    init_robot(params.q_init, params)
    pd = ProblemData(params)

    logger = LoggerControl(pd, params, file="/tmp/logs/2022_09_12_16_43/data.npz")

    logger.load()
    logger.plot()
