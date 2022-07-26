from datetime import datetime
from time import time
import numpy as np
from .kinematics_utils import get_translation, get_translation_array
import matplotlib
import matplotlib.pyplot as plt


class LoggerControl:
    def __init__(self, pd, log_size=60e3, loop_buffer=False, file=None):
        if file is not None:
            self.data = np.load(file, allow_pickle=True)

        self.log_size = np.int(log_size)
        self.i = 0
        self.loop_buffer = loop_buffer

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

        self.t_ocp_update_FK = np.zeros(size)
        self.t_ocp_shift = np.zeros(size)
        self.t_ocp_update_last = np.zeros(size)
        self.t_ocp_update_terminal = np.zeros(size)

        # MPC
        self.ocp_storage = {
            "xs": np.zeros([size, pd.T + 1, pd.nx]),
            "us": np.zeros([size, pd.T, pd.nu]),
        }
        self.target = np.zeros([size, 3])

        # Whole body control
        self.wbc_P = np.zeros([size, 12])  # proportionnal gains of the PD+
        self.wbc_D = np.zeros([size, 12])  # derivative gains of the PD+
        self.wbc_q_des = np.zeros([size, 12])  # desired position of actuators
        self.wbc_v_des = np.zeros([size, 12])  # desired velocity of actuators
        self.wbc_FF = np.zeros([size, 12])  # gains for the feedforward torques
        self.wbc_tau_ff = np.zeros([size, 12])  # feedforward torques

    def sample(self, controller, device, qualisys=None):
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
        self.target[self.i] = controller.point_target
        self.t_mpc[self.i] = controller.mpc.ocp.results.solver_time
        self.t_send[self.i] = controller.t_send
        self.t_loop[self.i] = controller.t_loop
        self.t_measures[self.i] = controller.t_measures

        # Logging from model predictive control
        self.ocp_storage["xs"][self.i] = np.array(controller.mpc.ocp.results.x)
        self.ocp_storage["us"][self.i] = np.array(controller.mpc.ocp.results.u)

        self.t_measures[self.i] = controller.t_measures
        self.t_mpc[self.i] = controller.t_mpc
        self.t_send[self.i] = controller.t_send
        self.t_loop[self.i] = controller.t_loop

        self.t_ocp_update[self.i] = controller.mpc.ocp.t_update
        self.t_ocp_warm_start[self.i] = controller.mpc.ocp.t_warm_start
        self.t_ocp_ddp[self.i] = controller.mpc.ocp.t_ddp
        self.t_ocp_solve[self.i] = controller.mpc.ocp.t_solve

        self.t_ocp_update_FK[self.i] = controller.mpc.ocp.t_FK
        self.t_ocp_shift[self.i] = controller.mpc.ocp.t_shift
        self.t_ocp_update_last[self.i] = controller.mpc.ocp.t_update_last_model
        self.t_ocp_update_terminal[self.i] = controller.mpc.ocp.t_update_terminal_model

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

        all_ocp_feet_p_log = {
            idx: [
                get_translation_array(self.pd, x, idx)[0]
                for x in self.ocp_storage["xs"]
            ]
            for idx in self.pd.allContactIds
        }
        for foot in all_ocp_feet_p_log:
            all_ocp_feet_p_log[foot] = np.array(all_ocp_feet_p_log[foot])

        x_mes = np.concatenate([self.q_mes[:, 3:6], self.v_mes[:, 3:6]], axis = 1)
        feet_p_log = {
            idx: 
                get_translation_array(self.pd, x_mes, idx)[0]
            for idx in self.pd.allContactIds
        }
        
        

        # plt.figure(figsize=(12, 6), dpi=90)
        # plt.title("Solver timings")
        # plt.hist(self.ocp_timings, 30)
        # plt.xlabel("timee [s]")
        # plt.ylabel("Number of cases [#]")
        # plt.draw()
        # if save:
        #     plt.savefig(fileName + "_solver_timings")

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
            plt.savefig(fileName + "_joint_positions")

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
            plt.savefig(fileName + "_joint_velocities")

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
            plt.savefig(fileName + "_joint_torques")

        legend = ["x", "y", "z"]
        plt.figure(figsize=(12, 18), dpi = 90)
        for p in range(3):
            plt.subplot(3,1, p+1)
            plt.title('Free foot on ' + legend[p])
            plt.plot(self.target[:, p])
            plt.plot(feet_p_log[self.pd.rfFootId][:, p])
            plt.legend(["Desired", "Measured"])

        self.plot_controller_times()
        self.plot_OCP_times()
        self.plot_OCP_update_times()

        plt.show()

    def plot_controller_times(self):
        import matplotlib.pyplot as plt

        t_range = np.array([k * self.pd.dt for k in range(self.tstamps.shape[0])])

        plt.figure()
        plt.plot(t_range, self.t_measures, "r+")
        plt.plot(t_range, self.t_mpc, "g+")
        plt.plot(t_range, self.t_send, "b+")
        plt.plot(t_range, self.t_loop, "+", color="violet")
        plt.axhline(y=0.001, color="grey", linestyle=":", lw=1.0)
        plt.axhline(y=0.01, color="grey", linestyle=":", lw=1.0)
        lgd = ["Measures", "MPC", "Send", "Whole-loop"]
        plt.legend(lgd)
        plt.xlabel("Time [s]")
        plt.ylabel("Time [s]")

    def plot_OCP_times(self):
        import matplotlib.pyplot as plt

        t_range = np.array([k * self.pd.dt for k in range(self.tstamps.shape[0])])

        plt.figure()
        plt.plot(t_range, self.t_ocp_update, "r+")
        plt.plot(t_range, self.t_ocp_warm_start, "g+")
        plt.plot(t_range, self.t_ocp_ddp, "b+")
        plt.plot(t_range, self.t_ocp_solve, "+", color="violet")
        plt.axhline(y=0.001, color="grey", linestyle=":", lw=1.0)
        lgd = ["t_ocp_update", "t_ocp_warm_start", "t_ocp_ddp", "t_ocp_solve"]
        plt.legend(lgd)
        plt.xlabel("Time [s]")
        plt.ylabel("Time [s]")

    def plot_OCP_update_times(self):
        import matplotlib.pyplot as plt

        t_range = np.array([k * self.pd.dt for k in range(self.tstamps.shape[0])])

        plt.figure()
        plt.plot(t_range, self.t_ocp_update_FK, "r+")
        plt.plot(t_range, self.t_ocp_shift, "g+")
        plt.plot(t_range, self.t_ocp_update_last, "b+")
        plt.plot(t_range, self.t_ocp_update_terminal, "+", color="seagreen")
        plt.axhline(y=0.001, color="grey", linestyle=":", lw=1.0)
        lgd = [
            "t_ocp_update_FK",
            "t_ocp_shift",
            "t_ocp_update_last",
            "t_ocp_update_terminal",
        ]
        plt.legend(lgd)
        plt.xlabel("Time [s]")
        plt.ylabel("Time [s]")

    def save(self, fileName="data"):
        date_str = datetime.now().strftime("_%Y_%m_%d_%H_%M")
        name = fileName + date_str + ".npz"

        np.savez_compressed(
            name,
            ocp_storage=self.ocp_storage,
            t_measures=self.t_measures,
            t_mpc=self.t_mpc,
            t_send=self.t_send,
            t_loop=self.t_loop,
            t_ocp_update=self.t_ocp_update,
            t_ocp_warm_start=self.t_ocp_warm_start,
            t_ocp_ddp=self.t_ocp_ddp,
            t_ocp_solve=self.t_ocp_solve,
            t_ocp_update_FK=self.t_ocp_update_FK,
            t_ocp_shift=self.t_ocp_shift,
            t_ocp_update_last=self.t_ocp_update_last,
            t_ocp_update_terminal=self.t_ocp_update_terminal,
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
        print("Logs and plots saved in " + name)

    def load(self):
        if self.data is None:
            print("No data file loaded. Need one in the constructor.")
            return

        # Load sensors arrays
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
        self.t_mpc = self.data["mpc_solving_duration"]
        self.t_send = self.data["t_send"]
        self.t_loop = self.data["t_loop"]
        self.t_measures = self.data["t_meausres"]

        self.ocp_storage = self.data["ocp_storage"].item()

        self.t_measures = self.data["t_measures"]
        self.t_mpc = self.data["t_mpc"]
        self.t_send = self.data["t_send"]
        self.t_loop = self.data["t_loop"]
        self.wbc_P = self.data["wbc_P"]
        self.wbc_D = self.data["wbc_D"]
        self.wbc_q_des = self.data["wbc_q_des"]
        self.wbc_v_des = self.data["wbc_v_des"]
        self.wbc_FF = self.data["wbc_FF"]
        self.wbc_tau_ff = self.data["wbc_tau_ff"]

        self.tstamps = self.data["tstamps"]


if __name__ == "__main__":
    import sys
    import os
    import argparse
    import quadruped_reactive_walking as qrw
    from quadruped_reactive_walking.tools import self

    sys.path.insert(0, os.getcwd())

    parser = argparse.ArgumentParser(description="Process logs.")
    parser.add_argument("--file", type=str, help="A valid log file path")
    args = parser.parse_args()

    logger = LoggerControl(file=args.file)
    logger.load()
    logger.plot()
