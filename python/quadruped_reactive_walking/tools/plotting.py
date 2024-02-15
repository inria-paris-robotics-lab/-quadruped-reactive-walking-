import numpy as np

from quadruped_reactive_walking import MPCResult


def plot_mpc(task, mpc_result: MPCResult, base=False, joints=True):
    import matplotlib.pyplot as plt

    if base:
        legend = ["X", "Y", "Z"]
        _, axs = plt.subplots(2)
        [axs[0].plot(np.array(mpc_result.xs)[:, axis]) for axis in range(3)]
        axs[0].legend(legend)
        axs[0].set_title("Base position")

        [axs[1].plot(np.array(mpc_result.xs)[:, 19 + axis]) for axis in range(3)]
        axs[1].legend(legend)
        axs[1].set_title("Base velocity")

    if joints:
        legend = ["Hip", "Shoulder", "Knee"]
        _, axs = plt.subplots(3, 4, sharex=True)
        for foot in range(4):
            [axs[0, foot].plot(np.array(mpc_result.xs)[:, 7 + 3 * foot + joint]) for joint in range(3)]
            axs[0, foot].legend(legend)
            axs[0, foot].set_title("Joint positions for " + task.feet_names[foot])

            [axs[1, foot].plot(np.array(mpc_result.xs)[:, 19 + 6 + 3 * foot + joint]) for joint in range(3)]
            axs[1, foot].legend(legend)
            axs[1, foot].set_title("Joint velocity for " + task.feet_names[foot])

            [axs[2, foot].plot(np.array(mpc_result.us)[:, 3 * foot + joint]) for joint in range(3)]
            axs[2, foot].legend(legend)
            axs[2, foot].set_title("Joint torques for foot " + task.feet_names[foot])

    plt.show()
