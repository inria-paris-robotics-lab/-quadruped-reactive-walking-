from example_robot_data import load
import numpy as np
import pinocchio as pin


def init_robot(q_init, params):
    """
    Load the solo model and initialize some parameters

    Args:
        q_init (array): the default position of the robot actuators
        params (object): store parameters
    """
    # Load robot model and data
    solo = load("solo12")
    q = solo.q0.reshape((-1, 1))

    # Initial angular positions of actuators
    q[7:, 0] = q_init
    pin.framesForwardKinematics(solo.model, solo.data, q)

    # Initialisation of model quantities
    pin.centerOfMass(solo.model, solo.data, q, np.zeros((18, 1)))
    pin.updateFramePlacements(solo.model, solo.data)
    pin.crba(solo.model, solo.data, solo.q0)

    LEGS = ["FL", "FR", "HL", "HR"]

    # Initialisation of the position of footsteps
    initial_footsteps = np.zeros((3, 4))
    h_init = 0.0
    indexes = [solo.model.getFrameId(leg + "_FOOT") for leg in LEGS]
    for i in range(4):
        initial_footsteps[:, i] = solo.data.oMf[indexes[i]].translation
        h = (solo.data.oMf[1].translation - solo.data.oMf[indexes[i]].translation)[2]
        if h > h_init:
            h_init = h
    initial_footsteps[2, :] = 0.0

    # Initialisation of the position of shoulders
    initial_shoulders = np.zeros((3, 4))
    indexes = [solo.model.getFrameId(leg + "_SHOULDER") for leg in LEGS]
    for i in range(4):
        initial_shoulders[:, i] = solo.data.oMf[indexes[i]].translation

    # Saving data
    params.h_ref = h_init # + 0.155  # Adding foot radius
    params.mass = solo.data.mass[0]
    params.I_mat = solo.data.Ycrb[1].inertia.ravel().tolist()
    params.CoM_offset = (solo.data.com[0][:3] - q[0:3, 0]).tolist()
    params.CoM_offset[1] = 0.0

    params.mpc_wbc_ratio = int(params.dt_mpc / params.dt_wbc)
    params.T = params.gait.shape[0] - 1
    for i in range(4):
        for j in range(3):
            params.shoulders[3 * i + j] = initial_shoulders[j, i]
            params.footsteps_init[3 * i + j] = initial_footsteps[j, i]


def quaternionToRPY(quat):
    """
    Quaternion (4 x 0) to Roll Pitch Yaw (3 x 1)
    """
    qx = quat[0]
    qy = quat[1]
    qz = quat[2]
    qw = quat[3]

    rotateXa0 = 2.0 * (qy * qz + qw * qx)
    rotateXa1 = qw * qw - qx * qx - qy * qy + qz * qz
    rotateX = 0.0

    if (rotateXa0 != 0.0) and (rotateXa1 != 0.0):
        rotateX = np.arctan2(rotateXa0, rotateXa1)

    rotateYa0 = -2.0 * (qx * qz - qw * qy)
    rotateY = 0.0
    if rotateYa0 >= 1.0:
        rotateY = np.pi / 2.0
    elif rotateYa0 <= -1.0:
        rotateY = -np.pi / 2.0
    else:
        rotateY = np.arcsin(rotateYa0)

    rotateZa0 = 2.0 * (qx * qy + qw * qz)
    rotateZa1 = qw * qw + qx * qx - qy * qy - qz * qz
    rotateZ = 0.0
    if (rotateZa0 != 0.0) and (rotateZa1 != 0.0):
        rotateZ = np.arctan2(rotateZa0, rotateZa1)

    return np.array([[rotateX], [rotateY], [rotateZ]])
