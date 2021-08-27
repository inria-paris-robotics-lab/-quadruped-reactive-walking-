# coding: utf8

import sys
import os 
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

import numpy as np
import matplotlib.pylab as plt
import pinocchio as pin
import libquadruped_reactive_walking as lqrw

def get_mocap_logs(path):
    ''' Get mocap logs and store state in (N,12) array in Base frame (local).
    Position: x,y,z
    Orientation: Roll, Pitch, Yaw
    Linear Velocity: Vx, Vy, Vz
    Angular Velocity: Wroll, Wpitch, Wyaw
    Args:
    - path (str) : path to the .npz file object containing the measures

    Returns:
    - array (Nx12) : Array containing the data
    '''
    # Recover MOCAP logs
    logs = np.load(path)
    mocapPosition = logs.get("mocapPosition")
    mocapOrientationQuat = logs.get("mocapOrientationQuat")
    mocapOrientationMat9 = logs.get("mocapOrientationMat9")
    mocapVelocity = logs.get("mocapVelocity")
    mocapAngularVelocity = logs.get('mocapAngularVelocity')
    N = mocapPosition.shape[0]

    state_measured = np.zeros((N,12))
    # Roll, Pitch, Yaw
    for i in range(N):
        state_measured[i,3:6] = pin.rpy.matrixToRpy(pin.Quaternion(mocapOrientationQuat[i]).toRotationMatrix())

    # Robot world to Mocap initial translationa and rotation
    mTo = np.array([mocapPosition[0, 0], mocapPosition[0, 1], 0.02])  
    mRo = pin.rpy.rpyToMatrix(0.0, 0.0, state_measured[0, 5])

    for i in range(N):
        oRb = mocapOrientationMat9[i]
        oRh = pin.rpy.rpyToMatrix(0.0, 0.0, state_measured[i, 5] - state_measured[0, 5])

        state_measured[i,:3] = (mRo.transpose() @ (mocapPosition[i, :] - mTo).reshape((3, 1))).ravel()
        state_measured[i,6:9] = (oRh.transpose() @ mRo.transpose() @ mocapVelocity[i].reshape((3, 1))).ravel()
        state_measured[i,9:12] = (oRb.transpose() @ mocapAngularVelocity[i].reshape((3, 1))).ravel()       

    return state_measured

def compute_RMSE(array, norm):
    return np.sqrt((array**2).mean()) / norm

##############
# PARAMETERS 
##############

# [Linear, Non Linear, Planner, OSQP]
MPCs = [True, True, True, True] # Boolean to choose which MPC to plot
MPCs_names = ["No FF", "-afeet", "JT fmpc", "JT fmpc + M ddq"]
name_files = ["data_2021_08_27_16_08_0.npz", "data_2021_08_27_15_53_0.npz", "data_2021_08_27_15_42_0.npz", "data_2021_08_27_15_44_0.npz"] # Names of the files
folder_path = "" # Folder containing the 4 .npz files

# Common data shared by 4 MPCs
params = lqrw.Params()  # Object that holds all controller parameters
logs = np.load(folder_path + name_files[0])
joy_v_ref = logs.get('joy_v_ref')       # Ref velocity (Nx6) given by the joystick  
planner_xref = logs.get("planner_xref") # Ref state
N = joy_v_ref.shape[0]                  # Size of the measures
data_ = np.zeros((N,12,4))              # Store states measured by MOCAP, 4 MPCs (pos,orientation,vel,ang vel)
tau_ff_ = np.zeros((N,12,4))             # Store feedforward torques

# Get state measured
for i in range(4):
    if MPCs[i]:
        data_[:,:,i] = get_mocap_logs(folder_path + name_files[i])
        tau_ff_[:, :, i] = np.load(folder_path + name_files[i]).get("wbc_tau_ff")

for j in range(4):
    for i in range(12):
        for t in range(N):
            if np.isnan(data_[t,i,j]):
                data_[t,i,j] = data_[t-1,i,j]

##########
# PLOTS 
##########
lgd = ["Position X", "Position Y", "Position Z", "Position Roll", "Position Pitch", "Position Yaw"]
index6 = [1, 3, 5, 2, 4, 6]
t_range = np.array([k*params.dt_wbc for k in range(N)])

color = ["k", "b", "r", "g--"]
legend = []
for i in range(4):
    if MPCs[i]:
        legend.append(MPCs_names[i])


plt.figure()
for i in range(6):
    plt.subplot(3, 2, index6[i])
    
    for j in range(4):
        if MPCs[j]:
            plt.plot(t_range, data_[:,i,j], color[j], linewidth=3)
    
   
    plt.legend(legend, prop={'size': 8})
    plt.ylabel(lgd[i])
plt.suptitle("Measured postion and orientation - MOCAP - ")

lgd = ["Linear vel X", "Linear vel Y", "Linear vel Z",
               "Angular vel Roll", "Angular vel Pitch", "Angular vel Yaw"]
plt.figure()
for i in range(6):
    plt.subplot(3, 2, index6[i])
    
    for j in range(4):
        if MPCs[j]:
            plt.plot(t_range, data_[:,i+6,j], color[j], linewidth=3)    
   
    plt.legend(legend, prop={'size': 8})
    plt.ylabel(lgd[i])
plt.suptitle("Measured postion and orientation - MOCAP - ")

# Compute difference measured - reference
data_diff = np.zeros((N, 12,4))

for i in range(4):
    if MPCs[i]:
        data_diff[:,:6,i] = data_[:,:6,i] - planner_xref[:, :6, 1]  # Position and orientation
        data_diff[:,6:,i] = data_[:,6:,i] - joy_v_ref[:,:]          # Linear and angular velocities

lgd = ["Linear vel X", "Linear vel Y", "Position Z",
               "Position Roll", "Position Pitch", "Ang vel Yaw"]
index_error = [6,7,2,3,4,11]




# Compute the mean of the difference (measured - reference)
# Using a window mean (valid, excludes the boundaries). 
# The size of the data output are then reduced by period - 1

period = int( 2* (params.T_gait / 2 ) / params.dt_wbc ) # Period of the window 
data_diff_valid = data_diff[int(period/2 - 1) : -int(period/2) , :,:] # Reshape of the (measure - ref) arrays
t_range_valid = t_range[int(period/2 - 1) : -int(period/2) ] # Reshape of the timing for plottings
data_diff_mean_valid = np.zeros(data_diff_valid.shape) # Mean array
data_mean_valid = np.zeros(data_diff_valid.shape) # Mean array

for j in range(4):
    for i in range(12):
        data_mean_valid[:,i,j]  = np.convolve(data_diff[:,i,j], np.ones((period,)) / period, mode = "valid")
        data_diff_mean_valid[:,i,j] = data_diff_valid[:,i,j] - np.convolve(data_diff[:,i,j], np.ones((period,)) / period, mode = "valid")

plt.figure()
for i in range(6):
    plt.subplot(3, 2, index6[i])
    
    for j in range(4):
        if MPCs[j]:
            plt.plot(t_range, data_diff[:,index_error[i],j], color[j], linewidth=3)    

# Add mean on graph
# for i in range(6):
#     plt.subplot(3, 2, index6[i])    
#     for j in range(4):
#         if MPCs[j]:
#             plt.plot(t_range_valid, data_mean_valid[:,index_error[i],j], color[j] + "x-", linewidth=3)   

    plt.legend(legend, prop={'size': 8})
    plt.ylabel(lgd[i])
plt.suptitle("Error wrt reference state")

plt.figure()
for i in range(6):
    plt.subplot(3, 2, index6[i])
    
    for j in range(4):
        if MPCs[j]:
            plt.plot(t_range_valid, data_diff_mean_valid[:,index_error[i],j], color[j], linewidth=3)    
   
    plt.legend(legend, prop={'size': 8})
    plt.ylabel(lgd[i])
plt.suptitle("Error wrt reference state - smooth mean (window of 2 period) - ")



data_RMSE = np.zeros((12,4))
data_RMSE_mean = np.zeros((12,4))

norm_max = np.max(abs(data_[:,:,0]) , axis = 0) # Max of first MPC as norm for each component
norm_max_mean = np.max(abs(data_[:,:,0]) , axis = 0) # Max of first MPC as norm for each component

for i in range(12):
    for j in range(4):
        if MPCs[j]:
            data_RMSE[i,j] = compute_RMSE(data_diff[:,i,j], norm_max[i])
            data_RMSE_mean[i,j] = compute_RMSE(data_diff_mean_valid[:,i,j], norm_max_mean[i])

lgd = ["Linear vel X", "Linear vel Y", "Position Z",
               "Position Roll", "Position Pitch", "Ang vel Yaw"]
index_error = [6,7,2,3,4,11]
bars = []
bars_names = ["Lin", "NL", "Plan", "OSQP"]
for j in range(4):
    if MPCs[j]:
        bars.append(bars_names[j])

plt.figure()
for i in range(6):
    plt.subplot(3, 2, index6[i])    
    heights = []

    for j in range(4):
        if MPCs[j]:
            heights.append(data_RMSE[index_error[i] , j])

        
    y_pos = range(len(bars))
    plt.bar(y_pos, heights)
    plt.ylim([0., 0.6])
    # Rotation of the bars names
    plt.xticks(y_pos, bars, rotation=0)
    plt.ylabel(lgd[i])
plt.suptitle("NORMALIZED RMSE : sqrt(  (measures - ref**2).mean() ) / measure_max")



plt.figure()
for i in range(6):
    plt.subplot(3, 2, index6[i])    
    heights = []

    for j in range(4):
        if MPCs[j]:
            heights.append(data_RMSE_mean[index_error[i] , j])

        
    y_pos = range(len(bars))
    plt.bar(y_pos, heights)
    plt.ylim([0., 0.6])
    # Rotation of the bars names
    plt.xticks(y_pos, bars, rotation=0)
    plt.ylabel(lgd[i])
plt.suptitle("NORMALIZED RMSE -MEAN: sqrt(  (mes - ref - mean(mes-ref))  **2).mean() ) / measure_max")


####
# FF torques & FB torques & Sent torques & Meas torques
####
index12 = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]
lgd1 = ["HAA", "HFE", "Knee"]
lgd2 = ["FL", "FR", "HL", "HR"]
plt.figure()
my_axs = []
for i in range(12):
    if i == 0:
        ax = plt.subplot(3, 4, index12[i])
        my_axs.append(ax)
    elif i in [1, 2]:
        ax = plt.subplot(3, 4, index12[i], sharex=my_axs[0])
        my_axs.append(ax)
    else:
        plt.subplot(3, 4, index12[i], sharex=my_axs[0], sharey=my_axs[int(i % 3)])

    for j in range(4):
        if MPCs[j]:
            plt.plot(t_range, tau_ff_[:,i,j], color[j], linewidth=3)

    plt.xlabel("Time [s]")
    plt.ylabel(lgd1[i % 3]+" "+lgd2[int(i/3)]+" [Nm]")
    tmp = lgd1[i % 3]+" "+lgd2[int(i/3)]
    plt.legend(legend, prop={'size': 8})
    plt.ylim([-8.0, 8.0])

# Display all graphs and wait
plt.show(block=True)

