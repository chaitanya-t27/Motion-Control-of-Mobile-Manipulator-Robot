'''This code calls the previous three components to generate a reference trajectory, 
simulate robot motion, and compute control inputs to achieve the desired motion.'''

import modern_robotics as mr
import numpy as np
from NextState import NextState
from TrajectoryGenerator import Trajectory_generator
from FeedbackControl import FeedbackControl
import matplotlib.pyplot as plt

# The initial configuration of the robot
# the array of (chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state)
initial_config = np.array([0.5, -0.2, 0.1, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])


# Initial Configuration of the end effector Tse,initial
T_se_initial = np.array([[0, 0, 1, 0.5],
                        [0, 1, 0, 0],               
                        [-1, 0, 0, 0.4 ],               
                        [0, 0, 0, 1 ]])


# Initial Configuration of the cube Tsc,initial
T_scube_initial = np.array([[1, 0, 0, 1 ],
                            [0, 1, 0, 0 ],               
                            [0, 0, 1, 0.025 ],               
                            [0, 0, 0, 1 ]])

# Desired Final Configuration of the cube Tsc,final
T_scube_final = np.array([[0, 1, 0, 0 ],
                        [-1, 0, 0, -1 ],               
                        [0, 0, 1, 0.025 ],               
                        [0, 0, 0, 1 ]])


# Configuration of the end-effector relative to the cube while grasping Tce,grasp
T_ce_grasp = np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), 0 ],
                    [0, 1, 0, 0 ],               
                    [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0 ],               
                    [0, 0, 0, 1 ]])

# Standoff Configuration of the cube, before and after grasping, relative to the cube
T_ce_standoff = np.array([[-1/np.sqrt(2), 0, 1/np.sqrt(2), 0 ],
                    [0, 1, 0, 0 ],               
                    [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0.1],               
                    [0, 0, 0, 1 ]])

# The fixed offset from the chassis frame {b} to the base frame of the arm {0}
Tb0 = np.array([[1,0,0,0.1662],
                [0,1,0,0],
                [0,0,1,0.0026],
                [0,0,0,1]]) 

# The end-effector frame {e} at the zero configuration of the arm     
M0e = np.array([[1,0,0,0.033],
                [0,1,0,0],
                [0,0,1,0.6546],
                [0,0,0,1]])

Blist = np.array([  [0,0,1,0,0.033,0],
                    [0,-1,0,-0.5076,0,0],
                    [0,-1,0,-0.3526,0,0],
                    [0,-1,0,-0.2176,0,0],
                    [0,0,1,0,0,0]   ]).T

# Initialization of feedback control constants:###
kp = 2
ki = 15
Kp = kp * np.identity(6)
Ki = ki * np.identity(6)

# Restrictions on the speeds vector:###
max_jw_vels = 10
# Initialization of simulation constants:
k = 1
timestep = 0.01

# Getting Trajectory data
Trajectory_data = Trajectory_generator(T_se_initial,T_scube_initial,T_scube_final,T_ce_grasp,T_ce_standoff,k)


# Initialization of variable lists:
config_array = np.zeros((Trajectory_data.shape[0], 13))
Xerr_array = np.zeros((Trajectory_data.shape[0], 6))
Xerr_integral = np.zeros(6)

config_array[0] = initial_config

for i in range(1, Trajectory_data.shape[0]-1):
    # Calculating current end effector configuration
    current_config = config_array[i-1, :]
    Tsb = np.array([    [np.cos(current_config[0]),-np.sin(current_config[0]),0,current_config[1]],
                    [np.sin(current_config[0]),np.cos(current_config[0]),0,current_config[2]],
                    [0,0,1,0.0963],
                    [0,0,0,1]])
    T0e = mr.FKinBody(M0e, Blist, current_config[3:8])
    Tbe = Tb0.dot(T0e)
    # The current end effector configuration is:
    X = Tsb.dot(Tbe)

    # Calculating Desired and the next desired end effector configuration
    Tse_row = Trajectory_data[i]
    Tse_nextrow = Trajectory_data[i+1]
    Tse = np.array([  [Tse_row[0],Tse_row[1],Tse_row[2],Tse_row[9]],
                        [Tse_row[3],Tse_row[4],Tse_row[5],Tse_row[10]],
                        [Tse_row[6],Tse_row[7],Tse_row[8],Tse_row[11]],
                        [0,0,0,1]  ])
    Tse_next = np.array([  [Tse_nextrow[0],Tse_nextrow[1],Tse_nextrow[2],Tse_nextrow[9]],
                        [Tse_nextrow[3],Tse_nextrow[4],Tse_nextrow[5],Tse_nextrow[10]],
                        [Tse_nextrow[6],Tse_nextrow[7],Tse_nextrow[8],Tse_nextrow[11]],
                        [0,0,0,1]  ])
    # Hence the desired and the next desired end effector configuration are:
    Xd = Tse
    Xdnext = Tse_next

    # Calculating the joint and wheel velocities using the FeedbackControl function
    thetalist = current_config[3:8]
    Vd, V, Je, u_theta_dot, Xerr, Xerr_integral = FeedbackControl(X,Xd,Xdnext,Kp,Ki,timestep,thetalist,Xerr_integral)
    jw_vels = np.concatenate((u_theta_dot[4:],u_theta_dot[:4]),axis =None)

    # Calculate the next configuration using the NextState function
    current_config = NextState(current_config[:12],jw_vels,timestep,max_jw_vels)
    config_array[i] = np.concatenate( (current_config, Tse_row[12]), axis=None)

    # Updating the error array:
    Xerr_array[i-1] = Xerr

# Saving in a csv file
np.savetxt("Configuration_data.csv",config_array, delimiter=',')

# Plotring the Xerr data:
trajectory_idx = [i for i in range(config_array.shape[0])]
plt.figure()
plt.plot(trajectory_idx, Xerr_array[:, 0], label='Wx')
plt.plot(trajectory_idx, Xerr_array[:, 1], label='Wy')
plt.plot(trajectory_idx, Xerr_array[:, 2], label='Wz')
plt.plot(trajectory_idx, Xerr_array[:, 3], label='Vx')
plt.plot(trajectory_idx, Xerr_array[:, 4], label='Vy')
plt.plot(trajectory_idx, Xerr_array[:, 5], label='Vz')
plt.xlabel('Time in seconds')
plt.ylabel('Error in X')
plt.legend(loc="best")
plt.title(f'Xerr, Kp={kp}, Ki={ki}')
plt.savefig(f'Xerr, Kp={kp}, Ki={ki}.png')
plt.show()