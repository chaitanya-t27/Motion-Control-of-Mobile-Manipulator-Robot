
'''
This code consists of a NextState function which returns the next state of the robot configuration.
The later part consists of testing the NextState function.

'''

import modern_robotics as mr
import numpy as np
from scipy.linalg import expm


def NextState(Current_state,jw_vels,timestep,max_jw_vels):
    '''
    This function uses the kinematics of the youBot and Euler method to predict how the
    robot will move in a small timestep given its current configuration and velocity.

    Inputs:
    • The current state of the robot (12 variables: 3 for chassis, 5 for arm, 4 for wheel angles)
    • The joint and wheel velocities (9 variables: 5 for arm ˙θ, 4 for wheels u)
    • The timestep size ∆t (1 parameter)
    • The maximum joint and wheel velocity magnitude (1 parameter)

    Output:
    It produces the following outputs that describe the configuration of the robot one
    timestep (∆t) later:
    • The next state (configuration) of the robot (12 variables)
    array = [chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
    '''
    # Listing the dimensions of the chassis
    r = 0.0475
    l = 0.47 / 2
    w = 0.3 / 2
    z = 0.0963

    # Sorting input data
    cur_chassis_angle = Current_state[0]
    cur_chassis_pos = Current_state[1:3]
    cur_joint_angles = Current_state[3:8]
    cur_wheel_angles = Current_state[8:13]

    # Limiting joint velocities
    for i, jw_vel in enumerate(jw_vels):
        if abs(jw_vel) > max_jw_vels:
            jw_vels[i]= jw_vel/abs(jw_vel)*max_jw_vels # Preserving the signs
    vel_arm = jw_vels[:5]
    vel_wheels = jw_vels[5:]
    
    # Finding new joints and wheel angles
    new_joint_angles = cur_joint_angles + vel_arm * timestep
    new_wheel_angles = cur_wheel_angles + vel_wheels * timestep
    # Hence we got J1, J2, J3, J4, J5, W1, W2, W3, W4 of the next state

    # Finding new robot pose for finding (chassis angle, chassis x, chassis y)

    # Getting current transformation matrix of the body frame
    cur_pose = np.array([ [np.cos(cur_chassis_angle), - np.sin(cur_chassis_angle),0,cur_chassis_pos[0]],
                            [np.sin(cur_chassis_angle), np.cos(cur_chassis_angle),0,cur_chassis_pos[1]],
                            [0,0,1,z],
                            [0,0,0,1] ]) 
    
    # Finding change in wheel angles
    delta_thetas = new_wheel_angles - cur_wheel_angles
 
    # Finding Chassis body twist
    F = (r / 4 ) * np.array([ [-1 / (l+w), 1 / (l+w), 1 / (l+w), -1 / (l+w)],
                                [1,1,1,1],
                                [-1,1,-1,1]])

    V_b = np.matmul(F,delta_thetas)
    # Hence we got yaw and x,y components of the body twist
    # Converting this to vector of R^6
    V_b6 = np.concatenate((np.array([0,0]), V_b, np.array([0])))

    # Converting the body twist vector to se3
    rel_pose = expm(mr.VecTose3(V_b6.T))

    # Finding new pose
    new_pose = np.matmul(cur_pose,rel_pose)

    # Determining new chassis angle and position 
    new_chassis_angle = np.arccos(new_pose[0,0])
    new_chassis_pos = new_pose[:2,3]
    # Hence we got (chassis angle, chassis x, chassis y)
    
    
    # Finding the new state by combining all the results  
    New_state = np.hstack((new_chassis_angle,new_chassis_pos, new_joint_angles, new_wheel_angles))

    return New_state

# Testing the above function
if __name__ == "__main__":
    # Testing time and time of each timestep in secs
    timespan = 10
    timestep = 0.01

    # Defining arbitrary initial state
    Current_state = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
    # Defining the joint and wheel velocities for each step (constant velocities)
    jw_vels = np.array([0.1,0.1,0.1,0.1,0.1,1,2,2,1])
    # Defining the limit of the joint and wheel velocities
    max_jw_vels = 10

    # Initiating the list for appending the data at each iteration
    next_state_rows=[]
    next_state_rows.append(Current_state)
    for i in range(int(timespan//timestep)):
        Current_state = NextState(Current_state,jw_vels,timestep,max_jw_vels)
        next_state_rows.append(Current_state)

    # Converting the list into array and adding a column of zeros for the gripper state
    next_state_data = np.concatenate((np.array(next_state_rows),np.zeros((len(next_state_rows),1))),axis = 1)
    # Saving into the csv file
    np.savetxt("Nextstate_data.csv",next_state_data, delimiter=',')