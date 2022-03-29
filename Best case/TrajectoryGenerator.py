
'''
This code consists of a function called TrajectoryGenerator to create the reference
(desired) trajectory for the end-effector frame {e}. This trajectory consists of eight 
concatenated trajectory segments. Each trajectory segment begins and ends at rest. 
The data is saved as "Trajectory_data.csv" in the testing part of the code.
'''

import modern_robotics as mr
import numpy as np

def Trajectory_generator(T_se_initial,T_scube_initial,T_scube_final,T_ce_grasp,T_ce_standoff,k):
    '''
    This function generates the reference (desired) trajectory for the end-effector frame {e}.

    Inputs:
    The transformation matrices in the form of 4 x 4 numpy arrays as:
    • The initial configuration of the end-effector: Tse,initial
    • The initial configuration of the cube: Tsc,initial
    • The desired final configuration of the cube: Tsc,f inal
    • The configuration of the end-effector relative to the cube while grasping: Tce,grasp
    • The standoff configuration of the end-effector above the cube, before and after grasping, relative to the cube:
    Tce,standof f
    • The number of trajectory reference configurations per 0.01 seconds: k.

    Output:
    An N x 13 array of the desired trajectory for the end-effector frame {e} having each row as:
    [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state]
     

    '''
    # Generating initial and final transformation matrices for each step as below:
    # 1. Move the gripper from its initial configuration to a ”standoff” configuration a few cm above the block.
    # 2. Move the gripper down to the grasp position.
    # 3. Close the gripper2
    # 4. Move the gripper back up to the ”standoff” configuration.
    # 5. Move the gripper to a ”standoff” configuration above the final configuration.
    # 6. Move the gripper to the final configuration of the object.
    # 7. Open the gripper.
    # 8. Move the gripper back to the ”standoff” configuration.

    # Step 1
    Xstart1 = T_se_initial
    Xend1= np.matmul(T_scube_initial,T_ce_standoff)
    # Step 2
    Xstart2 = Xend1
    Xend2= np.matmul(T_scube_initial,T_ce_grasp)
    # Step 3
    Xstart3 = Xend2
    Xend3= Xend2
    # Step 4
    Xstart4 = Xend3
    Xend4 = Xstart2
    # Step 5
    Xstart5 = Xend4
    Xend5 = np.matmul(T_scube_final,T_ce_standoff)
    # Step 6
    Xstart6 = Xend5
    Xend6 = np.matmul(T_scube_final,T_ce_grasp)
    # Step 7
    Xstart7 = Xend6
    Xend7 = Xstart7 
    # Step 8
    Xstart8 = Xend7
    Xend8 = Xstart6

    # Definitions
    def get_Trajectory_rows(Trajectory_list,gripper_state,N):
        '''
        This function converts the list of transformation matrices to an array of data rows.
        Each row has data as: [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state]
        '''
        Trajectory_rows = np.zeros((N,13))
        for i in range(N):
            Transfromation_mat= Trajectory_list[i]
            Rotation = Transfromation_mat[0:3,0:3].flatten()
            Position = Transfromation_mat[0:3,3].flatten()
            Trajectory_rows[i] = np.concatenate((Rotation,Position,gripper_state), axis=None)
        return Trajectory_rows

    def append_step_data(Trajectory_step_data,Tf,N,method,gripper_state_int,Xstart,Xend):
        '''
        This function appends the Trajectory_step_data with trajectory data for each step.
        '''
        Trajectory_list = mr.ScrewTrajectory(Xstart,Xend,Tf,N,method)
        gripper_state= np.array([gripper_state_int])
        Trajectory_step_data.append(get_Trajectory_rows(Trajectory_list,gripper_state,N))
        return Trajectory_step_data


    # For 8 steps:

    Trajectory_step_data = []
    Tf = np.array([5,2,1,2,5,2,1,2]) # Stating time required of each step in secs 
    N = np.int32(Tf * k / 0.01) # Number of rows in the array of the Trajectory_data
    method = 3 # For screw trajectory

    # 1. Move the gripper from its initial configuration to a ”standoff” configuration a few cm above the block.
    gripper_state_int = 0
    Trajectory_step_data =append_step_data(Trajectory_step_data,Tf[0],N[0],method,gripper_state_int,Xstart1,Xend1)

    # 2. Move the gripper down to the grasp position.
    gripper_state_int = 0
    Trajectory_step_data =append_step_data(Trajectory_step_data,Tf[1],N[1],method,gripper_state_int,Xstart2,Xend2)

    # 3. Close the gripper2
    gripper_state_int = 1
    Trajectory_step_data =append_step_data(Trajectory_step_data,Tf[2],N[2],method,gripper_state_int,Xstart3,Xend3)

    # 4. Move the gripper back up to the ”standoff” configuration.
    gripper_state_int = 1
    Trajectory_step_data =append_step_data(Trajectory_step_data,Tf[3],N[3],method,gripper_state_int,Xstart4,Xend4)

    # 5. Move the gripper to a ”standoff” configuration above the final configuration.
    gripper_state_int = 1
    Trajectory_step_data =append_step_data(Trajectory_step_data,Tf[4],N[4],method,gripper_state_int,Xstart5,Xend5)

    # 6. Move the gripper to the final configuration of the object.
    gripper_state_int = 1
    Trajectory_step_data =append_step_data(Trajectory_step_data,Tf[5],N[5],method,gripper_state_int,Xstart6,Xend6)

    # 7. Open the gripper.
    gripper_state_int = 0
    Trajectory_step_data =append_step_data(Trajectory_step_data,Tf[6],N[6],method,gripper_state_int,Xstart7,Xend7)

    # 8. Move the gripper back to the ”standoff” configuration.
    gripper_state_int = 0
    Trajectory_step_data =append_step_data(Trajectory_step_data,Tf[7],N[7],method,gripper_state_int,Xstart8,Xend8)


    # Converting List data to array
    Trajectory_data = np.concatenate(Trajectory_step_data,axis=0)
    
    return Trajectory_data

# Testing the above function
if __name__ == "__main__":
        
    # Input Data

    # Initial Configuration of the end effector Tse,initial
    T_se_initial = np.array([[1, 0, 0, 0 ],
                            [0, 1, 0, 0 ],               
                            [0, 0, 1, 0.5 ],               
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

    k=1

    # Getting Trajectory data
    Trajectory_data = Trajectory_generator(T_se_initial,T_scube_initial,T_scube_final,T_ce_grasp,T_ce_standoff,k)
    # Saving in a csv file
    np.savetxt("TrajectoryGenerator_data.csv",Trajectory_data, delimiter=',')