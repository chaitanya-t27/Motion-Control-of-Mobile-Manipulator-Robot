'''
This code defines the funciton FeedbackControl and tests the function in the later part.
'''

import modern_robotics as mr
import numpy as np


def FeedbackControl(X,Xd,Xdnext,Kp,Ki,timestep,thetalist,Xerr_integral):
    '''
    The function calculates the task-space feedforward plus feedback control law.

    Inputs:
    • The current actual end-effector configuration X (i.e. Tse)
    • The current reference end-effector configuration Xd (i.e. Tse,d)
    • The reference end-effector configuration at the next timestep, Xdnext (aka Tse,d,next)
    • The PI gain matrices Kp and Ki
    • The timestep ∆t between reference trajectory configurations
    • The joint angles of the manipulator : thetalist
    • The integral of error in the end-effector configuration over the time :Xerr_integral

    Outputs:Je, , Xerr, Xerr_integral
    • The desired body twist Vd
    • The commanded end-effector twist V expressed in the end-effector frame {e}
    • The mobile manipulator Jacobian Je(θ): Je
    • the wheel and joints velocities (u, θdot) :u_theta_dot
    • The end effector configuration error Xerr
    • The integral of end effector configuration error Xerr_integral

    '''

    # Setting the environment
    Blist = np.array([  [0,0,1,0,0.033,0],
                        [0,-1,0,-0.5076,0,0],
                        [0,-1,0,-0.3526,0,0],
                        [0,-1,0,-0.2176,0,0],
                        [0,0,1,0,0,0]   ]).T

    # Listing the dimensions of the chassis
    r = 0.0475
    l = 0.47 / 2
    w = 0.3 / 2
    F = (r / 4 ) * np.array([ [-1 / (l+w), 1 / (l+w), 1 / (l+w), -1 / (l+w)],
                                [1,1,1,1],
                                [-1,1,-1,1]])
    F6 = np.concatenate((np.zeros((2,4)),F,np.zeros((1,4))),axis=0)

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

    # Finding transformation matrix of the end effector in the chassis frame at zero configuration
    T0e = mr.FKinBody(M0e,Blist,thetalist)
    Tbe = np.matmul(Tb0,T0e)
    Teb = mr.TransInv(Tbe)

    # Finding the mobile manipulator jacobian Je
    J_base = np.matmul(mr.Adjoint(Teb),F6)
    J_arm = mr.JacobianBody(Blist, thetalist)
    Je = np.concatenate((J_base,J_arm),axis =1)

    psInv = np.linalg.pinv(Je,1e-2) # For calling into the wrapper script
    # psInv = np.linalg.pinv(Je) # For testing the code

    # Calculating end effector configuration error Xerr and Xerr_integral
    Xerr_bracket = mr.MatrixLog6(np.matmul(mr.TransInv(X),Xd))
    Xerr = mr.se3ToVec(Xerr_bracket)
    Xerr_integral = Xerr_integral + timestep * Xerr

    # Finding the desired body twist Vd
    Vd_bracket = (1 / timestep) * mr.MatrixLog6(np.matmul(np.linalg.inv(Xd),Xdnext))
    Vd = mr.se3ToVec(Vd_bracket)

    # Evaluating the commanded end-effector twist V expressed in the end-effector frame {e}
    Adj = mr.Adjoint(np.matmul(np.linalg.inv(X),Xd))
    Feedforward = np.matmul(Adj,Vd)
    V = Feedforward + np.matmul( Kp , Xerr) + np.matmul(Ki, Xerr_integral)

    # Hence the wheel and joint velocities are:
    u_theta_dot = np.matmul(psInv, V)
    return Vd, V, Je, u_theta_dot, Xerr, Xerr_integral

# Testing the above function

if __name__ == "__main__":
    # Loading given input data
    thetalist = np.array([0, 0, 0.2, -1.6, 0])
    X = np.array([  [0.170,0,0.985,0.387],
                    [0,1,0,0],
                    [-0.985,0,0.170,0.570],
                    [0,0,0,1]])
    Xd = np.array([ [0,0,1,0.5],
                    [0,1,0,0],
                    [-1,0,0,0.5],
                    [0,0,0,1]])
    Xdnext = np.array([ [0,0,1,0.6],
                        [0,1,0,0],
                        [-1,0,0,0.3],
                        [0,0,0,1]])   
    timestep = 0.01
    Xerr_integral = 0
    Kp = np.eye(6, dtype=int)
    Ki = np.zeros((6,6))
    
    # Generating results
    Vd, V, Je, u_theta_dot, Xerr, Xerr_integral = FeedbackControl(X,Xd,Xdnext,Kp,Ki,timestep,thetalist,Xerr_integral)
    print('Vd = ',Vd)
    print('V = ',V)
    print('Je = ',Je)
    print('u_theta_dot = ',u_theta_dot)
    print('Xerr = ',Xerr)
    print('Xerr_integral = ',Xerr_integral)
    