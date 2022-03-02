
import numpy as np
import modern_robotics as mr

X_errInt = np.zeros(6)

def Feedforward(x, x_d, x_dNext, Kp, Ki, dt, X_errInt):

    ''' The Feedforward function serves the purpose of correcting your trajectory and
        allowing the user to apply a feedforward/P/PI controller

        The inputs required are: 
            x: Tse frame
            x_d: Desired Tse frame
            x_dNext: Desired next Tse frame
            Kp: 6x6 identity matrix multiplied by gain for proportional control
            Ki: 6x6 identity matrix multiplied by gain for integral control
            dt: time step
            X_errInt: error integral
    '''
    # The below steps are explained in modern robotics chapter 13.4 and 13.5
    X_err = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(x)@x_d))
    Vd = (1/dt)*mr.se3ToVec(mr.MatrixLog6(mr.TransInv(x_d)@x_dNext))
    print('\nVd\n',Vd)
    Ad = mr.Adjoint(mr.TransInv(x)@x_d)
    print('\nAd_tot\n', Ad@Vd)
    X_errInt = X_err*dt + X_errInt
    V = Ad@Vd +Kp@X_err+Ki@X_errInt
    print('\nV\n', V)
    return V, X_err, X_errInt

def jPInv(Tb0, Blist, M0e, config):

    ''' The JPInv function serves the purpose of calculating the Jacobian Pseudo Inverse

        The inputs required are: 
            Tb0 frame
            Blist: screw axes B for the five joints
            M0e: The end-effector frame e relative to the arm base frame 0 at the home configuration
            config: 12 array of confi
    '''

    l = 0.47/2  #Length
    w = 0.3/2   #width
    r = 0.0475 #Wheel Radius

    # The below steps are explained in modern robotics chapter 13.4 and 13.5

    T0e = mr.FKinBody(M0e, Blist, config[3:8])

    F6 =  r/4*np.array([[0, 0, 0, 0], 
                        [0, 0, 0, 0], 
                        [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [1,  1,  1, 1],
                        [-1, 1, -1, 1], 
                        [0, 0, 0, 0]])
    T0e_Inv = mr.TransInv(T0e)

    Tb0_Inv = mr.TransInv(Tb0)

    jbase = mr.Adjoint(T0e_Inv@Tb0_Inv)@F6

    jarm = mr.JacobianBody(Blist, config[3:8])

    #Joint Limits
    jarm = jarm.T

    if config[3] > 3.00 or config[3] < -3.00: #Limits for joint limit 1
        jarm[0] = jarm[0]*0

    if  config[4] > 2.5 or config[4] < -2.5: #Limits for joint limit 2
        jarm[1] = jarm[1]*0

    if config[5] > 2.55 or config[5] < -2.64: #Limits for joint limit 3
        jarm[2] = jarm[2]*0

    if config[6] > 2.8 or config[6] < -2.8: #Limits for joint limit 4
        jarm[3] = jarm[3]*0

    if config[7] > 2.8 or config[7] < -3: #Limits for joint limit 5
        jarm[4] = jarm[4]*0

    jarm = jarm.T

    je = np.hstack((jbase, jarm))

    print('\nJe\n', je)

    jpInv = np.linalg.pinv(je)

    return jpInv


# phi = 0
# x = 0
# y = 0
# theta1 = 0
# theta2 = 0
# theta3 = 0.2
# theta4 = -1.6
# theta5 = 0


# config = np.array([phi, x, y, theta1, theta2, theta3, theta4, theta5])


# x = np.array([[0.170, 0, 0.985, 0.387],
#               [0, 1, 0, 0],
#               [-0.985, 0, 0.170, 0.570],
#               [0, 0, 0, 1]])

# x_d = np.array([[0, 0, 1, 0.5],
#                 [0, 1, 0, 0],
#                 [-1, 0, 0, 0.5],
#                 [0, 0, 0, 1]])

# x_dNext = np.array([[0, 0, 1, 0.6],
#                     [0, 1, 0, 0],
#                     [-1, 0, 0, 0.3],
#                     [0, 0, 0, 1]])


# Kp_const =0
# Ki_const = 0
# Kp = np.identity(6)*Kp_const
# Ki = np.identity(6)*Ki_const
# dt = 0.01


# Tb0 = np.array([[1, 0, 0, 0.1662],
#                [0, 1, 0, 0],
#                [0, 0, 1, 0.0026],
#                [0, 0, 0, 1]])

# M0e = np.array([[1, 0, 0, 0.033],
#                 [0, 1, 0, 0],
#                 [0, 0, 1, 0.6546],
#                 [0, 0, 0, 1]])

# Blist = np.array([[0, 0, 1, 0, 0.033, 0],
#                   [0, -1, 0, -0.5076, 0, 0], 
#                   [0, -1, 0, -0.3526, 0, 0],
#                   [0, -1, 0, -0.2176, 0, 0], 
#                   [0, 0, 1, 0, 0, 0]]).T
# tta_config = config[3:8]
# jpInv = jPInv(Tb0, Blist, M0e, config)
# V, X_err, X_errInt = Feedforward(x, x_d, x_dNext, Kp, Ki, dt, X_errInt)
# f = jpInv@V
# print('\nf\n', f)