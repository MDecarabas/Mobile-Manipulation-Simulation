import numpy as np
import modern_robotics as mr
from numpy import sin, cos, pi
from ms_code import NextState
from ms2_code import TrajectoryGenerator
from ms3_code import Feedforward, jPInv
import matplotlib.pyplot as plt
import logging

logging.basicConfig(filename = 'code.log', level = logging.INFO) #Logging
logging.info("Variable Initialization Begins")

Tse = np.array([[0, 0, 1, 0], #Initial Position of End-Effect
                [0, 1, 0, 0],
                [-1, 0, 0, 0.5],
                [0, 0, 0, 1]])  

Tsc_init = np.array([[1, 0, 0, 1], #Initial Position of Cube
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.025], 
                    [0, 0, 0, 1]])

Tsc_goal = np.array([[0, 1, 0, -1], #Goal Position of Cube
                    [-1, 0, 0,  0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])

Tce_standoff = np.array([[np.cos(-pi/2), 0, -np.sin(-pi/2), 0],
                        [0, 1, 0, 0], 
                        [np.sin(-pi/2), 0, np.cos(-pi/2), 0.5],
                        [0, 0, 0, 1]]) ##StandOff Position of End-Effector
phi = -pi/4

roty = np.array([[np.cos(phi), 0, -np.sin(phi), 0],
                [0, 1, 0, 0], 
                [np.sin(phi), 0, np.cos(phi), 0],
                [0, 0, 0, 1]])

Tce_grasp = np.array([[np.cos(-pi/2), 0, -np.sin(-pi/2), 0],
                      [0, 1, 0, 0], 
                      [np.sin(-pi/2), 0, np.cos(-pi/2), 0],
                      [0, 0, 0, 1]])@roty  #Grasp Position of End-Effect

k = 1 #The number of trajectory reference configurations per 0.01 seconds:

dt = 0.01 #Step Size

maxSpeed = 20


#Config
phi = 0
x = 0
y = 0
theta1 = 0
theta2 = pi/4
theta3 = -pi/2
theta4 = -pi/4
theta5 = 0
Wheel1 = 0
Wheel2 = 0
Wheel3= 0
Wheel4 = 0
gState = 0

Config = np.array([phi, x, y, theta1, theta2, theta3, theta4, theta5, Wheel1, Wheel2, Wheel3, Wheel4, gState])


#Gains
Kp_const = 0.7      
Ki_const = 0.01      
Kp = np.identity(6)*Kp_const
Ki = np.identity(6)*Ki_const

#Time step
dt = 0.01

# fixed offset from the chassis frame b to the base frame of the arm
Tb0 = np.array([[1, 0, 0, 0.1662],
               [0, 1, 0, 0],
               [0, 0, 1, 0.0026],
               [0, 0, 0, 1]])  

M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])

Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0], 
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0], 
                  [0, 0, 1, 0, 0, 0]]).T

X_errInt = np.zeros(6) #Initialization of error integral
x = Tse     

# Milestone Functions for robot simulation

x_err = []             #X error initalization

it = 0                 #Iteration number initalization
iteration_list = []    #Empty list

logging.info("Variable Initialization Done")


logging.info("generating traj from milestone 2")
traj = TrajectoryGenerator(Tse, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k) # Trajectory Generated from Ms2

iterations = traj.shape[0]  #Length of iterations

currConfig = Config        #Config renamed for simplicity

realConfig = []            #real config empty list

for i in range(iterations-1):           #For loop to follow trajectory
    logging.info("Looping through trajectory")

    realConfig.append(currConfig)

    T0e = mr.FKinBody(M0e, Blist, currConfig[3:8])  #T0e frame initialized

    Tsb = np. array([[cos(currConfig[0]), -sin(currConfig[0]), 0, currConfig[1]],
                     [sin(currConfig[0]),  cos(currConfig[0]), 0, currConfig[2]],
                     [0, 0, 1, 0.0963], 
                     [0,0, 0, 1]])                 #Tsb frame calculated
    
    logging.info("Initializing Variables to be used with milestone 3")

    x = Tsb@Tb0@T0e                                #Tse frame calculated

    x_dNext = np.array([[traj[i+1][0], traj[i+1][1], traj[i+1][2], traj[i+1][9]], 
                    [traj[i+1][3], traj[i+1][4], traj[i+1][5], traj[i+1][10]],
                    [traj[i+1][6], traj[i+1][7], traj[i+1][8], traj[i+1][11]],
                    [0, 0, 0, 1]])                #desired Tse next frame

    x_d = np.array([[traj[i][0], traj[i][1], traj[i][2], traj[i][9]], 
                        [traj[i][3], traj[i][4], traj[i][5], traj[i][10]],
                        [traj[i][6], traj[i][7], traj[i][8], traj[i][11]],
                        [0, 0, 0, 1]])           #desired Tse current frame

    V, X_err, X_errInt = Feedforward(x, x_d, x_dNext, Kp, Ki, dt, X_errInt) #Error and control theta speeds

    x_err.append(X_err)

    jpInv = jPInv(Tb0, Blist, M0e, currConfig)

    V_speed = jpInv@V

    V_speed = np.array([V_speed[4], V_speed[5], V_speed[6], V_speed[7], V_speed[8], V_speed[0], V_speed[1], V_speed[2], V_speed[3]])

    logging.info("Milestone 3 steps finished executing")

    currConfig = NextState(currConfig, V_speed, dt, maxSpeed)   #Next state calculated

    currConfig = np.array([currConfig[0], currConfig[1], currConfig[2], currConfig[3], currConfig[4], currConfig[5], currConfig[6], currConfig[7], currConfig[8], currConfig[9], currConfig[10], currConfig[11], traj[i][12]])

    logging.info("Milestone 1 done")

    iteration_list.append(it +1)
    it = it+1
    

np.savetxt('SimTraj.csv', realConfig, delimiter=',')
time  = np.multiply(iteration_list,dt)
plt.plot(time, x_err)
plt.legend(['Twist error 1', 'Twist error 2', 'Twist error 3', 'Twist error 4', 'Twist error 5', 'Twist error 6'])
plt.title('Twist Error vs Time')
plt.ylabel('Twist Error')
plt.xlabel('Time')
plt.show()