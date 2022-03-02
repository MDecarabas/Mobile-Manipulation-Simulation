import numpy as np
from numpy import sin, cos
import modern_robotics as mr

def NextState(currentConfig, jointControl, dt, maxSpeed):
    '''The NextState function provides the next configuration for the Youbot

       The currentConfig represents the 12 array of the Youbot joints
       jointControl: JointVelocities for arm and wheels
       dt: timestep
       maxSpeed: Top speed of velocities  

    '''
# Initialization

    r = 0.0475 #Wheel Radius
    l = 0.235  #Length
    w = 0.15   #width

    chassisConfig = currentConfig[0:3] #Chassis Configuration Components
    armConfig = currentConfig[3:8]     #Arm Configuration Components
    wheelConfig = currentConfig[8:12]  #Wheel Configuration Components


# Change velocities of components to maintain robot under maximum velocity

    for i in range(len(jointControl)):
        if jointControl[i] > maxSpeed:
            jointControl[i] = maxSpeed
        elif jointControl[i] < -maxSpeed:
            jointControl[i] = -maxSpeed
    # print(jointControl)

    armControl = jointControl[0:5]      #Arm Joint Velocities
    wheelControl = jointControl[5:9]    #Wheel Joint Velocities
    wheelControl = wheelControl.T

#Odometry of Chassis

    F = r/4*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                      [1,  1,  1, 1],
                      [-1, 1, -1, 1]])

    dTheta = wheelControl*dt

    Vb = F@dTheta
    wbz=  Vb[0]
    vbx = Vb[1]
    vby = Vb[2]


    if wbz == 0:
        dqb = np.array([0, vbx, vby])
    else:
        dqb = np.array([wbz, (vbx*sin(wbz)+vby*(cos(wbz)-1))/wbz, (vby*sin(wbz)+vbx*(1-cos(wbz)))/wbz])

    phi = currentConfig[0]

    T = np.array([[1, 0, 0],
                  [0, cos(phi), -sin(phi)],
                  [0, sin(phi), cos(phi)]])

    dq = T@dqb

    newChassisConfig = chassisConfig + dq
    
# New Arm Angles

    newArmConfig = armConfig + armControl*dt

# New Wheel Angles 

    newWheelConfig = wheelConfig + wheelControl*dt

# Reassembly of Components

    newConfig = np.concatenate((newChassisConfig, newArmConfig, newWheelConfig), axis=None)
    newConfig = list(newConfig)
    
    return newConfig

# #Initial Wheel Configuration
# w1 = 0
# w2 = 0
# w3 = 0
# w4 = 0

# #Initial Arm Configuration
# a1 = 0
# a2 = 0
# a3 = 0
# a4 = 0
# a5 = 0

# #Initial Chassis Configuration
# c1 = 0
# c2 = 0
# c3 = 0

# #Step Size
# dt = 0.01

# #Max Speed
# maxSpeed = 10

# #Assembled Initial Configuration Array
# CurrentConfig = np.array([c1, c2, c3, a1, a2, a3, a4, a5, w1,w2, w3, w4])

# #Initial Joint Velocities Array
# jointControl = np.array([0, 0, 0, 0, 0, 10, -20, 10, -20])
# print(jointControl[0:5])
# print(jointControl[5:8])

# time = 20
# iterations = int(time/dt)

# realConfig = np.empty((iterations, 12))

# for i in range(500):
#     CurrentConfig = NextState(CurrentConfig, jointControl, dt, maxSpeed)
#     realConfig[i] = CurrentConfig  

# np.savetxt('iPray.csv', realConfig, delimiter=',')
