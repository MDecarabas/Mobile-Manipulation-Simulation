ReadMe NextTask

The below read me file is for a the simulation of the Youbot using a different initial and goal location.
The inital configuration used is the below:

phi = 0
x = -1
y = 0.8
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

The code uses jointlimits explained in the main readme.
The controller used is a PI with FeedForward

The gains used are:
Kp_const = 0.7   
Ki_const = 0.01   

The results is positive. The robot is able to start at a different location and end at a different goal,
while still having the error converge towards zero.