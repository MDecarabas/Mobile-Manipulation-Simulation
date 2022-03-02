ReadMe Overshoot

The below read me file is for the overshoot convergence I was able to obtain for the errors.
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
Kp_const = 4  
Ki_const = 2

The results as expected, the robot goes slightly past the desired location designated for pickup, then readjusts,
proccedes to grab the cube and go to the desired location.
