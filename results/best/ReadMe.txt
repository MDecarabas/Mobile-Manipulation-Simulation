ReadMe Best

The below read me file is for the best convergence I was able to obtain for the errors.
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

The results as expected gently arrive at the location designated for pickup, and smoothly,
with no jitter take the cube to it designated goal as shown in the video. 