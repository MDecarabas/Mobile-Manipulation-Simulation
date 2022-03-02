import numpy as np
from numpy import pi
import modern_robotics as mr

# Example input code used to generate trajectory and turn it into CSV file

# Initial Inputs

# Tse_init = np.array([[0, 0, 1, 0.033],
#                         [0, 1, 0, 0], 
#                         [-1, 0, 0, 0.6546],
#                         [0, 0, 0, 1]]) 

# Tsc_init = np.array([[1, 0, 0, 1],
#                         [0, 1, 0, 0],
#                         [0, 0, 1, 0.025], 
#                         [0, 0, 0, 1]])

# Tsc_goal = np.array([[0, 1, 0, 0],
#                         [-1, 0, 0, -1],
#                         [0, 0, 1, 0.025],
#                         [0, 0, 0, 1]])

# Tce_standoff = np.array([[np.cos(-pi/2), 0, -np.sin(-pi/2), 0],
#                             [0, 1, 0, 0], 
#                             [np.sin(-pi/2), 0, np.cos(-pi/2), 0.5],
#                             [0, 0, 0, 1]])

# Tce_grasp = np.array([[np.cos(-pi/2), 0, -np.sin(-pi/2), 0],
#                             [0, 1, 0, 0], 
#                             [np.sin(-pi/2), 0, np.cos(-pi/2), 0],
#                             [0, 0, 0, 1]])

# k = 1


def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k):

    ''' The TrajectoryGenerator function takes as input:

        1) The initial configuration of the end-effector in the reference trajectory

        2) The cube's initial configuration

        3) The cube's desired final configuration

        4) The end-effector's configuration relative to the cube when it is grasping the cube

        5) The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube

        6) The number of trajectory reference configurations per 0.01 seconds

        The resulting outputs are a N 4x4 Trajectory frames, and a gripper state array of length N
    '''

    Tf = 4
    N = (Tf*k)/0.01
    method = 5

#Step 1:  A trajectory to move the gripper from its initial configuration 
#         to a "standoff" configuration a few cm above the block.
    Tse_standoff = Tsc_init@Tce_standoff
    traj = mr.ScrewTrajectory(Tse_init, Tse_standoff, Tf, N, method)
    gripper_state = np.zeros(len(traj)) 
    
# 2) A trajectory to move the gripper down to the grasp position.
    Tse_grasp = Tsc_init@Tce_grasp
    traj2 = mr.CartesianTrajectory(Tse_standoff, Tse_grasp, Tf, N, method)
    traj = np.vstack([traj, traj2])
    gripper_state2 = np.zeros(len(traj2))
    gripper_state = np.hstack([gripper_state, gripper_state2])

# 3) Closing of the gripper.
    traj2 = mr.ScrewTrajectory(Tse_grasp, Tse_grasp, Tf, N/2, method)
    traj = np.vstack([traj, traj2])
    gripper_state2 = np.ones(len(traj2))
    gripper_state = np.hstack([gripper_state, gripper_state2])

# 4) A trajectory to move the gripper back up to the "standoff" configuration.
    traj2 = mr.ScrewTrajectory(Tse_grasp, Tse_standoff,Tf, N, method)
    traj = np.vstack([traj, traj2])
    gripper_state2 = np.ones(len(traj2))
    gripper_state = np.hstack([gripper_state, gripper_state2])

# 5) A trajectory to move the gripper to a "standoff" configuration 
#    above the final configuration.
    Tse_standoffGoal = Tsc_goal@Tce_standoff
    traj2 = mr.ScrewTrajectory(Tse_standoff, Tse_standoffGoal, Tf, N, method)
    traj = np.vstack([traj, traj2])
    gripper_state2 = np.ones(len(traj2))
    gripper_state = np.hstack([gripper_state, gripper_state2])

# 6) A trajectory to move the gripper to the final configuration of the object.
    Tse_letgo = Tsc_goal@Tce_grasp
    traj2 = mr.ScrewTrajectory(Tse_standoffGoal, Tse_letgo,Tf, N, method)
    traj = np.vstack([traj, traj2])
    gripper_state2 = np.ones(len(traj2))
    gripper_state = np.hstack([gripper_state, gripper_state2])

# 7) Opening of the gripper.
    traj2 = mr.ScrewTrajectory(Tse_letgo, Tse_letgo,Tf, N/2, method)
    traj = np.vstack([traj, traj2])
    gripper_state2 = np.zeros(len(traj2))
    gripper_state = np.hstack([gripper_state, gripper_state2])

# 8) A trajectory to move the gripper back to the "standoff" configuration.
    traj2 = mr.ScrewTrajectory(Tse_letgo, Tse_standoffGoal, Tf, N, method)
    traj = np.vstack([traj, traj2])
    gripper_state2 = np.zeros(len(traj2))
    gripper_state = np.hstack([gripper_state, gripper_state2])

    traj_array = traj2csv(traj, gripper_state)

    return traj_array



def traj2csv(traj, gripper_state):

    '''The traj2csv function takes N 4x4 trajectory arrays and N gripper states,
       these then get flattened out so they can be easily converted into a csv file.
    '''

    csv_traj = np.zeros((len(traj),13))
    for i in range(len(traj)):

        csv_traj[i][0] = traj[i][0][0]
        csv_traj[i][1] = traj[i][0][1]
        csv_traj[i][2] = traj[i][0][2]
        csv_traj[i][3] = traj[i][1][0]
        csv_traj[i][4] = traj[i][1][1]
        csv_traj[i][5] = traj[i][1][2]
        csv_traj[i][6] = traj[i][2][0]
        csv_traj[i][7] = traj[i][2][1]
        csv_traj[i][8] = traj[i][2][2]
        csv_traj[i][9] = traj[i][0][3]
        csv_traj[i][10] = traj[i][1][3]
        csv_traj[i][11] = traj[i][2][3]
        csv_traj[i][12] = gripper_state[i]

    return csv_traj

# Call function for TrajectoryGenerator which returns a trajectory and gripper state set of arrays

# traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_goal, Tce_grasp, Tce_standoff, k)

# # Call function for traj2csv which returns a N lines of 13 collumns with data to populate CopeliaSim scene

# # csv_traj = traj2csv(traj, gripper_state)

# # Argument that saves csv_traj data in a csv format

# np.savetxt('milestone2_csv.csv', traj, delimiter=',')