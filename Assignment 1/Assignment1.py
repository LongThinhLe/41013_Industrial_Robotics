from ir_support import UR5, UR3, LinearUR5

from math import pi
from spatialmath import SE3
from spatialmath.base import *
from LinearUR3 import LinearUR3
from environment_stuff import stuff_env
from bricks import Bricks

import time
import swift

import logging

# Configure logging
""" 
Logging for informative messages throughout the program.
This configuration sets the format and log level for info messages.
"""
fmt = "%(asctime)s %(levelname)8s: %(message)s"
logging.basicConfig(format=fmt, level=logging.INFO)
log = logging.getLogger(__name__)


# Create and launch a swift environment
env = swift.Swift()
env.launch(realtime= True)

################## ADDING ENVIRONMENT STUFF
env_stuff = stuff_env()
env_stuff.add_to_env(env)


################## BRICKS 
my_Bricks = Bricks()
my_Bricks.add_to_env(env)

################### FINAL POSITION
final_pose_bricks = my_Bricks.final_pose_bricks()

################# LINEAR UR3
# Create a UR3 on the linear rails used by the UR5Linear
r_UR3_linear = LinearUR3()
r_UR3_linear.add_to_env(env)
r_UR3_linear.add_gripper_to_env(env)

r_UR3_linear.set_q_goal(-0,0,-pi/2,0,0,0,0)
r_UR3_linear.gripper_transl_base(env)

# CENTER OF THE BRICK : X + 0.03336 | Y + 0.06663
# Offset width : 0.06671
# Offset length: 0.13343
# Offset height: 0.03336

# RUN HERE
# We move to the initial state for solving the best inverse kinematic

print('Caution ! Please stay away from the brown table 1 meter\n')

q_goal = r_UR3_linear.set_q_goal(0, 1.57, -2.3, -0.6, 1.6, 1.57, 0)

print('Caution ! Robot arm is moving toward in positive X direction\n')
time.sleep(1)

time.sleep(0.5)
r_UR3_linear.ur3_move_to(q_goal,env)

print('Now, the UR3 start to execute the mission\n')

# ANSI escape code for colorful text
RED_TEXT = '\033[91m'
BLUE_TEXT = '\033[94m'
GREEN_TEXT = '\033[92m'
YELLOW_TEXT = '\033[93m'
RESET_COLOR = '\033[0m'

for i in range (len(my_Bricks.bricks_position)):
    
    # STEP 1: Move the UR3 to a desired position
    q_goal = r_UR3_linear.filter_ikine(threshold= 2.0,
                                       final_pose= r_UR3_linear.center_of_brick_up(my_Bricks.poses_bricks[i].A),
                                       my_Bricks= my_Bricks, 
                                       bring_brick= False, 
                                       brick_name= my_Bricks.my_Bricks[i],
                                       env= env)
    # Calculate errors and log information
    state_1 = r_UR3_linear.center_of_brick_up(my_Bricks.poses_bricks[i].A) # desired position
    state_2 = r_UR3_linear.fkine(q_goal).A
    error = abs(state_2-state_1)
    log_message = f'\nThe {YELLOW_TEXT}UR3{RESET_COLOR} move to the {YELLOW_TEXT}position{RESET_COLOR}:\n {RED_TEXT}X = {round(r_UR3_linear.fkine(q_goal).A[1,3],4)}{RESET_COLOR}\n {GREEN_TEXT}Y = {round(r_UR3_linear.fkine(q_goal).A[2,3],4)}{RESET_COLOR}\n {BLUE_TEXT}Z = {round(r_UR3_linear.fkine(q_goal).A[3,3],4)}{RESET_COLOR}\n\nThe {YELLOW_TEXT}Error{RESET_COLOR} between {GREEN_TEXT}desired position{RESET_COLOR} and {RED_TEXT}actual position{RESET_COLOR}:\n {RED_TEXT}X = {round(error[1,3],10)} ({(round(error[1,3],10) * 100)/round(state_1[1,3],10)}%){RESET_COLOR}\n {GREEN_TEXT}Y = {round(error[2,3],10)} ({(round(error[2,3],10) * 100)/round(state_1[2,3],10)}%){RESET_COLOR}\n {BLUE_TEXT}Z = {round(error[3,3],10)} ({(round(error[3,3],10) * 100)/round(state_1[3,3],10)}%){RESET_COLOR}\n'
    log.info(log_message)
    r_UR3_linear.ur3_move_to(q_goal= q_goal,
                             env= env)


    # STEP 2: Move the UR3 to a desired position for the brick
    q_goal = r_UR3_linear.filter_ikine(threshold= 2.0, 
                                       final_pose= r_UR3_linear.center_of_brick_down(my_Bricks.poses_bricks[i].A),
                                       my_Bricks= my_Bricks,
                                       bring_brick= False,
                                       brick_name= my_Bricks.my_Bricks[i],
                                       env= env)
    # Calculate errors and log information
    state_1 = r_UR3_linear.center_of_brick_down(my_Bricks.poses_bricks[i].A) # desired position
    state_2 = r_UR3_linear.fkine(q_goal).A
    error = abs(state_2-state_1)
    log_message = f'\nThe {YELLOW_TEXT}UR3{RESET_COLOR} move to brick number {YELLOW_TEXT}{i+1}{RESET_COLOR} at the position: \n {RED_TEXT}X = {round(r_UR3_linear.fkine(q_goal).A[1,3],4)}{RESET_COLOR}\n {GREEN_TEXT}Y = {round(r_UR3_linear.fkine(q_goal).A[2,3],4)}{RESET_COLOR}\n {BLUE_TEXT}Z = {round(r_UR3_linear.fkine(q_goal).A[3,3],4)}{RESET_COLOR}\n\nThe {YELLOW_TEXT}Error{RESET_COLOR} between {GREEN_TEXT}desired position{RESET_COLOR} and {RED_TEXT}actual position{RESET_COLOR}:\n {RED_TEXT}X = {round(error[1,3],10)} ({(round(error[1,3],10) * 100)/round(state_1[1,3],10)}%){RESET_COLOR}\n {GREEN_TEXT}Y = {round(error[2,3],10)} ({(round(error[2,3],10) * 100)/round(state_1[2,3],10)}%){RESET_COLOR}\n {BLUE_TEXT}Z = {round(error[3,3],10)} ({(round(error[3,3],10) * 100)/round(state_1[3,3],10)}%){RESET_COLOR}\n'
    log.info(log_message)
    r_UR3_linear.ur3_move_to(q_goal= q_goal,
                             env= env)


    # STEP 3: Gripping the brick
    log_message = f'\nThe {YELLOW_TEXT}UR3{RESET_COLOR} grabs the brick number {YELLOW_TEXT}{i+1}{RESET_COLOR} at the position: \n {RED_TEXT}X = {round(r_UR3_linear.fkine(q_goal).A[1,3],4)}{RESET_COLOR}\n {GREEN_TEXT}Y = {round(r_UR3_linear.fkine(q_goal).A[2,3],4)}{RESET_COLOR}\n {BLUE_TEXT}Z = {round(r_UR3_linear.fkine(q_goal).A[3,3],4)}{RESET_COLOR}\n'
    log.info(log_message)
    # Close the gripper to grab the brick
    r_UR3_linear.gripper_close(drop_brick= False, 
                               brick_name= my_Bricks.my_Bricks[i], 
                               trans_matrix= my_Bricks.final_pose_bricks[i].A, 
                               env= env)


    # STEP 4: Move the brick to a new position while gripping
    q_goal = r_UR3_linear.filter_ikine(threshold= 2.0, 
                                       final_pose= r_UR3_linear.center_of_brick_up(my_Bricks.poses_bricks[i].A), 
                                       my_Bricks= my_Bricks,
                                       bring_brick= True,
                                       brick_name= my_Bricks.my_Bricks[i],
                                       env= env)
    # Calculate errors and log information
    state_1 = r_UR3_linear.center_of_brick_up(my_Bricks.poses_bricks[i].A) # desired position
    state_2 = r_UR3_linear.fkine(q_goal).A
    error = abs(state_2-state_1)
    log_message = f'\nThe {YELLOW_TEXT}UR3{RESET_COLOR} move the brick number {YELLOW_TEXT}{i+1}{RESET_COLOR} to the position: \n {RED_TEXT}X = {round(r_UR3_linear.fkine(q_goal).A[1,3],4)}{RESET_COLOR}\n {GREEN_TEXT}Y = {round(r_UR3_linear.fkine(q_goal).A[2,3],4)}{RESET_COLOR}\n {BLUE_TEXT}Z = {round(r_UR3_linear.fkine(q_goal).A[3,3],4)}{RESET_COLOR}\n\nThe {YELLOW_TEXT}Error{RESET_COLOR} between {GREEN_TEXT}desired position{RESET_COLOR} and {RED_TEXT}actual position{RESET_COLOR}:\n {RED_TEXT}X = {round(error[1,3],10)} ({(round(error[1,3],10) * 100)/round(state_1[1,3],10)}%){RESET_COLOR}\n {GREEN_TEXT}Y = {round(error[2,3],10)} ({(round(error[2,3],10) * 100)/round(state_1[2,3],10)}%){RESET_COLOR}\n {BLUE_TEXT}Z = {round(error[3,3],10)} ({(round(error[3,3],10) * 100)/round(state_1[3,3],10)}%){RESET_COLOR}\n'
    log.info(log_message)
    r_UR3_linear.ur3_move_and_grip(q_goal= q_goal,
                                   my_Bricks= my_Bricks, 
                                   brick_name= my_Bricks.my_Bricks[i], 
                                   env= env)


    # STEP 5: Place the brick at a new position while gripping and avoiding collision
    offset_z_brick = SE3.Trans(0,0,0.03336 + 0.015) # For preventing from crashing other bricks
    q_goal = r_UR3_linear.filter_ikine(threshold= 2.0, 
                                       final_pose= my_Bricks.final_pose_bricks[i].A * offset_z_brick,
                                       my_Bricks= my_Bricks,
                                       bring_brick= True, 
                                       brick_name=my_Bricks.my_Bricks[i] , 
                                       env= env)
    # Calculate errors and log information
    state_1 = my_Bricks.final_pose_bricks[i].A * offset_z_brick # desired position
    state_2 = r_UR3_linear.fkine(q_goal).A
    error = abs(state_2-state_1)
    log_message = f'\nThe {YELLOW_TEXT}UR3{RESET_COLOR} place the brick number {YELLOW_TEXT}{i+1}{RESET_COLOR} to the position: \n {RED_TEXT}X = {round(r_UR3_linear.fkine(q_goal).A[1,3],4)}{RESET_COLOR}\n {GREEN_TEXT}Y = {round(r_UR3_linear.fkine(q_goal).A[2,3],4)}{RESET_COLOR}\n {BLUE_TEXT}Z = {round(r_UR3_linear.fkine(q_goal).A[3,3],4)}{RESET_COLOR}\n\nThe {YELLOW_TEXT}Error{RESET_COLOR} between {GREEN_TEXT}desired position{RESET_COLOR} and {RED_TEXT}actual position{RESET_COLOR}:\n {RED_TEXT}X = {round(error[1,3],10)} ({(round(error[1,3],10) * 100)/round(state_1[1,3],10)}%){RESET_COLOR}\n {GREEN_TEXT}Y = {round(error[2,3],10)} ({(round(error[2,3],10) * 100)/round(state_1[2,3],10)}%){RESET_COLOR}\n {BLUE_TEXT}Z = {round(error[3,3],10)} ({(round(error[3,3],10) * 100)/round(state_1[3,3],10)}%){RESET_COLOR}\n'
    log.info(log_message)  
    # Move the brick to the new position while gripping and avoiding collision
    r_UR3_linear.ur3_move_and_grip(q_goal= q_goal,
                                   my_Bricks= my_Bricks,
                                   brick_name= my_Bricks.my_Bricks[i],
                                   env= env)

    # STEP 6: Drop the gripped brick to its final position
    log_message = f'\nThe {YELLOW_TEXT}UR3{RESET_COLOR} drop the brick number {YELLOW_TEXT}{i+1}{RESET_COLOR} to the position: \n {RED_TEXT}X = {round(my_Bricks.final_pose_bricks[i].A[1,3],4)}{RESET_COLOR}\n {GREEN_TEXT}Y = {round(my_Bricks.final_pose_bricks[i].A[2,3],4)}{RESET_COLOR}\n {BLUE_TEXT}Z = {round(my_Bricks.final_pose_bricks[i].A[3,3],4)}{RESET_COLOR}\n'
    log.info(log_message)
    r_UR3_linear.gripper_open(drop_brick= True,
                               brick_name= my_Bricks.my_Bricks[i],
                               trans_matrix= my_Bricks.final_pose_bricks[i].A, 
                               env= env)

print('\nThe UR3 completed the Pick and Place 9 bricks process ! \n')
env.hold()

################################################# END PROGRAM