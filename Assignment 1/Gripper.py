## @file
#  @brief Gripper for UR3 Robot on Linear rails defined by standard DH parameters with 3D model
#  @author Long Thinh Le
#  @date Sep 1, 2023

from spatialmath import SE3
from GripperLEFT import GripperLEFT_LinearUR3 
from GripperRIGHT import GripperRIGHT_LinearUR3

import roboticstoolbox as rtb
from math import pi

import numpy as np
import time

import swift

class Gripper_LinearUR3():
    """ 
    Example usage:
    >>> from Gripper import Gripper_LinearUR3
    >>> import swift
    
    >>> r_gripper = Gripper_LinearUR3()
    >>> env = swift.Swift()
    >>> env.launch(realtime = True)
    >>> r_gripper.add_to_env(env)
    >>> r_gripper.move_gripper(pi/3,env)
    >>> r_gripper.move_gripper(pi/8,env)
    
    """
    
    def __init__(self):
        self.Gripper_Right = GripperRIGHT_LinearUR3()
        self.Gripper_Left = GripperLEFT_LinearUR3()
        
        # Open
        self.Gripper_Left.q = [0, 0, pi/4, -pi/4]
        self.Gripper_Right.q = [0, 0, -pi/4, pi/4]
        self.Gripper_Left_base_origin = self.Gripper_Left.base
        self.Gripper_Right_base_origin = self.Gripper_Right.base
        


    def add_to_env(self, env):
        """Add the left and right grippers to the given environment.

        This function adds the left and right grippers, represented by self.Gripper_Left
        and self.Gripper_Right, to the specified environment.

        Args:
            environment (object): The environment to which the grippers will be added.
        """
        self.Gripper_Left.add_to_env(env)
        self.Gripper_Right.add_to_env(env)


    def move_gripper(self, rad, env, drop_brick, brick_name, trans_matrix):
        """Move the gripper following the specified angle in radians.

        Args:
            radian_angle (float): The angle of the gripper in radians (open or close).
            environment (object): The environment where the gripper operates.
            drop_brick (bool): True if the gripper should drop the brick, False otherwise.
            brick_geometry (array_like): The position of a brick's geometry mesh.
            transformation_matrix (matrix): A transformation matrix for the brick's position.
        """
        q_goal = np.array([0 , 0, rad,-rad])
        
        qtraj_Left = rtb.jtraj(self.Gripper_Left.q, q_goal, 50).q
        qtraj_Right = rtb.jtraj(self.Gripper_Right.q, -1*q_goal, 50).q

        final_brick_pose = trans_matrix @ SE3.Rz(pi/2).A @ SE3.Trans(-0.03336,-0.0666,-0.3).A
        
        trans_z = (brick_name.T[2,3] - final_brick_pose[2,3]) / 50
        
        for i in range(50):
            if drop_brick:
                brick_name.T = brick_name.T @ SE3(0,0,-trans_z).A#final_brick_pose  # trans_matrix
            self.Gripper_Left.q = qtraj_Left[i]
            self.Gripper_Right.q = qtraj_Right[i]
            env.step(0.02)
    
    
    def transl_base(self, transl_mat, env):
        """Translate the base position of both grippers and update their positions in the environment.

        This function translates the base position of both the left and right grippers
        using the provided translation matrix. It also updates the gripper positions in
        the specified environment.

        Args:
            translation_matrix (matrix): The transformation matrix representing the desired translation.
            environment (object): The environment where the gripper positions will be updated.
        """
        # print('Previous base gripper: \n', self.Gripper_Left.base)
        
        self.Gripper_Left.base = self.Gripper_Left_base_origin * transl_mat * SE3.Ry(-pi/2) * SE3.Ry(pi) * SE3.Trans(0.16,0,0) *  SE3.Rx(pi)
        self.Gripper_Right.base = self.Gripper_Right_base_origin * transl_mat * SE3.Ry(-pi/2) * SE3.Ry(pi) * SE3.Trans(0.16,0,0) *  SE3.Rx(pi)
        
        # print('After base gripper: \n', self.Gripper_Left.base)
        self.update_gripper_pos()
        env.step(0.001)
        # print('Already updated!')
            
    def update_gripper_pos(self):
        """Update the 3D models of both left and right grippers.

        This function triggers the update of the 3D models representing the left and right grippers.
        It ensures that the visual representation of the grippers matches their current positions.

        Note:
            This function is typically used to synchronize the visual representation with the
            physical positions of the grippers in a graphical simulation.

        """
        self.Gripper_Left._update_3dmodel()
        self.Gripper_Right._update_3dmodel()
    
    def test(self):
        env = swift.Swift()
        env.launch(realtime= True)
        self.add_to_env(env)        
        while True:
            self.move_gripper(pi/3, env)
            self.move_gripper(pi/8, env)
            time.sleep(1)

        
        
        

if __name__ == "__main__":
    r = Gripper_LinearUR3()
    r.test()
    