## @file
#  @brief UR3 Robot on Linear rails defined by standard DH parameters with 3D model
#  @author Long Thinh Le
#  @date Sep 1, 2023

import swift
import roboticstoolbox as rtb
from roboticstoolbox import jtraj
import spatialmath.base as spb
from spatialmath import SE3
from spatialmath.base import *
from ir_support.robots.DHRobot3D import DHRobot3D
import os
import matplotlib.pyplot as plt

from math import pi

from Gripper import Gripper_LinearUR3
from bricks import Bricks

class LinearUR3(DHRobot3D):
    """ 
    Example usage:
    >>> from LinearUR3 import LinearUR3
    >>> import swift
    
    >>> r = LinearUR3()
    >>> q = [0, -pi/2, 0, 0, 0, 0, 0]
    >>> r.q = q
    >>> q_goal = [r.q[i] - pi/4 for i in range(r.n)]
    >>> env = swift.Swift()
    >>> env.launch(realtime = True)
    >>> r.add_to_env(env)
    >>> qtraj = rtb.jtraj(r.q, q_goal, 50).q
    >>> for q in qtraj:
    >>>     r.q = q
    >>>     env.step(0.02)

    """
    def __init__(self):
        # DH links
        links = self._create_DH()
        
        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'base_rail', color0 = (0.2,0.2,0.2,1),      # color option only takes effect for stl file
                            link1 = 'slider_rail', color1 = (0.1,0.1,0.1,1),
                            link2 = 'shoulder_ur3', 
                            link3 = 'upperarm_ur3',
                            link4 = 'forearm_ur3',
                            link5 = 'wrist1_ur3dae',
                            link6 = 'wrist2_ur3',
                            link7 = 'wrist3_ur3')
        
        # A joint config and the 3D object transforms to match that config
        qtest = [0, 0, -pi/2, 0, 0, 0, 0] # CHANGE NUMBER OF JOINTS HERE
        
        # WARNING HERE !!! THE SWIFT COORDINATE IS Z,X,Y = X,Y,Z = RED, GREEN, BLUE
        qtest_transforms = [spb.transl(0,0,0),
                            spb.trotx(-pi/2),
                            spb.transl(0,0,0.15239) @ spb.trotz(pi)                                      @ spb.transl(0,-0.1577,0) @ spb.trotx(pi/2) @ spb.transl(0,-0.155,0),
                            spb.transl(0,-0.12,0.1524) @ spb.trotz(pi)                                   @ spb.transl(0,-0.2778,0) @ spb.trotx(pi/2) @ spb.transl(0,-0.0352,0), 
                            spb.transl(0,-0.027115,0.39583) @ spb.trotz(pi)                              @ spb.transl(0,-0.4285,0) @ spb.trotx(pi/2) @ spb.transl(0,-0.3711,0), 
                            spb.transl(0,-0.027316,0.60903) @ spb.rpy2tr(0,-pi/2,pi, order = 'xyz')      @ spb.transl(0,-0.64149,0) @ spb.trotz(pi/2) @ spb.transl(0,-0.5843,0),
                            spb.transl(0.000389,-0.11253,0.60902) @ spb.rpy2tr(0,-pi/2,pi, order= 'xyz') @ spb.transl(0,-0.7275,0) @ spb.trotz(pi/2) @ spb.transl(0,-0.5007,0),
                            spb.transl(-0.0835,-0.11333,0.61096) @ spb.trotz(pi)                         @ spb.transl(0,-0.73055,0) @ spb.trotx(pi/2) @ spb.transl(0,-0.5027,0)]
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'LinearUR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2) * SE3(0,0.5,0)
        self.q = qtest
        
        # Create a Gripper and Brick object using Gripper_LinearUR3() class and Bricks() class
        self.r_UR3_gripper = Gripper_LinearUR3()
        self.my_Bricks = Bricks()
        
            

        
    def _create_DH(self):
        """ 
        Create Robot's standard DH model
        """
        links = [rtb.PrismaticDH(theta= pi, a= 0, alpha= pi/2, qlim= [-0.8, 0])]    # Prismatic Link
        
        a = [0, -0.24365, -0.21325, 0, -0.0007, 0] # joint 1,2,3,4,5,6 # CHANGE NUMBER OF JOINTS HERE     a = [0, -0.24365, -0.21325, 0, -0.0007, 0, 0, 0]
        d = [0.158, 0, 0, -0.1081, -0.083, 0.0819] # CHANGE NUMBER OF JOINTS HERE       d = [0.158, 0, 0, -0.1081, -0.083, 0.0819, 0.5, 0]
        
        alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]     # CHANGE NUMBER OF JOINTS HERE            alpha = [pi/2, 0, 0, pi/2, pi/2, 0, 0, 0] 
        qlim = [[-2*pi, 2*pi] for _ in range(6)]    # CHANGE NUMBER OF JOINTS HERE
        
        """ 
        WARNING !!!
        q0 : -0.8 < x < 0           # 
        q1 : -2pi < x < 2pi         # 0    
            
        q2 : -pi - 0.2 (-3.34) < x < 0.2           # 1  middle rad: -pi/2 = -1.57
        q3 : -pi/1.3 (-2.41) < x < pi/1.2 (2.61)  # 2     ( default q4,5,6 ) middle rad: 0.1
        
        q4 : -2pi < x < 2pi         # 3
        q5 : -2pi < x < 2pi         # 4
        q6 : -2pi < x < 2pi         # 5
        """
        for i in range(6):                          # CHANGE NUMBER OF JOINTS HERE
            link = rtb.RevoluteDH(d= d[i], a=a[i], alpha= alpha[i], qlim= qlim[i])
            links.append(link)
        return links
    
    
    def add_gripper_to_env(self,env):
        """Add the UR3 gripper to the specified environment.

        This function adds the UR3 gripper, represented by the 'r_UR3_gripper' attribute,
        to the provided environment.

        Args:
            environment (object): The environment to which the UR3 gripper will be added.
        """
        self.r_UR3_gripper.add_to_env(env)
    
    def add_bricks_to_env(self,env):
        """Add bricks to the specified environment.

        This function adds the bricks stored in the 'my_Bricks' attribute to the provided environment.

        Args:
            environment (object): The environment to which the bricks will be added.
        """
        self.my_Bricks.add_to_env(env)
    
    def set_q_goal(self, q_goal0, q_goal1, q_goal2, q_goal3, q_goal4, q_goal5, q_goal6):
        """Set joint angle goals for a robotic arm.

        This function sets the joint angle goals for a robotic arm by updating the elements
        of the 'q_goal' list. The goals are specified individually for each joint.

        Args:
            q_goal0 (float): Goal joint angle for joint 0.
            q_goal1 (float): Goal joint angle for joint 1.
            q_goal2 (float): Goal joint angle for joint 2.
            q_goal3 (float): Goal joint angle for joint 3.
            q_goal4 (float): Goal joint angle for joint 4.
            q_goal5 (float): Goal joint angle for joint 5.
            q_goal6 (float): Goal joint angle for joint 6.

        Returns:
            list: A list containing the updated joint angle goals.
        """
        q_goal = [self.q[i] for i in range(self.n)]
        q_goal[0] = q_goal0
        q_goal[1] = q_goal1
        q_goal[2] = q_goal2
        q_goal[3] = q_goal3
        q_goal[4] = q_goal4
        q_goal[5] = q_goal5
        q_goal[6] = q_goal6
        return q_goal
    
    def ur3_move_to(self, q_goal, env):
        """Move a UR3 robotic arm to a specified joint configuration.

        This function plans and executes a trajectory to move a UR3 robotic arm from its current
        joint configuration to the specified joint configuration 'q_goal'. The arm's position
        is updated in the provided environment during the trajectory execution.

        Args:
            q_goal (list): The target joint configuration to reach.
            environment (object): The environment where the arm's position will be updated.
        """
        
        # plt.close(1)
        # steps = 50
        # current_q = self.q
        # q_matrix = jtraj(current_q, q_goal, steps).q
        # fig = plt.figure(1)
        # fig = self.plot(q_goal, fig = fig)
        # ax = plt.gca()
        
        # for q in q_matrix:
        #     self.q = q
        #     ee_pos = transl(self.fkine(q).A)
        #     ax.plot(ee_pos[0],ee_pos[1], ee_pos[2], color = 'red', marker = '.', markersize = 3)
        #     fig.step(0.01)
        # plt.show()
        # self.q = current_q
        
        
        qtraj = rtb.jtraj(self.q, q_goal, 30).q
        for q in qtraj:
            self.q = q
            fkine_ur3_mat = self.fkine(q).A
            self.r_UR3_gripper.transl_base(fkine_ur3_mat, env)
            env.step(0.005)
    
    def ur3_move_and_grip(self, q_goal, my_Bricks, brick_name, env):
        """Move a UR3 robotic arm to a specified joint configuration and grip a brick.

        This function plans and executes a trajectory to move a UR3 robotic arm from its current
        joint configuration to the specified joint configuration 'q_goal'. During the trajectory,
        it also moves a brick represented by 'brick_name' based on the arm's position and grips
        the brick using the UR3 gripper.

        Args:
            q_goal (list): The target joint configuration to reach.
            my_Bricks (object): An object containing bricks and related functions.
            brick_name (matrix): The position or transformation matrix of the brick to be moved.
            environment (object): The environment where the arm's position and the brick's position
                                will be updated.
        """
        qtraj = rtb.jtraj(self.q, q_goal, 30).q
        for q in qtraj:
            self.q = q
            fkine_ur3_mat = self.fkine(q).A
            my_Bricks.move_brick_class(brick_name, fkine_ur3_mat,env)
            self.r_UR3_gripper.transl_base(fkine_ur3_mat, env)
            env.step(0.005)
    
    def gripper_close(self, drop_brick, brick_name, trans_matrix, env):
        """Close the gripper to grab a brick.

        This function closes the gripper of a UR3 robotic arm to grab a brick. It may also drop
        the brick depending on the 'drop_brick' flag. The brick's position and transformation
        matrix are considered for the gripper operation, and the environment is updated accordingly.

        Args:
            drop_brick (boolean): True to drop the brick, False to hold the brick.
            brick_name (matrix): The position or transformation matrix of the brick.
            trans_matrix (matrix): A transformation matrix for the brick.
            environment (object): The environment where the gripper and brick positions will be updated.
        """
        self.r_UR3_gripper.move_gripper(pi/6.5, env, drop_brick, brick_name, trans_matrix) # Grab
    
    def gripper_open(self, drop_brick, brick_name, trans_matrix, env):
        """Open the gripper to release a brick.

        This function opens the gripper of a UR3 robotic arm to release a brick. It may also drop
        the brick depending on the 'drop_brick' flag. The brick's position and transformation
        matrix are considered for the gripper operation, and the environment is updated accordingly.

        Args:
            drop_brick (boolean): True to drop the brick, False to hold the brick.
            brick_name (matrix): The position or transformation matrix of the brick.
            trans_matrix (matrix): A transformation matrix for the brick.
            environment (object): The environment where the gripper and brick positions will be updated.
        """
        self.r_UR3_gripper.move_gripper(pi/4, env, drop_brick, brick_name, trans_matrix) # Open
    
    def center_of_brick_up(self, brick_pose_se3):
        """Calculate the center of the top face of a brick in its local coordinates.

        This function computes the position of the center of the top face of a brick
        in its local coordinate system based on the provided brick pose.

        Args:
            brick_pose_se3 (matrix): The pose or transformation matrix of the brick.

        Returns:
            matrix: The position of the center of the top face of the brick in its local coordinates.
        """
        return brick_pose_se3 * SE3([0.0334, 0.0666,0.43]) * SE3.Rz(pi/2)
    
    def center_of_brick_down(self, brick_pose_se3):
        """Calculate the center of the bottom face of a brick in its local coordinates.

        This function computes the position of the center of the bottom face of a brick
        in its local coordinate system based on the provided brick pose.

        Args:
            brick_pose_se3 (matrix): The pose or transformation matrix of the brick.

        Returns:
            matrix: The position of the center of the bottom face of the brick in its local coordinates.
        """
        return brick_pose_se3 * SE3([0.0334, 0.0666,0.3]) * SE3.Rz(pi/2)
    
    def gripper_transl_base(self, env):
        """Update the position of the UR3 gripper in the environment.

        This function calculates the forward kinematics of the UR3 robotic arm
        to obtain its current pose and then updates the position of the UR3 gripper
        in the provided environment accordingly.

        Args:
            environment (object): The environment where the gripper's position will be updated.
        """
        fkine_ur3_mat = self.fkine(self.q).A
        self.r_UR3_gripper.transl_base(fkine_ur3_mat,env)
    
    def filter_ikine(self, threshold, final_pose, my_Bricks, bring_brick, brick_name, env):
        """Filter and adjust an IK solution to achieve a desired pose.

        This function filters and adjusts an inverse kinematics (IK) solution to achieve a desired
        final pose. It iteratively refines the IK solution while considering specified thresholds
        and constraints. The function can be used to bring a brick to a desired pose or move the
        UR3 arm to a target pose.

        Args:
            threshold (float): The threshold for considering an IK solution as valid.
            final_pose (matrix): The desired final pose to achieve.
            my_Bricks (object): An object containing bricks and related functions.
            bring_brick (boolean): True to bring a brick, False to move the arm to a pose.
            brick_name (matrix): The position or transformation matrix of the brick.
            environment (object): The environment where the arm and brick positions will be updated.

        Returns:
            list: The refined joint angle goals to achieve the desired pose.
        """
        flag = True
        change_q0_guess = False
        initial_q_guess = self.q
        a = pi/10
        b = 0.05

        while flag:

            q_goal = self.ikine_LM(final_pose, initial_q_guess, mask = [1,1,1,1,1,1]).q
            # print('Generate q_goal : \n', q_goal)
            # print('Initial_q_guess = \n', initial_q_guess)
            for _ in range(q_goal.size):
                if abs((q_goal[2]  - initial_q_guess[2])) > threshold or abs((q_goal[3]  - initial_q_guess[3])) > threshold or abs((q_goal[4]  - initial_q_guess[4])) > threshold or q_goal[0] > 0.02 or q_goal[0] < -0.83:
                    change_q0_guess = True
                else: change_q0_guess = False
                
            if change_q0_guess == False: 
                flag = False              
            else:           
                if bring_brick:  self.ur3_move_and_grip(initial_q_guess, my_Bricks , brick_name, env) 
                else: self.ur3_move_to(initial_q_guess,env)
                if initial_q_guess[1] >= 2*pi: a = - pi/10
                elif initial_q_guess[1] <= -2*pi: a = pi/10
                
                if initial_q_guess[0] >= 0.02: b = -0.05
                elif initial_q_guess[0] <= -0.8: b = 0.05
                
                initial_q_guess[0] = initial_q_guess[0] + b # -0.3
                initial_q_guess[1] = initial_q_guess[1] + a
                initial_q_guess[2] = -2.3
                initial_q_guess[3] = -0.6
                initial_q_guess[4] = 1.6
                initial_q_guess[5] = 1.57
                initial_q_guess[6] = 0

            
        return q_goal
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    def test(self):
        """
        Test the class by adding the 3D objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest
        # self.base = SE3(0.5, 0.5, 0)
        self.add_to_env(env)
        
        fig = self.plot(self.q, limits= [-0.25,0.25,-0.25,0.25,0.4,1.5])
        fig._add_teach_panel(self, self.q)
        fig.hold()
        
        # q_goal = [self.q[i] - pi/3 for i in range(self.n)]
        # qtraj = rtb.jtraj(self.q, q_goal, 50).q

        # for q in qtraj:
        #     self.q = q
        #     env.step(0.02)
        # #     fig.step(0.01)
        # # fig.hold()
        env.hold()    
        # time.sleep(3)
        



if __name__ == "__main__":
    r = LinearUR3()
    r.test()


