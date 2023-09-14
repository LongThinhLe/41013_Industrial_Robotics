## @file
#  @brief Gripper for UR3 Robot on Linear rails defined by standard DH parameters with 3D model
#  @author Long Thinh Le
#  @date Sep 1, 2023

import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from ir_support.robots.DHRobot3D import DHRobot3D
import os

from math import pi

class GripperLEFT_LinearUR3(DHRobot3D):
    """ GRIPPER RIGHT FOR UR3 ROBOT ON LINEAR RAILS
    """
    def __init__(self):
        # DH links
        links = self._create_DH()
        
        # Names of the robot link files in the directory
        link3D_names = dict(link0 = 'leftGripper_link0', color0 = (0.2,0.2,0.2,1),      # color option only takes effect for stl file
                            link1 = 'empty_stl',
                            link2 = 'empty_stl',
                            link3 = 'leftGripper_link1', color3 = (0.1,0.1,0.1,1),
                            link4 = 'leftGripper_link2', color4 = (1.0,0.0,0.0,1))
        
        # A joint config and the 3D object transforms to match that config
        qtest = [0,0,0,0] # qtest = link3D_names - 1 = 3 - 1 = 2
        
        # WARNING HERE !!! THE SWIFT COORDINATE IS Z,X,Y = X,Y,Z = RED, GREEN, BLUE
        qtest_transforms = [spb.transl(0,0,0) @ spb.trotz(pi/2) @ spb.trotx(pi/2),# @ spb.transl(0,0,-0.0515) @ spb.transl(-0.01482,0,0),
                            spb.transl(0,0,0) ,
                            spb.transl(0,0,0) ,
                            spb.transl(0,0,0) @ spb.trotz(pi/2) @ spb.trotx(pi/2),
                            spb.transl(0,0,0) @ spb.trotz(pi/2) @ spb.trotx(pi/2)]                      
        
        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name = 'GripperLEFT_LinearUR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
        self.base = self.base #* SE3.Trans([0.0515, 0,0]) * SE3.Trans([0,0.01482,0])#
        self.q = qtest
        
    
    def _create_DH(self):
        """ 
        Create Gripper RIGHT Robot's standard DH model
        """
        #links = [rtb.RevoluteDH(d= 0, a=0, alpha= 0, qlim= [-2*pi, 2*pi])]    # Prismatic Link
        links = [] # BASE
        
        a = [0.0515, 0.01482, 0.05, 0.04]
        d = [0.0, 0.0, 0.0, 0.0]
        
        offset = [0,pi/2,-pi/2,0]
        alpha = [0, 0, 0, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(4)]
        
        for i in range(4):
            link = rtb.RevoluteDH(d= d[i], a=a[i], alpha= alpha[i], qlim= qlim[i], offset = offset[i])
            links.append(link)
        return links

    
    def test(self):
        """
        Test the class by adding the 3D objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest
        self.add_to_env(env)
                

        flag = True
        while True:
            if flag:
                q_goal = [0,0,pi/2,-pi/2]
                flag = False
            else:
                # q_goal = [0,0,-pi/4,-pi/8]
                flag = True
   
            qtraj = rtb.jtraj(self.q, q_goal, 50).q
            
            for q in qtraj:
                self.q = q
                env.step(0.02)
            
            env.hold()
                
        
        # fig = self.plot(self.q, limits= [-0.1,0.1,-0.1,0.1,-0.1,0.1])
        # fig._add_teach_panel(self, self.q)
        # fig.hold()


if __name__ == "__main__":
    r = GripperLEFT_LinearUR3()
    r.test()
















################################################################# OLD FILE


# ## @file
# #  @brief Gripper for UR3 Robot on Linear rails defined by standard DH parameters with 3D model
# #  @author Long Thinh Le
# #  @date Sep 1, 2023

# import swift
# import roboticstoolbox as rtb
# import spatialmath.base as spb
# from spatialmath import SE3
# from ir_support.robots.DHRobot3D import DHRobot3D
# import time
# import os

# from math import pi

# class GripperLEFT_LinearUR3(DHRobot3D):
#     """ GRIPPER RIGHT FOR UR3 ROBOT ON LINEAR RAILS
#     """
#     def __init__(self):
#         # DH links
#         links = self._create_DH()
        
#         # Names of the robot link files in the directory
#         link3D_names = dict(link0 = 'Gripper_link0', color0 = (0.2,0.2,0.2,1),      # color option only takes effect for stl file
#                             link1 = 'Gripper_link3', color1 = (0.1,0.1,0.1,1),
#                             link2 = 'Gripper_link4', color2 = (1.0,0.0,0.0,1))
        
#         # A joint config and the 3D object transforms to match that config
#         qtest = [0,0] # qtest = link3D_names - 1 = 3 - 1 = 2
        
#         # WARNING HERE !!! THE SWIFT COORDINATE IS Z,X,Y = X,Y,Z = RED, GREEN, BLUE
#         qtest_transforms = [spb.transl(0,0,0) @ spb.trotz(pi/2) @ spb.trotx(pi/2) @ spb.transl(0,0,-0.0515) @ spb.transl(-0.01482,0,0),
#                             spb.transl(0,0,0) ,
#                             spb.transl(0.05,0,0)]                      
        
#         current_path = os.path.abspath(os.path.dirname(__file__))
#         super().__init__(links, link3D_names, name = 'GripperLEFT_LinearUR3', link3d_dir = current_path, qtest = qtest, qtest_transforms = qtest_transforms)
#         self.base = self.base * SE3.Trans([0.0515, 0,0]) * SE3.Trans([0,0.01482,0])#
#         self.q = qtest
        
    
#     def _create_DH(self):
#         """ 
#         Create Gripper RIGHT Robot's standard DH model
#         """
#         #links = [rtb.RevoluteDH(d= 0, a=0, alpha= 0, qlim= [-2*pi, 2*pi])]    # Prismatic Link
#         links = [] # BASE
        
#         a = [0.05, 0.04]
#         d = [0.0, 0.0]
        
#         alpha = [0, 0]
#         qlim = [[-2*pi, 2*pi] for _ in range(2)]
        
#         for i in range(2):
#             link = rtb.RevoluteDH(d= d[i], a=a[i], alpha= alpha[i], qlim= qlim[i])
#             links.append(link)
#         return links

    
#     def test(self):
#         """
#         Test the class by adding the 3D objects into a new Swift window and do a simple movement
#         """
#         # env = swift.Swift()
#         # env.launch(realtime= True)
#         # self.q = self._qtest
#         # self.add_to_env(env)
                
#         # count = 0
#         # a = pi/2
#         # b = pi/4
#         # while True:

#         #     q_goal = [a,b]
#         #     qtraj = rtb.jtraj(self.q, q_goal, 50).q
#         #     for q in qtraj:
#         #         self.q = q
#         #         env.step(0.02)

#         #     time.sleep(1)
#         #     count = count + 1
#         #     a = a + pi/4
#         #     b = b - pi/8
     
        
#         fig = self.plot(self.q, limits= [-0.1,0.1,-0.1,0.1,-0.1,0.1])
#         fig._add_teach_panel(self, self.q)
#         fig.hold()


# if __name__ == "__main__":
#     r = GripperLEFT_LinearUR3()
#     r.test()