from spatialmath import SE3
import spatialgeometry as geometry

from math import pi

class Bricks():
    """ Bricks' information """
    
    def __init__(self):
        brick_file_path = 'HalfSizedRedGreenBrick_STL.stl'
        self.my_brick = geometry.Mesh(brick_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6), color=(0.54, 0.27, 0.07, 1.0))

        # Create an empty list of my Bricks that contain the geometry.Mesh characteristic
        self.my_Bricks = []
        
        # Create a list of postion of bricks
        self.bricks_position =[ (-0.15, 0.43, 0.5),  # 1
                                (-0.1, -0.45, 0.5),      # 2
                                (-0.3, -0.4, 0.5),   # 3
                                (-0.6, 0.3, 0.5),      # 4
                                (-0.5, -0.5, 0.5),     # 5
                                (-0.8, -0.4, 0.5),   # 6 here (-0.5, 0.4, 0)
                                (-0.7, -0.5, 0.5),  # 7
                                (-1.0, 0.15, 0.5),   # 8
                                (-1.0, -0.2, 0.5)   # 9
        ]
        
        # Create a list of rotation of bricks
        self.bricks_rot = [  pi/4,  # 1
                            -pi/2,  # 2
                            pi/3,  # 3
                            pi/8,  # 4
                            -pi/4,  # 5
                            pi/2,  # 6
                            pi/5,  # 7
                            pi/7,  # 8
                            pi/4   # 9
        ]
        
        # Create an empty list to contain the position * rotation of bricks
        self.poses_bricks = []
        
        # Add each pose and rotational status to each brick and store to poses_brick
        for i in range (len(self.bricks_position)):
            self.poses_bricks.append(SE3(self.bricks_position[i]) * SE3.Rz(self.bricks_rot[i])) 
        
        # Add each characteristic of each brick to my Bricks
        for i in range (len(self.bricks_position)):
            my_Brick = geometry.Mesh(brick_file_path, pose = self.poses_bricks[i], color = (1.0, 0, 0, 1.0))
            self.my_Bricks.append(my_Brick)
        
                
    def add_to_env(self,env):
        """Add each brick to the specified environment.

        This function adds each brick in the 'my_Bricks' list to the provided environment.

        Args:
            environment (object): The environment to which the bricks will be added.
        """
        [env.add(brick) for brick in self.my_Bricks]


    def move_brick_class(self, brick_name, trans_matrix, env):
        """Move a brick's position using a given transformation matrix and update the environment.

        This function updates the position of a brick specified by 'brick_name' using the provided
        transformation matrix. It then advances the environment simulation to reflect the new brick position.

        Args:
            brick_name (matrix): The brick's position or transformation matrix to be updated.
            transformation_matrix (matrix): The transformation matrix to apply to the brick's position.
            environment (object): The environment where the simulation will be updated.
        """
        brick_name.T = trans_matrix @ SE3.Rz(pi/2).A @ SE3.Trans(-0.03336,-0.0666,-0.3).A  
        # print('Brick pos \n', brick_name.T)
        env.step(0.02)
        
    
    def final_pose_bricks(self):
        """Calculate the final positions and rotations for a set of bricks.

        This function calculates the final positions and rotations for a set of bricks
        arranged in a specific pattern. The calculated positions and rotations are stored
        in the 'final_pose_bricks' list attribute of the class.

        Note:
            The function uses offsets for width, length, and height to determine the
            relative positions of the bricks and arranges them in a specific pattern.

        """
        # Create an empty list of final position and rotation of bricks
        self.final_pose_bricks = []
        
        # Offset width : 0.06671
        # Offset length: 0.13343
        # Offset height: 0.03336
        # Final position and rot for each brick
        final_position_bricks = SE3(-0.4,0.35,0.8)
        final_rot_bricks = SE3.Rz(pi/2)

        offset_x = 0
        offset_z = 0

        # Arrange the bricks like this: '-' for brick            
        """ 
        ---
        ---
        ---
        """
        for i in range (len(self.bricks_position)):
            if i % 3 == 0 and i > 1:
                offset_z = offset_z + 0.03336
                offset_x = 0
            self.final_pose_bricks.append( final_position_bricks * final_rot_bricks * SE3(0, offset_x, offset_z)) # A matrix 4x4 of a brick
            offset_x = offset_x - 0.06671