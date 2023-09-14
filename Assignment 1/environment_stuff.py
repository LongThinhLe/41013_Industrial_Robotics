from spatialmath import SE3
import spatialgeometry as geometry

class stuff_env():
    """ All stuff for environment such as tiles, table, fence, human """
    
    def __init__(self):
        table_file_path = 'Table.stl'
        self.my_table = geometry.Mesh(table_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6), color=(0.54, 0.27, 0.07, 1.0))

        table_logo_file_path = 'Table_logo.stl'
        self.my_table_logo = geometry.Mesh(table_logo_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6), color=(0.0, 0.0, 0.0, 1.0))
        
        tiles_file_path = 'Tiles.stl'
        self.my_tiles = geometry.Mesh(tiles_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6) * SE3(-1.3,0,-1.27), color=(0.2, 0.2, 0.7, 1.0))

        fence_file_path = 'Fence.stl'
        self.my_fence = geometry.Mesh(fence_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6) * SE3(-1.3,0,-1.27), color=(1.0, 1.0, 0, 1.0))

        man_file_path = 'man_standing.stl'
        self.my_man = geometry.Mesh(man_file_path, pose = SE3(0,0,0) * SE3.Rx(90,'deg') * SE3(-1.4,-0.36,-0.6) * SE3(-1.9,0.35,-1.0), color=(1.0, 0.855, 0.725, 1.0))

        
    def add_to_env(self,env):
        """Add objects to the specified environment.

        This function adds various objects such as a table, table logo, tiles, fence, and a human model
        to the provided environment.

        Args:
            environment (object): The environment to which the objects will be added.
        """
        env.add(self.my_table)
        env.add(self.my_table_logo)
        env.add(self.my_tiles)
        env.add(self.my_fence)
        env.add(self.my_man)
        
        
    