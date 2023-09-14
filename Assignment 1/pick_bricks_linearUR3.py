from LinearUR3 import LinearUR3
from Gripper import Gripper_LinearUR3

import roboticstoolbox as rtb

class pick_bricks_linearUR3():
    def __init__(self):
        self.r_UR3_linear = LinearUR3()
        self.r_UR3_gripper = Gripper_LinearUR3()

    def add_to_env(self,env):
        self.r_UR3_linear.add_to_env(env)
        self.r_UR3_gripper.add_to_env(env)
    
    def ur3_move_to(q_goal):
        qtraj = rtb.jtraj(r_UR3_linear.q, q_goal, 30).q
        for q in qtraj:
            r_UR3_linear.q = q
            fkine_ur3_mat = r_UR3_linear.fkine(q).A
            r_UR3_gripper.transl_base(fkine_ur3_mat, env)
            env.step(0.005)