import numpy as np
import utils
from torobo import Torobo

class Environment:
    def __init__(self, gui = 1):
        self._p = utils.connect(gui)
        self.agent = Torobo(self._p)
        self.init_pos = (0.7, -0.45, 1.8)
        self.ds = 0.4
        self.debug_lines = []
        self.num_actions = 7
        self.reset()
        self.init_debug_lines()
    def init_debug_lines(self):
        
        for i in range(0, self.num_actions):
            id1 = self._p.addUserDebugLine(self.init_pos,
                                           self.get_target_pos(i * np.pi / 6 ),
                                           lifeTime=0,
                                           lineWidth=0.25,
                                           lineColorRGB=(200, 0, 150))
    def reset(self):
        self.agent.init_robot()
    def get_target_pos(self, angle: float):
        return [
                self.init_pos[0],
                self.init_pos[1]+ self.ds * np.cos(angle), 
                self.init_pos[2]+ self.ds * np.sin(angle)
                ]
    def step(self, action: int, phase=0):
        target_pos = self.get_target_pos(np.pi/6 * action +phase )
        joints , pos= self.agent.move_in_cartesian(target_pos,t = 2, sleep=True)
        self.reset()
        return joints, pos
    def use_joint_series(self, joint_time_series, sleep = False, t = 1/30):
        self.reset()
        position_time_series = np.zeros((joint_time_series.shape[0], 3))
        for i, joint in enumerate(joint_time_series):
            self.agent.set_joint_position(joint, t=t, sleep=sleep)
            position_time_series[i] = self.agent.get_link_pos_ori(self.agent.link)[0]
        return position_time_series