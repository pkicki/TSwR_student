import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel

class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        self.models = [
            ManiuplatorModel(Tp, 0.1, 0.05),
            ManiuplatorModel(Tp, 0.01, 0.01),
            ManiuplatorModel(Tp, 1.0, 0.3)
            ]
        self.Tp = Tp
        self.i = 0
        self.u = np.array([0, 0]).transpose()
        self.last_x = None
        self.Kp = 4.0
        self.Kd = 2.0

    def choose_model(self, x):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        if self.last_x is None:
            return
        
        q1, q2, q1_dot, q2_dot = x
        e_list = []
        
        for model in self.models:
            x_dot = model.x_dot(self.last_x, self.u)
            x_dot = x_dot.reshape((4,))
            x_pred = self.Tp * x_dot   + self.last_x
            e = np.linalg.norm(x_pred - x)
            e_list.append(e)            
        
        self.i = np.argmin(e_list)
        print(self.i)
        pass

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)
        q = x[:2]
        q_dot = x[2:]
        
        q1, q2, q1_dot, q2_dot = x
        q = np.array([q1, q2]).transpose()
        q_dot = np.array([q1_dot, q2_dot]).transpose()
        v = q_r_ddot.transpose() + self.Kd * (q_r_dot.transpose() - q_dot) + self.Kp * (q_r.transpose()- q)
        
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        self.u = u
        self.last_x = x
        return u
