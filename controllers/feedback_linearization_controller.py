import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp, m3):
        self.model = ManiuplatorModel(Tp, m3)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        Kd = 5.0
        Kp = 10.0
        q1, q2, q1_dot, q2_dot = x
        
        q = np.array([q1, q2]).transpose()
        q_dot = np.array([q1_dot, q2_dot]).transpose()
    
        v = q_r_ddot.transpose() + Kd * (q_r_dot.transpose() - q_dot) + Kp * (q_r.transpose()- q)
        
        tau = self.model.M(x) @ v + self.model.C(x) @ q_dot
                
        return tau
