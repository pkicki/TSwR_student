import numpy as np
from .controller import Controller


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # Use parameters from manipulators/mm_planar_2dof.py
        self.models = [None, None, None]
        self.i = 0

    def choose_model(self, x, u, x_dot):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        pass

    def calculate_control(self, x, desired_q_ddot):
        v = desired_q_ddot  # TODO: Add Feedback
        q_dot = x[2:, np.newaxis]
        M = self.models[self.i].M(x)
        return M @ (v + np.linalg.inv(M) @ self.models[self.i].C(x) @ q_dot)
