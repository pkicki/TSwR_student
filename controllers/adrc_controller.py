import numpy as np
from .controller import Controller


class ADRController(Controller):
    def __init__(self, b, kp, kd):
        self.b = b
        self.kp = kp
        self.kd = kd

    def calculate_control(self, q, q_r, q_r_dot, q_r_ddot, eso_estimates):
        q_dot_est = eso_estimates[1]
        f = eso_estimates[2]
        ### TODO: Please implement me
        u = None
        return u
