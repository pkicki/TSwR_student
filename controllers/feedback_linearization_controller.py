import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)

    def calculate_control(self, x, v):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        q_dot = x[2:, np.newaxis]
        # tau = self.model.M(x) * v + self.model.C(x) * q_dot
        tau = np.dot(self.model.M(x), v) + np.dot(self.model.C(x), q_dot)

        return tau
