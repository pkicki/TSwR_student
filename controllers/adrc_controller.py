import numpy as np
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class ADRController(Controller):
    def __init__(self, Tp, params):
        self.model = ManiuplatorModel(Tp, 1.0, 0.01)
        self.joint_controllers = []
        for param in params:
            self.joint_controllers.append(ADRCJointController(*param, Tp))

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        u = []
        for i, controller in enumerate(self.joint_controllers):
            b_hat = np.linalg.inv(self.model.M(x))[i, i]
            controller.set_b(b_hat)
            u.append(controller.calculate_control([x[i], x[i+2]], q_d[i], q_d_dot[i], q_d_ddot[i]))
        u = np.array(u)[:, np.newaxis]
        return u

