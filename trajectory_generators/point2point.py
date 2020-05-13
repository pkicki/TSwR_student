import numpy as np
from trajectory_generators.trajectory_generator import TrajectoryGenerator


class Point2point(TrajectoryGenerator):
    def __init__(self, desired_q, desired_q_dot):
        self.q_0 = np.zeros_like(desired_q)
        self.q_dot_0 = np.zeros_like(desired_q_dot)
        self.q_k = desired_q
        self.q_dot_k = desired_q_dot

    def generate(self, t):
        """
        Implement trajectory generator for your manipulator.
        Positional trajectory should be a 3rd degree polynomial going from an initial state q_0 to desired state q_k.
        Remember to derive the first and second derivative of it also.
        Use following formula for the polynomial from the instruction.
        """
        return NotImplementedError()

    def set_initial_state(self, q, q_dot):
        self.q_0 = q
        self.q_dot_0 = q_dot
