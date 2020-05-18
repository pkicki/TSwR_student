import numpy as np
from trajectory_generators.trajectory_generator import TrajectoryGenerator


class Poly3(TrajectoryGenerator):
    def __init__(self, start_q, desired_q):
        self.q_0 = start_q
        self.q_k = desired_q
        """
        Please implement the formulas for a_0 till a_3 using self.q_0 and self.q_k
        Assume that the velocities at start and end are zero.
        """
        self.a_0 = None
        self.a_1 = None
        self.a_2 = None
        self.a_3 = None

    def generate(self, t):
        """
        Implement trajectory generator for your manipulator.
        Positional trajectory should be a 3rd degree polynomial going from an initial state q_0 to desired state q_k.
        Remember to derive the first and second derivative of it also.
        Use following formula for the polynomial from the instruction.
        """
        return NotImplementedError()
