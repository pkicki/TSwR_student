import numpy as np


class ManiuplatorModel:
    def __init__(self, Tp, m3_a, r3_a):
        self.Tp = Tp
        self.l1 = 0.5
        self.r1 = 0.01
        self.m1 = 1.
        self.l2 = 0.5
        self.r2 = 0.01
        self.m2 = 1.
        self.I_1 = 1 / 12 * self.m1 * (3 * self.r1 ** 2 + self.l1 ** 2)
        self.I_2 = 1 / 12 * self.m2 * (3 * self.r2 ** 2 + self.l2 ** 2)
        self.m3 = m3_a
        self.r3 = r3_a
        self.I_3 = 2. / 5 * self.m3 * self.r3 ** 2
        
        self.d1 = self.l1 / 2
        self.d2 = self.l2 / 2

        self.alpha = self.m1*self.d1**2 + self.I_1 + \
                     self.m2*(self.l1**2 + self.d2**2) + self.I_2 \
                   + self.m3*(self.l1**2 + self.l2**2) + self.I_3

        self.beta = self.m2 * self.l1 * self.d2 + self.m3 * self.l1 * self.l2

        self.gamma = self.m2 * self.d2**2 + self.I_2 \
                   + self.m3 * self.l2**2 + self.I_3

    def M(self, x):
        """
        Please implement the calculation of the mass matrix, according to the model derived in the exercise
        (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x

        M = np.array([[self.alpha + 2*self.beta * np.cos(q2), self.gamma +  self.beta * np.cos(q2)],
                      [self.gamma + self.beta*np.cos(q2),     self.gamma]])

        return M

    def C(self, x):
        """
        Please implement the calculation of the Coriolis and centrifugal forces matrix, according to the model derived
        in the exercise (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x

        C = np.array([[ - self.beta * np.sin(q2) * q2_dot,  - self.beta * np.sin(q2) * (q1_dot + q2_dot)], 
                      [self.beta * np.sin(q2) * q1_dot, 0]])
        return C

    def x_dot(self, x, u):
        # M * q_dot_dot + C * q_dot = u
        # q_dot_dot = M_inv * (u - C * q_dot)
        
        q_dot = x[2:]
        M_inv = np.linalg.inv(self.M(x))
        
        return M_inv @ (u - self.C(x) @ q_dot)
            
        