import numpy as np
from observers.eso import ESO
from .controller import Controller



class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd

        A = np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]])
        B = np.array([[0], [b], [0]])
        l1 = 3 * p
        l2 =  3* p ** 2
        l3 = p ** 3
        L = np.array([[l1], [l2], [l3]])
        W = np.array([1, 0, 0])
        self.eso = ESO(A, B, W, L, q0, Tp)
        

    def set_b(self, b):
        self.b = b
        self.eso.set_B(np.array([[0], [b], [0]]))

    def psi(self, e : float, e_dot : float ):
        return self.kp * e + self.kd * e_dot


    def calculate_control(self, x, q_d : float, q_d_dot : float, q_d_ddot: float):
        assert len(x) == 2
        q, q_dot = x
        e = q_d - q
        e_dot = q_d_dot - self.eso.get_q_dot_hat()
        u = (q_d_ddot + self.psi(e, e_dot) - self.eso.get_F_hat()) / self.b 
        u = np.clip(u, -10, 10)
        self.eso.update(q, u)
        return u