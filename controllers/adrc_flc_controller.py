import numpy as np

# from models.free_model import FreeModel
from observers.eso import ESO_centralized
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
# from models.ideal_model import IdealModel
from models.manipulator_model import ManiuplatorModel


class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):
        self.Kp = Kp
        self.Kd = Kd
        self.L = np.array([[3*p[0], 0], 
                            [0, 3*p[1]], 
                            [3*p[0]**2, 0], 
                            [0, 3*p[1]**2], 
                            [p[0]**3, 0], 
                            [0, p[1]**3]])
        self.A = np.array([[0., 0., 1., 0., 0., 0.], 
                           [0., 0., 0., 1., 0., 0.], 
                           [0., 0., 0., 0., 1., 0.], 
                           [0., 0., 0., 0., 0., 1.], 
                           [0., 0., 0., 0., 0., 0.], 
                           [0., 0., 0., 0., 0., 0.]])
        self.B = np.zeros((6, 2))
        W = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])  
        self.model = ManiuplatorModel(Tp, 1.0, 0.01)
        self.eso = ESO_centralized(self.A, self.B, W, self.L, q0, Tp)
        self.update_params(q0[:2], q0[2:])

    def update_params(self, q, q_dot):
        ### TODO Implement procedure to set eso.A and eso.B
        A = self.A
        B = self.B
        x = np.concatenate([q, q_dot], axis=0)
        M = self.model.M(x)
        M_inv = np.linalg.inv(M)
        C = self.model.C(x)
        M_inv_C = -(M_inv @ C)
                
        A[2,2] = M_inv_C[0, 0]
        A[2,3] = M_inv_C[0, 1]
        A[3,2] = M_inv_C[1, 0]
        A[3,3] = M_inv_C[1, 1]
        
        B[2, 0] = M_inv[0, 0]
        B[2, 1] = M_inv[0, 1]
        B[3, 0] = M_inv[1, 0]
        B[3, 1] = M_inv[1, 1]
                
        self.eso.A = A
        self.eso.B = B

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        ### TODO implement centralized ADRFLC
        q1, q2, q1_dot, q2_dot = x
        q = np.array([q1, q2])
        M = self.model.M(x)
        C = self.model.C(x)
        
        x_hat = self.eso.get_q_hat()
        x_dot_hat = self.eso.get_q_dot_hat()
        f_hat = self.eso.get_F_hat()
        
        v = q_d_ddot + self.Kd @ (q_d_dot - x_dot_hat) + self.Kp @ (q_d - q)
        u = M @ (v - f_hat) + C @ x_dot_hat
        
        u = np.clip(u, -10, 10)
        
        self.update_params(q, x_dot_hat)
        self.eso.update(q.reshape(len(q), 1), u.reshape(len(u), 1))
        
        return u
    
    

