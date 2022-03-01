from .controller import Controller


class DummyController(Controller):
    def __init__(self, Tp):
        pass

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        return q_r_ddot

    def choose_model(self, x, u, x_dot):
        pass
