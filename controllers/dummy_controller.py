from .controller import Controller


class DummyController(Controller):
    def __init__(self, Tp):
        pass

    def calculate_control(self, x, v, *args):
        return v

    def choose_model(self, x, u, x_dot):
        pass
