from time import sleep
from copy import copy
import numpy as np
from scipy.integrate import odeint
from manipulators.planar_2dof import PlanarManipulator2DOF
from manipulators.planar_2dof_pybullet import PlanarManipulator2DOFPyBullet


def simulate(mode, trajectory_generator, controller, Tp, T, multimodel=False):
    assert mode in ["PYBULLET", "SCIPY"]
    timesteps = np.linspace(0., T, int(T / Tp))
    if mode == "PYBULLET":
        return simulate_pybullet(trajectory_generator, controller, timesteps, multimodel)
    elif mode == "SCIPY":
        return simulate_scipy(trajectory_generator, controller, timesteps)


def simulate_pybullet(trajectory_generator, controller, timesteps, multimodel):
    ctrl = []
    Q = []
    Q_d = []
    q0, qdot0, _ = trajectory_generator.generate(0.)
    manipulator = PlanarManipulator2DOFPyBullet(timesteps[1], q0, qdot0, multimodel)
    for t in timesteps:
        x = np.array(manipulator.get_state())
        Q.append(copy(x))

        q_d, q_d_dot, q_d_ddot = trajectory_generator.generate(t)
        control = controller.calculate_control(x, q_d, q_d_dot, q_d_ddot)

        Q_d.append(np.concatenate([q_d, q_d_dot]))
        ctrl.append(control)
        manipulator.set_control(control)
        manipulator.simulation_step()
        sleep(timesteps[1] / 2)
    return np.array(Q), np.array(Q_d), np.array(ctrl), timesteps


def simulate_scipy(trajectory_generator, controller, timesteps):
    ctrl = []
    Q = []
    Q_d = []
    manipulator = PlanarManipulator2DOF(timesteps[1])
    T = []

    def system(x, t):
        T.append(t)
        Q.append(copy(x))

        q_d, q_d_dot, q_d_ddot = trajectory_generator.generate(t)
        control = controller.calculate_control(x, q_d, q_d_dot, q_d_ddot)

        Q_d.append(np.concatenate([q_d, q_d_dot]))
        ctrl.append(control)
        x_dot = manipulator.x_dot(x, control)
        return x_dot[:, 0]

    q_d, q_d_dot, q_d_ddot = trajectory_generator.generate(0.)
    x = odeint(system, np.concatenate([q_d, q_d_dot], 0), timesteps, hmax=1e-2)
    manipulator.plot(x)
    return np.array(Q), np.array(Q_d), np.array(ctrl), np.array(T)
