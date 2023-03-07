import random
import pybullet
import pybullet_data
from pybullet_utils.bullet_client import BulletClient


class PlanarManipulator2DOFPyBullet:
    def __init__(self, timestep, q0, qdot0, multimodel=False):
        self.client = BulletClient(connection_mode=pybullet.GUI)
        self.client.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        self.client.setTimeStep(timestep)
        self.client.setGravity(0, 0, -9.81)
        self.client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.client.loadURDF("./urdf/planar2dof.urdf",
                             flags=pybullet.URDF_USE_IMPLICIT_CYLINDER | pybullet.URDF_USE_INERTIA_FROM_FILE)
        self.client.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0.0, cameraPitch=-89.9,
                                               cameraTargetPosition=[0., 0., 0.])
        for i in range(3):
            self.client.changeDynamics(0, i, lateralFriction=0., linearDamping=0., angularDamping=0.)
        for j in range(self.client.getNumJoints(0)):
            self.client.setJointMotorControl2(0, j, pybullet.POSITION_CONTROL, force=0)
        for i in range(2):
            self.client.resetJointState(0, i + 1, q0[i], qdot0[i])
        self.multimodel = multimodel
        if self.multimodel:
            self.objects_params = [(0.1, 0.05), (0.01, 0.01), (1., 0.3)]
        self.i = 0

    def get_state(self):
        x = [0.] * 4
        for i in range(2):
            x[i], x[i + 2], _, _ = self.client.getJointState(0, i + 1)
        return x

    def set_control(self, u):
        for i in range(2):
            self.client.setJointMotorControl2(0, i + 1, pybullet.TORQUE_CONTROL, **dict(force=u[i]))

    def simulation_step(self):
        if self.multimodel:
            if random.random() < 0.05:
                self.i = random.randint(0, 2)
                m, r = self.objects_params[self.i]
                Ii = 2. / 5 * m * r ** 2
                I = (Ii, Ii, Ii)
                self.change_dynamics(3, m, I)
        print("OBJ_IDX:", self.i)
        self.client.stepSimulation()

    def change_dynamics(self, idx, m, I):
        self.client.changeDynamics(0, idx, mass=m, localInertiaDiagonal=I)
