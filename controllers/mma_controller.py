import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        self.models = [
            ManiuplatorModel(Tp, m3=0.1, r3=0.05),
            ManiuplatorModel(Tp, m3=0.01, r3=0.01),
            ManiuplatorModel(Tp, m3=1.0, r3=0.3),
        ]
        self.i = 0
        self.Tp = Tp
        self.last_u = np.zeros(2)
        self.last_x = np.zeros(4)

    def choose_model(self, x):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        x_est = [
            model.dx(self.last_u, self.last_x) * self.Tp + self.last_x.reshape(4, 1)
            for model in self.models
        ]

        model_errors = [np.linalg.norm(x_model - x.reshape(4, 1)) for x_model in x_est]
        self.i = model_errors.index(min(model_errors))
        print(self.i)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)

        q1, q2, q1_dot, q2_dot = x

        q = np.array([[q1], [q2]])
        q_dot = np.array([[q1_dot], [q2_dot]])

        kp = 60
        kd = 12

        v = (
            q_r_ddot.reshape(2, 1)
            + kd * (q_r_dot.reshape(2, 1) - q_dot)
            + kp * (q_r.reshape(2, 1) - q)
        )

        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v + C @ q_dot

        self.last_u = u
        self.last_x = x
        return u
