import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """

        q1, q2, q1_dot, q2_dot = x

        q = np.array([[q1], [q2]])
        q_dot = np.array([[q1_dot], [q2_dot]])


        omega_c = 50
        xi = 1

        kp = omega_c**2
        kd = 2 * xi * omega_c

        v = (
            q_r_ddot.reshape(2, 1)
            + kd * (q_r_dot.reshape(2, 1) - q_dot)
            + kp * (q_r.reshape(2, 1) - q)
        )

        return self.model.M(x) @ v + self.model.C(x) @ q_dot
