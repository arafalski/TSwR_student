import numpy as np


class ManiuplatorModel:
    def __init__(self, Tp, m3=0, r3=0.05):
        self.Tp = Tp
        self.l1 = 0.5
        self.r1 = 0.04
        self.m1 = 3.0
        self.l2 = 0.4
        self.r2 = 0.04
        self.m2 = 2.4
        self.I_1 = 1 / 12 * self.m1 * (3 * self.r1**2 + self.l1**2)
        self.I_2 = 1 / 12 * self.m2 * (3 * self.r2**2 + self.l2**2)
        self.m3 = m3
        self.r3 = r3
        self.I_3 = 2.0 / 5 * self.m3 * self.r3**2

        self.d1 = self.l1 / 2
        self.d2 = self.l2 / 2

        self.alpha = (
            self.m1 * self.d1**2
            + self.I_1
            + self.m2 * (self.l1**2 + self.d2**2)
            + self.I_2
            + self.m3 * (self.l1**2 + self.l2**2)
            + self.I_3
        )
        self.betha = self.m2 * self.l1 * self.d2 + self.m3 * self.l1 * self.l2
        self.gamma = (
            self.m2 * self.d2**2 + self.I_2 + self.m3 * self.l2**2 + self.I_3
        )

    def M(self, x):
        """
        Please implement the calculation of the mass matrix, according to the model derived in the exercise
        (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x

        return np.array(
            [
                [
                    self.alpha + 2 * self.betha * np.cos(q2),
                    self.gamma + self.betha * np.cos(q2),
                ],
                [self.gamma + self.betha * np.cos(q2), self.gamma],
            ]
        )

    def C(self, x):
        """
        Please implement the calculation of the Coriolis and centrifugal forces matrix, according to the model derived
        in the exercise (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x

        return np.array(
            [
                [
                    -self.betha * np.sin(q2) * q2_dot,
                    -self.betha * np.sin(q2) * (q1_dot + q2_dot),
                ],
                [self.betha * np.sin(q2) * q1_dot, 0],
            ]
        )

    def dx(self, u, x):
        M_inv = np.linalg.inv(self.M(x))
        C = self.C(x)

        A = np.zeros((4, 4))
        A[:2, 2:] = np.eye(2)
        A[2:, 2:] = -M_inv @ C

        B = np.zeros((4, 2))
        B[2:] = M_inv

        return A @ x.reshape(4, 1) + B @ u.reshape(2, 1)
