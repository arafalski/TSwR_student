from copy import copy
import numpy as np


class ESO:
    def __init__(self, A, B, W, L, state, Tp):
        self.A = A
        self.B = B
        self.W = W
        self.L = L
        self.state = np.pad(np.array(state), (0, A.shape[0] - len(state)))
        self.Tp = Tp
        self.states = []

    def set_B(self, B):
        self.B = B

    def update(self, q, u):
        self.states.append(copy(self.state))
        ### TODO implement ESO update
        state_dot = (
            self.A @ self.state.reshape((len(self.state), 1))
            + self.B @ u
            + self.L @ (q - self.W @ self.state.reshape((len(self.state), 1)))
        )
        self.state = self.Tp * state_dot.reshape((state_dot.shape[0],)) + self.state

    def get_state(self):
        return self.state
