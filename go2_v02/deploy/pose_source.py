from abc import ABC, abstractmethod
import math


class PoseSource(ABC):
    @abstractmethod
    def get_pose(self):
        pass

class SimPoseSource(PoseSource):
    def __init__(self, start_state):
        self.state = list(start_state)

    def get_pose(self):
        return list(self.state)
    def apply_control(self, control, tau=0.3, nint=10):
        def rhs(state, control):
            x, y, theta = state
            vx, vy, vyaw = control

            dx = vx * math.cos(theta) - vy * math.sin(theta)
            dy = vx * math.sin(theta) + vy * math.cos(theta)
            dtheta = vyaw

            return [dx, dy, dtheta]

        h = tau / nint
        x = self.state[:]

        for _ in range(nint):
            k1 = rhs(x, control)

            x2 = [x[i] + 0.5 * h * k1[i] for i in range(3)]
            k2 = rhs(x2, control)

            x3 = [x[i] + 0.5 * h * k2[i] for i in range(3)]
            k3 = rhs(x3, control)

            x4 = [x[i] + h * k3[i] for i in range(3)]
            k4 = rhs(x4, control)

            x = [
                x[i] + (h / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
                for i in range(3)
            ]

        self.state = x