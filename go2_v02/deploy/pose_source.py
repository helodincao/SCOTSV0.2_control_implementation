from abc import ABC, abstractmethod
import math
import requests
import time

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

class OptiTrackPoseSource(PoseSource):
    def __init__(self, url, robot_name):
        self.url = url
        self.robot_name = robot_name
        self.session = requests.Session()

    def get_pose(self):
        response = self.session.get(self.url)
        objects = response.json()

        raw = objects[self.robot_name]

        if raw == "untracked":
            return None

        coordinates = [float(x) for x in raw.split(",")]

        x = coordinates[2]
        y = -coordinates[1]
        yaw = coordinates[3]

        return [x, y, yaw]

    def loop(self):
        while True:
            pose = self.get_pose()

            if pose is None:
                print("NOT BEING TRACKED")
            else:
                print(pose)

            time.sleep(0.2)


if __name__ == "__main__":
    p = OptiTrackPoseSource(
        "http://192.168.1.194:12345/OptiTrackRestServer",
        "GO2-001"
    )

    p.loop()