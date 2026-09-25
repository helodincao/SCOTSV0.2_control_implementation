from abc import ABC, abstractmethod


class CommandSink(ABC):
    @abstractmethod
    def send(self, vx, vy, vyaw):
        pass

    @abstractmethod
    def stop(self):
        pass

class SimSink(CommandSink):
    def __init__(self, pose_source, tau=0.3, nint=10):
        self.pose_source = pose_source
        self.tau = tau
        self.nint = nint

    def send(self, vx, vy, vyaw):
        self.pose_source.apply_control(
            [vx, vy, vyaw],
            tau=self.tau,
            nint=self.nint
        )

    def stop(self):
        pass