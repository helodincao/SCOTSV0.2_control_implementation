import time
import scots_ctrl
from pose_source import SimPoseSource
from command_sink import SimSink

def run_controller(pose_source, command_sink, controller_path, target_check, tau=0.3, max_steps=2000):
        
    controller = scots_ctrl.Controller(controller_path)
    try:
        for step in range(max_steps):
            x = pose_source.get_pose()
            controls = controller.get_control(x)

            if not controls:
                print("outside winning set")
                command_sink.stop()
                break

            u = controls[0]

            command_sink.send(u[0],u[1],u[2])

            x = pose_source.get_pose()
            print(x)

            if target_check(x):
                print("Reached target.")
                break

            time.sleep(tau)

    finally:
        command_sink.stop()

def in_target(state):
    c = [1.2, 0.0, 0.0]
    L = [3.3, 3.3, 0.001]

    value = sum(
        (L[i] * (state[i] - c[i])) ** 2
        for i in range(3)
    )

    return value <= 1.0


if __name__ == "__main__":
    pose = SimPoseSource([-1.3, 0.0, 0.0])
    sink = SimSink(pose, tau=0.3, nint=10)

    run_controller(
        pose_source=pose,
        command_sink=sink,
        controller_path="controller",
        target_check=in_target,
        tau=0.3,
        max_steps=2000
    )