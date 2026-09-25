import math
import scots_ctrl

controller = scots_ctrl.Controller("controller")

x = [-1.3, 0.0, 0.0]

def rhs(state, control):
    x, y, theta = state
    vx, vy, vyaw = control

    dx = vx * math.cos(theta) - vy * math.sin(theta)
    dy = vx * math.sin(theta) + vy * math.cos(theta)
    dtheta = vyaw

    return [dx, dy, dtheta]

def rk4_step(state, control, tau=0.3, nint=10):
    h = tau / nint
    x = state[:]

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

    return x

def in_target(state):
    c = [1.2, 0.0, 0.0]
    L = [3.3, 3.3, 0.001]

    value = sum(
        (L[i] * (state[i] - c[i])) ** 2
        for i in range(3)
    )

    return value <= 1.0

for step in range(2000):
    controls = controller.get_control(x)

    if not controls:
        print("outside winning set")
        break

    u = controls[0]

    x = rk4_step(x, u)

    print(x)

    if in_target(x):
        print("Reached target.")
        break