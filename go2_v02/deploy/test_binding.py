import scots_ctrl

c = scots_ctrl.Controller("controller")

u = c.get_control([-1.3, 0.0, 0.0])

print(u)