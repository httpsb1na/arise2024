I = 0.00012  # kg*m^2
torques = [0.003, 0.002, 0.001] 
dt = 0.5  # seconds

theta = 0  # initial position in radians
omega = 0  # initial velocity in rad/s

for tau in torques:
    alpha = tau / I  # calculate angular acceleration
    omega += alpha * dt  # update angular velocity
    theta += omega * dt  # update angular position

print("Final position:", theta, "radians")
