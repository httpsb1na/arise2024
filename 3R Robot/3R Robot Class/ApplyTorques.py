def apply_torques(robot, tau1, tau2, tau3):


    # Use Direct Dynamic Model to calculate joint accelerations
    q = get_joint_positions(robot)
    dq = get_joint_velocities(robot)
    ddq = robot.direct_dynamic_model(q, dq, [tau1, tau2, tau3])


    # Apply torques to the joints
    command_torques(robot, tau1, tau2, tau3)
    return ddq

# Example usage:
ddq = apply_torques(robot, 10.0, 15.0, 20.0)
print(f"Joint accelerations: {ddq}")
