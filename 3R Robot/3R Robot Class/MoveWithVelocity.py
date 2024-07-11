def move_with_velocity(robot, vx, vy):
    # Use Inverse Kinematic Model to calculate joint velocities
    dot_theta1, dot_theta2, dot_theta3 = inverse_kinematic_model(vx, vy)


    # Send commands to set the joint velocities
    set_joint_velocities(robot, dot_theta1, dot_theta2, dot_theta3)

# Example usage:
move_with_velocity(robot, 0.5, 0.3)
