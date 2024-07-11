def move_to_position(robot, x, y):
    # Use Inverse Geometric Model to calculate joint angles
    theta1, theta2, theta3 = inverse_geometric_model(x, y)
    # Send commands to move joints to the calculated angles
    set_joint_angles(robot, theta1, theta2, theta3)

# Example usage:
move_to_position(robot, 2.0, 1.0)
