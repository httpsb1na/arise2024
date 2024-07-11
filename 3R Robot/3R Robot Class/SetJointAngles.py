def set_joint_angles(robot, theta1, theta2, theta3):
    
# Use Direct Geometric Model to calculate end-effector position
    x, y = direct_geometric_model(theta1, theta2, theta3)


# Send commands to set the joint angles
    command_joints(theta1, theta2, theta3)
    return x, y

# Example usage:
x, y = set_joint_angles(robot, 0.5, 1.0, -0.5)
print(f"End-effector position: ({x}, {y})")
