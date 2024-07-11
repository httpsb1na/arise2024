# IKM

def move_with_velocity(robot, vx, vy):
    # Use Inverse Kinematic Model to calculate joint velocities
    dot_theta1, dot_theta2, dot_theta3 = inverse_kinematic_model(vx, vy)


    # Send commands to set the joint velocities
    set_joint_velocities(robot, dot_theta1, dot_theta2, dot_theta3)

# Example usage:
move_with_velocity(robot, 0.5, 0.3)

# DKM 

def move_with_velocity(robot, vx, vy):
    # Get current joint positions (assuming we have a method to get these)
    q = get_joint_positions(robot)
    
    # Use Inverse Jacobian to calculate joint velocities
    J = robot.jacobian(q)
    try:
        J_inv = np.linalg.inv(J)
    except np.linalg.LinAlgError:
        raise ValueError("Jacobian matrix is singular and cannot be inverted.")
    
    # Desired end-effector velocity
    v = np.array([vx, vy])
    
    # Calculate joint velocities
    joint_velocities = J_inv @ v
    set_joint_velocities(robot, joint_velocities[0], joint_velocities[1], joint_velocities[2])
    return joint_velocities

# Example usage:
vx, vy = 0.5, 0.3
joint_velocities = move_with_velocity(robot, vx, vy)
print(f"Joint velocities: {joint_velocities}")



