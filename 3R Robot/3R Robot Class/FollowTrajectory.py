def follow_trajectory(robot, desired_positions, desired_velocities, desired_accelerations):
    torques = []


    for q, dot_q, ddot_q in zip(desired_positions, desired_velocities, desired_accelerations):
        
        # Use Inverse Dynamic Model to calculate joint torques
        tau1, tau2, tau3 = inverse_dynamic_model(robot, q, dot_q, ddot_q)
        torques.append((tau1, tau2, tau3))
       
        # Apply the calculated torques to the joints
        apply_torques(robot, tau1, tau2, tau3)
    return torques

# Example usage:
desired_positions = [(0.1, 0.2, 0.3), (0.15, 0.25, 0.35)]
desired_velocities = [(0.01, 0.02, 0.03), (0.015, 0.025, 0.035)]
desired_accelerations = [(0.001, 0.002, 0.003), (0.0015, 0.0025, 0.0035)]
torques = follow_trajectory(robot, desired_positions, desired_velocities, desired_accelerations)
print(f"Applied torques: {torques}")
