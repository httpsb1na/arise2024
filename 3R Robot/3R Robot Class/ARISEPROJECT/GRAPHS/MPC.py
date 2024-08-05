import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt

class MPCController:
    def __init__(self, horizon, dt, q, r):
        self.horizon = horizon
        self.dt = dt
        self.q = q
        self.r = r

    def compute_control(self, setpoint, state):
        u_optimal = 0
        min_cost = float('inf')
        for u in np.linspace(-10, 10, 100):  # Finer control sampling for better precision
            cost = self.simulate_trajectory(setpoint, state, u)
            if cost < min_cost:
                min_cost = cost
                u_optimal = u
        return u_optimal

    def simulate_trajectory(self, setpoint, state, u):
        cost = 0
        x = state
        for _ in range(self.horizon):
            x = self.dynamics(x, u)
            cost += self.q * (setpoint - x)**2 + self.r * u**2
        return cost

    def dynamics(self, state, u):
        return state + self.dt * u  # More realistic dynamics can be added here

def setup_environment():
    """Set up the simulation environment."""
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")
    return plane_id

def load_robot(urdf_path, basePosition, baseOrientation=[0, 0, 0, 1]):
    """Load the robot from a specified URDF file."""
    robot_id = p.loadURDF(urdf_path, basePosition=basePosition, baseOrientation=baseOrientation, useFixedBase=True)
    return robot_id

def create_ball():
    """Create a ball with collision and visual properties and a constraint to hold it in place."""
    ball_radius = 0.125
    ball_mass = 1.6
    ball_col_id = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
    ball_vis_id = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 0, 0, 1])
    ball_id = p.createMultiBody(ball_mass, ball_col_id, ball_vis_id, basePosition=[0.1, 0, 0.0])
    constraint_id = p.createConstraint(ball_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.1, 0, 0])
    return ball_id, constraint_id

def simulate_kick(robot_id, robot_id2, ball_id, constraint_id, controller):
    """Simulate kicking using MPC control."""
    
    joint2_positions = []
    joint3_positions = []
    joint2_velocities = []
    joint3_velocities = []
    control_efforts = []
    errors_joint2 = []
    errors_joint3 = []
    time_data = []

    for i in range(200):
        p.stepSimulation()
        time.sleep(1./240.)

        # Set desired positions (setpoints) for error calculation
        setpoint_joint2 = 0.8 if i in range(100, 125) else -0.8 if i in range(125, 145) else 0
        setpoint_joint3 = -0.8 if i in range(100, 125) else 0.8 if i in range(125, 145) else 0

        if i == 50:  # Release the ball constraint before kicking
            p.removeConstraint(constraint_id)

        if 100 <= i < 150:
            if i == 100:
                current_position = p.getJointState(robot_id, 2)[0]
                control_input = controller.compute_control(0.8, current_position)
                p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=control_input, force=4500)
                p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=control_input, force=4500)

            elif i == 125:
                current_position = p.getJointState(robot_id, 3)[0]
                control_input = controller.compute_control(-0.8, current_position)
                p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=control_input, force=4500)
                p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=control_input, force=4500)

            elif i == 135:
                p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=0, force=4500)
                p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=0, force=4500)

        if 135 <= i < 175:
            if i == 135:
                current_position = p.getJointState(robot_id, 2)[0]
                control_input = controller.compute_control(0.8, current_position)
                p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=control_input, force=4500)
                p.setJointMotorControl2(robot_id2, 3, p.POSITION_CONTROL, targetPosition=control_input, force=4500)

            elif i == 140:
                current_position = p.getJointState(robot_id, 3)[0]
                control_input = controller.compute_control(-0.8, current_position)
                p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=control_input, force=4500)
                p.setJointMotorControl2(robot_id2, 3, p.POSITION_CONTROL, targetPosition=control_input, force=4500)

            elif i == 150:
                p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=0, force=4500)
                p.setJointMotorControl2(robot_id2, 3, p.POSITION_CONTROL, targetPosition=0, force=4500)

        # Collect data
        joint2_position = p.getJointState(robot_id, 2)[0]
        joint3_position = p.getJointState(robot_id, 3)[0]
        joint2_velocity = p.getJointState(robot_id, 2)[1]
        joint3_velocity = p.getJointState(robot_id, 3)[1]
        control_effort = p.getJointState(robot_id, 2)[3] + p.getJointState(robot_id, 3)[3]
        
        # Calculate errors
        error_joint2 = setpoint_joint2 - joint2_position
        error_joint3 = setpoint_joint3 - joint3_position

        # Append data
        joint2_positions.append(joint2_position)
        joint3_positions.append(joint3_position)
        joint2_velocities.append(joint2_velocity)
        joint3_velocities.append(joint3_velocity)
        control_efforts.append(control_effort)
        errors_joint2.append(error_joint2)
        errors_joint3.append(error_joint3)
        time_data.append(i * 1./240.)

    return time_data, joint2_positions, joint3_positions, joint2_velocities, joint3_velocities, control_efforts, errors_joint2, errors_joint3

def plot_results(time_data, joint2_positions, joint3_positions, joint2_velocities, joint3_velocities, control_efforts, errors_joint2, errors_joint3):
    plt.figure(figsize=(12, 18))

    plt.subplot(3, 2, 1)
    plt.plot(time_data, joint2_positions, label='Joint 2 Position')
    plt.plot(time_data, joint3_positions, label='Joint 3 Position')
    plt.title('Joint Positions')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.legend()

    plt.subplot(3, 2, 2)
    plt.plot(time_data, joint2_velocities, label='Joint 2 Velocity')
    plt.plot(time_data, joint3_velocities, label='Joint 3 Velocity')
    plt.title('Joint Velocities')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (rad/s)')
    plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(time_data, control_efforts, label='Control Effort')
    plt.title('Control Effort (Energy)')
    plt.xlabel('Time (s)')
    plt.ylabel('Effort (arbitrary units)')
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.plot(time_data, errors_joint2, label='Error Joint 2')
    plt.plot(time_data, errors_joint3, label='Error Joint 3')
    plt.title('Tracking Errors')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (rad)')
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.plot(joint2_positions, joint2_velocities, label='Joint 2 Trajectory')
    plt.plot(joint3_positions, joint3_velocities, label='Joint 3 Trajectory')
    plt.title('Position vs. Velocity (Trajectory)')
    plt.xlabel('Position (rad)')
    plt.ylabel('Velocity (rad/s)')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    physics_client = p.connect(p.GUI)
    setup_environment()
    robot_id = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [-0.1, 0, 0.1])
    robot_id2 = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [0.3, 0, 0.1], baseOrientation=[0, 0, 1, 0])
    ball_id, constraint_id = create_ball()

    horizon = 10
    dt = 1.0 / 240.0
    q = 3.0
    r = 0.10
    controller = MPCController(horizon, dt, q, r)

    # Simulate the kick and collect data
    time_data, joint2_positions, joint3_positions, joint2_velocities, joint3_velocities, control_efforts, errors_joint2, errors_joint3 = simulate_kick(robot_id, robot_id2, ball_id, constraint_id, controller)

    # Plot the results
    plot_results(time_data, joint2_positions, joint3_positions, joint2_velocities, joint3_velocities, control_efforts, errors_joint2, errors_joint3)

    p.disconnect()
