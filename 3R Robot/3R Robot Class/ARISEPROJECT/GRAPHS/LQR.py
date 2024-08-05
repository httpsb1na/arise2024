import pybullet as p
import pybullet_data
import time
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

class LQRController:
    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.K = self.compute_lqr_gain(A, B, Q, R)

    def compute_lqr_gain(self, A, B, Q, R):
        """Solve the continuous time LQR controller."""
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        return K

    def compute_control(self, setpoint, state):
        """Calculate control input based on state and setpoint."""
        error = setpoint - state
        u = -self.K @ np.array([error, 0])  # Assuming a simple single integrator model for demonstration
        return u.item()

def setup_environment():
    """Set up the simulation environment and return the ID of the plane."""
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
    ball_id = p.createMultiBody(ball_mass, ball_col_id, ball_vis_id, basePosition=[0, 0, 0.1])
    constraint_id = p.createConstraint(ball_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
    return ball_id, constraint_id

def simulate_kick(robot_id, robot_id2, ball_id, constraint_id, controller):
    """Simulate kicking using LQR control."""

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

        setpoint_joint2 = 1.0
        setpoint_joint3 = 1.0

        if i == 50:
            p.removeConstraint(constraint_id)

        if i == 100:
            current_position2 = p.getJointState(robot_id, 2)[0]
            current_position3 = p.getJointState(robot_id, 3)[0]
            control_input2 = controller.compute_control(setpoint_joint2, current_position2)
            control_input3 = controller.compute_control(setpoint_joint3, current_position3)
            p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=control_input2, force=5000)
            p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=control_input3, force=5000)

        if i == 150:
            p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=0, force=4500)
            p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=0, force=4500)

        if i == 105:
            current_position2 = p.getJointState(robot_id2, 2)[0]
            current_position3 = p.getJointState(robot_id2, 3)[0]
            control_input2 = controller.compute_control(setpoint_joint2, current_position2)
            control_input3 = controller.compute_control(setpoint_joint3, current_position3)
            p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=control_input2, force=5000)
            p.setJointMotorControl2(robot_id2, 3, p.POSITION_CONTROL, targetPosition=control_input3, force=5000)

        if i == 110:
            p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=0, force=4500)
            p.setJointMotorControl2(robot_id2, 3, p.POSITION_CONTROL, targetPosition=0, force=4500)

        # Collect data
        joint2_position = p.getJointState(robot_id, 2)[0]
        joint3_position = p.getJointState(robot_id, 3)[0]
        joint2_velocity = p.getJointState(robot_id, 2)[1]
        joint3_velocity = p.getJointState(robot_id, 3)[1]
        control_effort = p.getJointState(robot_id, 2)[3] + p.getJointState(robot_id, 3)[3]

        error_joint2 = setpoint_joint2 - joint2_position
        error_joint3 = setpoint_joint3 - joint3_position

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
    robot_id2 = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [0.3, 0, 0.1], [0, 0, 1, 0])
    ball_id, constraint_id = create_ball()

    # Define LQR parameters
    A = np.array([[0, 1], [0, 0]])  # Simple state space model for the joint
    B = np.array([[0], [1]])
    Q = np.array([[100, 0], [0, 1]])  # Emphasizing position control
    R = np.array([[1]])  # Control effort

    controller = LQRController(A, B, Q, R)

    # Simulate the kick and collect data
    time_data, joint2_positions, joint3_positions, joint2_velocities, joint3_velocities, control_efforts, errors_joint2, errors_joint3 = simulate_kick(robot_id, robot_id2, ball_id, constraint_id, controller)

    # Plot the results
    plot_results(time_data, joint2_positions, joint3_positions, joint2_velocities, joint3_velocities, control_efforts, errors_joint2, errors_joint3)

    p.disconnect()
