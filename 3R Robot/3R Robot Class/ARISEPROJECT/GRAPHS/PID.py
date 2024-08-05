import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

def setup_environment():
    """Set up the simulation environment."""
    p.setGravity(0, 0, -10)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")
    return plane_id

def load_robot(urdf_path, basePosition, baseOrientation=[0, 0, 0, 1]):
    """Load the robot from a specified URDF file with a specified base position and orientation."""
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

def simulate_kick(robot_id, robot_id2, ball_id, constraint_id):
    """Simulate two robots taking turns to kick the ball, releasing the constraint before kicking."""
    
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

        if i == 124:  # Just before the kick
            p.removeConstraint(constraint_id)

        if 100 <= i < 150:
            if i == 100:
                p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=0.8, force=4500)
                p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=-0.8, force=4500)
            elif i == 125:
                p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=-0.8, force=4500)
                p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=0.8, force=4500)
            elif i == 145:
                p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=0, force=4500)
                p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=0, force=4500)

        if 145 <= i < 165:
            if i == 145:
                p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=0.8, force=4500)
                p.setJointMotorControl2(robot_id2, 3, p.POSITION_CONTROL, targetPosition=-0.8, force=4500)
            elif i == 150:
                p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=-0.8, force=4500)
                p.setJointMotorControl2(robot_id2, 3, p.POSITION_CONTROL, targetPosition=0.8, force=4500)
            elif i == 165:
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

    # Simulate the kick and collect data
    time_data, joint2_positions, joint3_positions, joint2_velocities, joint3_velocities, control_efforts, errors_joint2, errors_joint3 = simulate_kick(robot_id, robot_id2, ball_id, constraint_id)

    # Plot the results
    plot_results(time_data, joint2_positions, joint3_positions, joint2_velocities, joint3_velocities, control_efforts, errors_joint2, errors_joint3)

    p.disconnect()
