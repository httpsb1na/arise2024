import pybullet as p
import pybullet_data
import time
import numpy as np
import scipy.linalg

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
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)

        if i == 50:
            p.removeConstraint(constraint_id)

        if i == 100:
            # First robot kicks with adjusted control for a straighter kick
            current_position = p.getJointState(robot_id, 2)[0]
            control_input = controller.compute_control(1.0, current_position)  # Adjusted setpoint for straighter kick
            p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=control_input, force=5000)  # Increased force

        if i == 150:
            # Immediately reset first robot's leg
            p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=0, force=4500)

        if i == 105:
            # Second robot kicks without any conditions
            current_position = p.getJointState(robot_id2, 2)[0]
            control_input = controller.compute_control(1.0, current_position)  # Same adjustment as first robot
            p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=control_input, force=5000)  # Same increased force
        
        if i == 110:
            # Immediately reset second robot's leg
            p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=0, force=4500)

if __name__ == "__main__":
    physics_client = p.connect(p.GUI)
    setup_environment()
    robot_id = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [-0.1, 0, 0.1])  # Set path to the actual URDF
    robot_id2 = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [0.3, 0, 0.1], [0, 0, 1, 0])  # Set path to the actual URDF
    ball_id, constraint_id = create_ball()

    # Define LQR parameters
    A = np.array([[0, 1], [0, 0]])  # Simple state space model for the joint
    B = np.array([[0], [1]])
    Q = np.array([[100, 0], [0, 1]])  # Emphasizing position control
    R = np.array([[1]])  # Control effort

    controller = LQRController(A, B, Q, R)
    simulate_kick(robot_id, robot_id2, ball_id, constraint_id, controller)

    p.disconnect()

# "/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf"
