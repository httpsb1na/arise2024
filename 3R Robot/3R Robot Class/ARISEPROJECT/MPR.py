import pybullet as p
import pybullet_data
import numpy as np
import time
import scipy.linalg

# Classes for PID, MPC, and LQR Controllers 
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

class LQRController:
    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.K = self.compute_lqr_gain(A, B, Q, R)

    def compute_lqr_gain(self, A, B, Q, R):
        """Solve the continuous time LQR controller."""
        # Solve the continuous-time algebraic Riccati equation (CARE)
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        # Compute the LQR gain
        K = np.linalg.inv(R) @ (B.T @ P)
        return K

    def compute_control(self, setpoint, state):
        """Calculate control input based on state and setpoint."""
        error = setpoint - state
        u = -self.K @ np.array([error, 0])  # Assuming a simple single integrator model for demonstration
        return u.item()

# Simulation setup and utility functions
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

def reset_ball(ball_id, constraint_id):
    """Reset the ball to its initial position."""
    p.resetBasePositionAndOrientation(ball_id, [0.1, 0, 0.0], [0, 0, 0, 1])
    constraint_id = p.createConstraint(ball_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0.1, 0, 0])
    return constraint_id

def simulate_kick(robot_id, robot_id2, ball_id, constraint_id, controller):
    """Simulate two robots taking turns to kick the ball, using the appropriate kicking logic for the controller."""
    reset_position = [0, 0]  # Position to reset the robot legs

    if isinstance(controller, LQRController):
        # LQR kicking logic
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
        
        # Wait until the kick is fully complete before resetting legs
        time.sleep(0.1)
        p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=reset_position[0], force=4500)
        p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=reset_position[1], force=4500)

    elif isinstance(controller, MPCController):
        # MPC kicking logic
        for i in range(150):
            p.stepSimulation()
            time.sleep(1./120.)  # Increase time step to make it faster
            
            if i == 50:
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

        # Wait until the kick is fully complete before resetting legs
        time.sleep(0.1)
        p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=reset_position[0], force=4500)
        p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=reset_position[1], force=4500)

    elif isinstance(controller, PIDController):
        # PID kicking logic
        for i in range(150):
            p.stepSimulation()
            time.sleep(1./120.)  # Increase time step to make it faster
            
            if i == 124:
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

        # Wait until the kick is fully complete before resetting legs
        time.sleep(0.1)
        p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=reset_position[0], force=4500)
        p.setJointMotorControl2(robot_id2, 2, p.POSITION_CONTROL, targetPosition=reset_position[1], force=4500)

if __name__ == "__main__":
    physics_client = p.connect(p.GUI)
    setup_environment()

    robot_id = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [-0.1, 0, 0.1])
    robot_id2 = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [0.3, 0, 0.1], baseOrientation=[0, 0, 1, 0])
    
    ball_id, constraint_id = create_ball()

    for controller in [PIDController(100, 0, 20), MPCController(10, 1.0 / 240.0, 3.0, 0.10), LQRController(np.array([[0, 1], [0, 0]]), np.array([[0], [1]]), np.array([[100, 0], [0, 1]]), np.array([[1]]))]:
        simulate_kick(robot_id, robot_id2, ball_id, constraint_id, controller)
        constraint_id = reset_ball(ball_id, constraint_id)  # Reset the ball after each simulation

    p.disconnect()
