import pybullet as p
import pybullet_data
import time

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
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)
        
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

        if i in [125, 275]:
            contacts = p.getContactPoints(bodyA=robot_id if i == 125 else robot_id2, bodyB=ball_id)
            if contacts:
                p.changeVisualShape(ball_id, -1, rgbaColor=[0, 1, 0, 1])

if __name__ == "__main__":
    physics_client = p.connect(p.GUI)
    setup_environment()

    robot_id = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [-0.1, 0, 0.1])
    robot_id2 = load_robot("/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf", [0.3, 0, 0.1], baseOrientation=[0, 0, 1, 0])

    
    ball_id, constraint_id = create_ball()
    simulate_kick(robot_id, robot_id2, ball_id, constraint_id)

    p.disconnect()
