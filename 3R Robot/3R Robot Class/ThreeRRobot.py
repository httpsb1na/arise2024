import numpy as np

class ThreeRRobot:
    def __init__(self, L1, L2, L3, m1, m2, m3, I1, I2, I3):
        # Robot link lengths
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        
        # Robot link masses
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        
        # Moments of inertia
        self.I1 = I1
        self.I2 = I2
        self.I3 = I3

    def mass_matrix(self, q):
        # Get joint positions
        theta1, theta2, theta3 = q


        # Solve for elements of the mass matrix
        M11 = self.I1 + self.I2 + self.I3 + self.m1 * (self.L1**2) + self.m2 * (self.L1**2 + self.L2**2 + 2 * self.L1 * self.L2 * np.cos(theta2)) + self.m3 * (self.L1**2 + self.L2**2 + self.L3**2 + 2 * self.L1 * self.L2 * np.cos(theta2) + 2 * self.L2 * self.L3 * np.cos(theta3) + 2 * self.L1 * self.L3 * np.cos(theta2 + theta3))
        M12 = self.I2 + self.I3 + self.m2 * (self.L2**2 + self.L1 * self.L2 * np.cos(theta2)) + self.m3 * (self.L2**2 + self.L3**2 + self.L1 * self.L2 * np.cos(theta2) + 2 * self.L2 * self.L3 * np.cos(theta3) + self.L1 * self.L3 * np.cos(theta2 + theta3))
        M13 = self.I3 + self.m3 * (self.L3**2 + self.L2 * self.L3 * np.cos(theta3) + self.L1 * self.L3 * np.cos(theta2 + theta3))
        M21 = M12
        M22 = self.I2 + self.I3 + self.m2 * (self.L2**2) + self.m3 * (self.L2**2 + self.L3**2 + 2 * self.L2 * self.L3 * np.cos(theta3))
        M23 = self.I3 + self.m3 * (self.L3**2 + self.L2 * self.L3 * np.cos(theta3))
        M31 = M13
        M32 = M23
        M33 = self.I3 + self.m3 * (self.L3**2)
        
        # Return mass matrix
        M = np.array([[M11, M12, M13],
                      [M21, M22, M23],
                      [M31, M32, M33]])
        return M

    def coriolis_matrix(self, q, dq):
        # Get joint positions and velocities
        theta1, theta2, theta3 = q
        dtheta1, dtheta2, dtheta3 = dq
        
        # Solve for elements of the Coriolis matrix
        C11 = -self.m2 * self.L1 * self.L2 * np.sin(theta2) * (2 * dtheta1 * dtheta2 + dtheta2**2) - self.m3 * self.L1 * self.L2 * np.sin(theta2) * (2 * dtheta1 * dtheta2 + dtheta2**2) - self.m3 * self.L2 * self.L3 * np.sin(theta3) * (2 * dtheta2 * dtheta3 + dtheta3**2) - self.m3 * self.L1 * self.L3 * np.sin(theta2 + theta3) * (2 * dtheta1 * (dtheta2 + dtheta3) + (dtheta2 + dtheta3)**2)
        C12 = -self.m2 * self.L1 * self.L2 * np.sin(theta2) * dtheta1**2 - self.m3 * self.L1 * self.L2 * np.sin(theta2) * dtheta1**2 - self.m3 * self.L2 * self.L3 * np.sin(theta3) * dtheta2**2 - self.m3 * self.L1 * self.L3 * np.sin(theta2 + theta3) * dtheta1**2
        C13 = -self.m3 * self.L2 * self.L3 * np.sin(theta3) * dtheta2**2 - self.m3 * self.L1 * self.L3 * np.sin(theta2 + theta3) * dtheta1**2
        C21 = self.m2 * self.L1 * self.L2 * np.sin(theta2) * dtheta1**2 + self.m3 * self.L1 * self.L2 * np.sin(theta2) * dtheta1**2 + self.m3 * self.L1 * self.L3 * np.sin(theta2 + theta3) * dtheta1**2
        C22 = 0
        C23 = -self.m3 * self.L2 * self.L3 * np.sin(theta3) * dtheta2**2
        C31 = self.m3 * self.L2 * self.L3 * np.sin(theta3) * dtheta2**2 + self.m3 * self.L1 * self.L3 * np.sin(theta2 + theta3) * dtheta1**2
        C32 = self.m3 * self.L2 * self.L3 * np.sin(theta3) * dtheta2**2
        C33 = 0
        
        # Return Coriolis matrix
        C = np.array([[C11, C12, C13],
                      [C21, C22, C23],
                      [C31, C32, C33]])
        return C

    def gravity_vector(self, q):
        # Get joint positions
        theta1, theta2, theta3 = q
        
        # Solve for elements of the gravity vector
        G1 = (self.m1 * self.L1 / 2 + self.m2 * self.L1 + self.m3 * self.L1) * np.cos(theta1) + (self.m2 * self.L2 / 2 + self.m3 * self.L2) * np.cos(theta1 + theta2) + self.m3 * self.L3 / 2 * np.cos(theta1 + theta2 + theta3)
        G2 = (self.m2 * self.L2 / 2 + self.m3 * self.L2) * np.cos(theta1 + theta2) + self.m3 * self.L3 / 2 * np.cos(theta1 + theta2 + theta3)
        G3 = self.m3 * self.L3 / 2 * np.cos(theta1 + theta2 + theta3)
        
        # Return gravity vector
        G = np.array([G1, G2, G3])
        return G

    def direct_dynamic_model(self, q, dq, tau):
        # Compute mass matrix, Coriolis matrix, and gravity vector
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, dq)
        G = self.gravity_vector(q)
        
        # Solve for joint accelerations
        ddq = np.linalg.inv(M) @ (tau - C @ dq - G)
        return ddq

    def jacobian(self, q):
        theta1, theta2, theta3 = q
        J = np.array([
            [-self.L1 * np.sin(theta1) - self.L2 * np.sin(theta1 + theta2) - self.L3 * np.sin(theta1 + theta2 + theta3), -self.L2 * np.sin(theta1 + theta2) - self.L3 * np.sin(theta1 + theta2 + theta3), -self.L3 * np.sin(theta1 + theta2 + theta3)],
            [self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2) + self.L3 * np.cos(theta1 + theta2 + theta3), self.L2 * np.cos(theta1 + theta2) + self.L3 * np.cos(theta1 + theta2 + theta3), self.L3 * np.cos(theta1 + theta2 + theta3)]
        ])
        return J

    def direct_kinematic_model(self, q, dq):
        J = self.jacobian(q)
        end_effector_velocity = J @ dq
        return end_effector_velocity



# Example usage
L1, L2, L3 = 1.0, 1.0, 1.0  # Link lengths
m1, m2, m3 = 1.0, 1.0, 1.0  # Link masses
I1, I2, I3 = 0.1, 0.1, 0.1  # Moments of inertia

robot = ThreeRRobot(L1, L2, L3, m1, m2, m3, I1, I2, I3)

q = [0.5, 0.5, 0.5]          # Joint positions
dq = [0.1, 0.1, 0.1]         # Joint velocities
tau = [1.0, 1.0, 1.0]        # Joint torques

ddq = robot.direct_dynamic_model(q, dq, tau)
print(f"Joint accelerations: {ddq}")

