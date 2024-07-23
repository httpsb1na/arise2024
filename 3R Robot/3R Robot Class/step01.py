import numpy as np
import matplotlib.pyplot as plt

class ThreeRRobot:
    def __init__(self, L1, L2, L3, m1, m2, m3, I1, I2, I3):
        # Initialize link lengths, masses, and moments of inertia
        self.L1, self.L2, self.L3 = L1, L2, L3
        self.m1, self.m2, self.m3 = m1, m2, m3
        self.I1, self.I2, self.I3 = I1, I2, I3

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

    def inverse_dynamic_model(self, q, dq, ddq):
        # Calculate the necessary joint torques
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, dq)
        G = self.gravity_vector(q)
        tau = M.dot(ddq) + C.dot(dq) + G
        return tau

    def simulate_kick(self):
        # Define the motion trajectory for the kick in joint space
        key_frames = [
            np.radians([0, 30, -30]),  # Start position
            np.radians([10, 80, -60]), # Wind-up
            np.radians([15, 120, -90]),# Kick
            np.radians([0, 30, -30])   # Return to start
        ]
        # Time intervals (seconds)
        time_intervals = [0.5, 0.2, 0.3, 0.5]
        # Simulation details
        time_steps = 50
        total_time = sum(time_intervals)
        dt = total_time / time_steps
        positions = np.linspace(0, total_time, num=time_steps)

        # Prepare lists to store simulation results
        qs, dqs, ddqs, taus = [], [], [], []

        # Interpolating key frames
        for i in range(len(key_frames) - 1):
            start_frame = key_frames[i]
            end_frame = key_frames[i + 1]
            t_steps = int(time_steps * (time_intervals[i] / total_time))
            for t in np.linspace(0, 1, t_steps):
                q = (1 - t) * start_frame + t * end_frame
                dq = (end_frame - start_frame) / time_intervals[i]
                ddq = np.zeros_like(dq)  # Simplification for this demonstration
                tau = self.inverse_dynamic_model(q, dq, ddq)
                qs.append(q)
                dqs.append(dq)
                ddqs.append(ddq)
                taus.append(tau)

        return qs, dqs, ddqs, taus

    def plot_trajectory(self, qs):
        # Visualization of the trajectory
        fig, ax = plt.subplots()
        for q in qs:
            x0, y0 = 0, 0
            x1 = x0 + self.L1 * np.cos(q[0])
            y1 = y0 + self.L1 * np.sin(q[0])
            x2 = x1 + self.L2 * np.cos(q[0] + q[1])
            y2 = y1 + self.L2 * np.sin(q[0] + q[1])
            x3 = x2 + self.L3 * np.cos(q[0] + q[1] + q[2])
            y3 = y2 + self.L3 * np.sin(q[0] + q[1] + q[2])
            ax.plot([x0, x1, x2, x3], [y0, y1, y2, y3], 'ko-')
            ax.set_xlim([-3, 3])
            ax.set_ylim([-3, 3])
            plt.pause(0.1)
        plt.show()

# Instantiate and use the robot
robot = ThreeRRobot(1.0, 0.8, 0.5, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1)
qs, dqs, ddqs, taus = robot.simulate_kick()
robot.plot_trajectory(qs)

# robot_paths = ["/Users/httpsbina/ARISE2024_students/1-python/3R/fingeredu.urdf"] * 2  
