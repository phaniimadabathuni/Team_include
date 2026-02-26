import numpy as np

class DroneEKF:
    def __init__(self):
        print("Initializing Extended Kalman Filter Brain...")
        
        # State Vector [X, Y, Z, Vx, Vy, Vz]
        self.x = np.zeros((6, 1))
        
        # Covariance Matrix (P) - Starts with low uncertainty
        self.P = np.eye(6) * 0.1
        
        # Process Noise (Q) - How much do we distrust the IMU?
        self.Q = np.eye(6) * 0.01
        
        # Measurement Noise (R) - How much do we distrust the Camera?
        self.R = np.eye(3) * 0.05
        
        # Measurement Matrix (H) - Maps state to measurement (we only measure X, Y, Z from vision)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.gravity = 9.81

    def predict(self, accel_z, dt):
        """Step 1: The IMU guesses the new position (Runs at 200Hz)"""
        
        # State Transition Matrix (F) for basic kinematics
        F = np.eye(6)
        F[0, 3] = dt  # X = X + Vx*dt
        F[1, 4] = dt  # Y = Y + Vy*dt
        F[2, 5] = dt  # Z = Z + Vz*dt
        
        # Control Input Matrix (B) and vector (u)
        B = np.zeros((6, 1))
        B[3, 0] = dt
        B[4, 0] = dt
        B[5, 0] = dt
        
        # Remove gravity from Z acceleration
        a_z_true = accel_z - self.gravity
        u = np.array([[0], [0], [a_z_true]])
        
        # 1. Predict State
        self.x = np.dot(F, self.x) + np.dot(B, u)
        
        # 2. Predict Covariance (Uncertainty grows)
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q
        
        return self.x[0:3] # Return current [X, Y, Z] guess

    def update(self, vision_z):
        """Step 2: The Camera corrects the IMU drift (Runs at 20Hz)"""
        
        # Vision measurement vector
        z = np.array([[0], [0], [vision_z]])
        
        # 1. Calculate Kalman Gain (K)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # 2. Calculate the Error (Measurement - Prediction)
        y = z - np.dot(self.H, self.x)
        
        # 3. Update State (Correct the drift!)
        self.x = self.x + np.dot(K, y)
        
        # 4. Update Covariance (Uncertainty shrinks)
        I = np.eye(6)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
        
        return self.x[0:3]

if __name__ == "__main__":
    ekf = DroneEKF()
    print("EKF initialized and ready to fuse data.")