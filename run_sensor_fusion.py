import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from slam_ekf import DroneEKF

def run_fusion():
    print("Initializing Team #include V-I SLAM Engine...")
    
    # Load your EuRoC IMU Data
    imu_csv = r"C:\Users\phani\Drone_SLAM\mav0\imu0\data.csv"
    df = pd.read_csv(imu_csv)
    timestamps = df.iloc[:, 0].values
    accel_z = df.iloc[:, 3].values  
    
    # 1. Initialize Your Brain
    ekf = DroneEKF()
    
    # Arrays to store data for the graph
    time_history = []
    raw_imu_z = []
    ekf_fused_z = []
    
    # Variables for the "dumb" raw IMU calculation
    raw_pos = 0.0
    raw_vel = 0.0
    gravity = 9.81
    
    print("Fusing IMU (200Hz) and Vision (20Hz)...")
    
    # The Master Loop
    for i in range(1, len(timestamps)):
        dt = (timestamps[i] - timestamps[i-1]) / 1e9
        time_sec = (timestamps[i] - timestamps[0]) / 1e9
        
        # --- THE CONTROL GROUP (Raw Physics) ---
        a_z_true = accel_z[i] - gravity
        raw_pos = raw_pos + (raw_vel * dt) + (0.5 * a_z_true * dt**2)
        raw_vel = raw_vel + (a_z_true * dt)
        
        # --- THE EKF BRAIN (Sensor Fusion) ---
        # Step 1: Predict via IMU
        ekf.predict(accel_z[i], dt)
        
        # Step 2: Update via Vision (Simulated 20Hz update)
        if i % 10 == 0:
            # We simulate the camera seeing the drone hovering perfectly at 1.0 meter
            simulated_vision_altitude = 1.0 
            ekf.update(simulated_vision_altitude)
            
        # Record the results
        time_history.append(time_sec)
        raw_imu_z.append(raw_pos)
        ekf_fused_z.append(ekf.x[2, 0]) # Index 2 is the Z-axis

    print("Processing complete. Rendering telemetry...")

    # Plot the showdown
    plt.figure(figsize=(10, 6))
    
    # Plot the raw, failing IMU
    plt.plot(time_history, raw_imu_z, label="Raw IMU (Mathematical Drift)", color='red', linestyle='dashed')
    
    # Plot the EKF
    plt.plot(time_history, ekf_fused_z, label="EKF Fused Altitude (Stable)", color='green', linewidth=2.5)
    
    plt.title("Sensor Fusion Showdown: EKF vs Raw IMU")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Altitude (meters)")
    
    # We zoom the graph in to the top 2 meters. 
    # The red line will literally shoot off the bottom of the screen into the abyss!
    plt.ylim(-1, 2.5) 
    
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    run_fusion()