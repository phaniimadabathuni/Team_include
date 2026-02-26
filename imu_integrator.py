import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def run_imu_integration():
    print("Initializing IMU Dead Reckoning Engine...")
    
    # Path to your IMU data
    imu_csv = r"C:\Users\phani\Drone_SLAM\mav0\imu0\data.csv"
    df = pd.read_csv(imu_csv)
    
    # Extract timestamps (nanoseconds) and Z-axis acceleration (m/s^2)
    timestamps = df.iloc[:, 0].values
    accel_z = df.iloc[:, 3].values  # Z-axis is usually the 4th column in EuRoC
    
    # Initial Conditions
    position_z = 0.0
    velocity_z = 0.0
    gravity = 9.81
    
    # Lists to store the tracked data for graphing
    time_history = [0.0]
    position_history = [0.0]
    
    print("Integrating IMU data...")
    
    # Loop through the data (starting from index 1 to calculate time difference)
    for i in range(1, len(timestamps)):
        # Calculate Delta T (time elapsed between readings in seconds)
        dt = (timestamps[i] - timestamps[i-1]) / 1e9 
        
        # Get current acceleration and remove gravity
        # (Note: Depending on IMU orientation, gravity might be positive or negative)
        a_z = accel_z[i] - gravity
        
        # Kinematic Equations
        position_z = position_z + (velocity_z * dt) + (0.5 * a_z * dt**2)
        velocity_z = velocity_z + (a_z * dt)
        
        # Save for plotting
        time_sec = (timestamps[i] - timestamps[0]) / 1e9
        time_history.append(time_sec)
        position_history.append(position_z)

    # Plot the drone's calculated altitude
    plt.figure(figsize=(10, 5))
    plt.plot(time_history, position_history, label="Calculated Z-Position (Altitude)", color='b')
    plt.title("IMU Dead Reckoning: Drone Altitude Over Time")
    plt.xlabel("Time (seconds)")
    plt.ylabel("Position Z (meters)")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    run_imu_integration()