import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_drone_takeoff():
    print("Initializing SLAM Data Engine...")
    
    # The exact path matching your extracted folder
    imu_csv_path = r"C:\Users\phani\Drone_SLAM\mav0\imu0\data.csv"
    
    if not os.path.exists(imu_csv_path):
        print(f"ERROR: Cannot find the file at {imu_csv_path}")
        return

    print("Dataset found! Loading IMU data into memory...")
    
    # Read the massive CSV file
    df = pd.read_csv(imu_csv_path)
    
    # Convert timestamps from nanoseconds to seconds
    df['time_seconds'] = (df.iloc[:, 0] - df.iloc[0, 0]) / 1e9
    
    # Extract the Z-axis acceleration
    z_accel = df.iloc[:, 6]
    
    # Plot the data
    plt.figure(figsize=(12, 6))
    plt.plot(df['time_seconds'], z_accel, color='blue', alpha=0.7)
    
    plt.title("Drone IMU Analysis: Z-Axis Acceleration (Takeoff Detection)")
    plt.xlabel("Time (Seconds)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.grid(True)
    
    print("Spawning Graph Window...")
    plt.show()

if __name__ == "__main__":
    plot_drone_takeoff()