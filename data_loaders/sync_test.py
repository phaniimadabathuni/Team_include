import pandas as pd
import os

def test_synchronization():
    print("Initializing Time-Sync Engine...")
    
    # Paths to both timestamp files
    imu_csv = r"C:\Users\phani\Drone_SLAM\mav0\imu0\data.csv"
    cam_csv = r"C:\Users\phani\Drone_SLAM\mav0\cam0\data.csv"
    
    if not os.path.exists(imu_csv) or not os.path.exists(cam_csv):
        print("ERROR: Cannot find one of the data.csv files. Check paths!")
        return

    # Load the data into memory
    imu_df = pd.read_csv(imu_csv)
    cam_df = pd.read_csv(cam_csv)
    
    # Get the exact nanosecond timestamps of the first two camera frames
    t_cam1 = cam_df.iloc[0, 0]
    t_cam2 = cam_df.iloc[1, 0]
    
    # Filter the IMU data to only show readings that happened BETWEEN those two frames
    imu_between_frames = imu_df[(imu_df.iloc[:, 0] >= t_cam1) & (imu_df.iloc[:, 0] < t_cam2)]
    
    print("\n--- SYNCHRONIZATION RESULTS ---")
    print(f"Camera Frame 1 Time: {t_cam1} ns")
    print(f"Camera Frame 2 Time: {t_cam2} ns")
    print(f"Total IMU readings found between these frames: {len(imu_between_frames)}")
    print("-------------------------------")
    
    if len(imu_between_frames) == 10:
        print("\nSUCCESS! The IMU and Camera are perfectly synchronized.")
    else:
        print("\nWARNING: Sync ratio is off.")

if __name__ == "__main__":
    test_synchronization()