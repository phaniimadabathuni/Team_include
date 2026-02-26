import cv2
import pandas as pd
import os
import numpy as np

def run_slam_engine():
    print("Starting Main SLAM Dataset Player with Feature Detection...")
    
    cam0_dir = r"C:\Users\phani\Drone_SLAM\mav0\cam0\data"
    cam0_csv = r"C:\Users\phani\Drone_SLAM\mav0\cam0\data.csv"
    
    if not os.path.exists(cam0_csv):
        print("ERROR: Could not find the camera CSV file.")
        return

    cam_df = pd.read_csv(cam0_csv)
    print("Playing dataset... Click the video window and press 'q' to stop.")
    
    for index, row in cam_df.iterrows():
        timestamp = str(row.iloc[0]) 
        img_name = timestamp + ".png"
        img_path = os.path.join(cam0_dir, img_name)
        
        if not os.path.exists(img_path):
            continue
            
        # Load the image in grayscale
        img_gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        
        # --- COMPUTER VISION: FEATURE DETECTION ---
        # 1. Ask OpenCV to find the 100 best corners to track
        corners = cv2.goodFeaturesToTrack(img_gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
        
        # 2. Convert to color just so we can draw bright green dots on it
        img_display = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
        
        # 3. Draw a green circle over every detected corner
        if corners is not None:
            for corner in corners:
                x, y = corner.ravel()
                cv2.circle(img_display, (int(x), int(y)), 3, (0, 255, 0), -1)
        # ------------------------------------------
        
        # Display the drone's live point of view with features
        cv2.imshow("VI-SLAM Engine - Feature Tracking", img_display)
        
        if cv2.waitKey(30) & 0xFF == ord('q'):
            print("Engine stopped by user.")
            break
            
    cv2.destroyAllWindows()
    print("Playback complete.")

if __name__ == "__main__":
    run_slam_engine()