import cv2
import os
import numpy as np

def run_visual_odometry():
    print("Initializing Team #include Visual Odometry Engine...")
    
    # Path to your EuRoC Dataset Images
    dataset_path = r"C:\Users\phani\Drone_SLAM\mav0\cam0\data"
    
    if not os.path.exists(dataset_path):
        print(f"ERROR: Cannot find dataset at {dataset_path}")
        return

    image_files = sorted([f for f in os.listdir(dataset_path) if f.endswith('.png')])
    
    # 1. Initialize ORB Detector and Brute Force Matcher
    orb = cv2.ORB_create(nfeatures=500)
    # NORM_HAMMING is the standard mathematical distance metric for ORB features
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    
    # Memory variables to hold the previous frame's data
    prev_kp = None
    prev_desc = None
    
    print("Engaging Brute Force Feature Matching...")

    for image_name in image_files:
        img_path = os.path.join(dataset_path, image_name)
        frame = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        
        if frame is None:
            continue
            
        # Detect current features
        kp, desc = orb.detectAndCompute(frame, None)
        
        # Convert to color just so we can draw bright colored lines on it
        display_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        
        # 2. The Odometry Math: Match current features to previous features
        if prev_desc is not None and desc is not None:
            matches = bf.match(prev_desc, desc)
            # Sort them by distance so we only use the most accurate matches
            matches = sorted(matches, key=lambda x: x.distance)
            
            # 3. Draw the movement vectors (Optical Flow)
            # We take the top 100 strongest matches
            for match in matches[:100]: 
                # Get the pixel coordinates from the previous and current frame
                pt1 = tuple(np.round(prev_kp[match.queryIdx].pt).astype(int))
                pt2 = tuple(np.round(kp[match.trainIdx].pt).astype(int))
                
                # Draw a yellow line showing the direction of movement
                cv2.line(display_frame, pt1, pt2, (0, 255, 255), 2)
                # Draw a green dot at the current position
                cv2.circle(display_frame, pt2, 3, (0, 255, 0), -1)

        # Update HUD Text
        cv2.putText(display_frame, "V-I SLAM: Visual Odometry Active", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.imshow("Drone Visual Cortex (Velocity Vectors)", display_frame)
        
        # Save current data into memory for the next loop iteration
        prev_kp = kp
        prev_desc = desc
        
        if cv2.waitKey(30) & 0xFF == ord('q'):
            print("Playback terminated.")
            break

    cv2.destroyAllWindows()
    print("Visual Odometry sequence complete.")

if __name__ == "__main__":
    run_visual_odometry()