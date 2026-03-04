import cv2
import os
import glob

def run_orb_tracking():
    print("Initializing Raj's Domain: ORB Feature Tracking...")

    # Path to your EuRoC Camera 0 data
    cam0_path = r"C:\Users\phani\Drone_SLAM\mav0\cam0\data"
    
    # Grab all PNG images and sort them by timestamp
    image_files = sorted(glob.glob(os.path.join(cam0_path, "*.png")))
    
    if not image_files:
        print("Error: Could not find images. Check your folder path!")
        return

    # 1. Initialize the ORB Engine (Finding 500 corners per frame)
    orb = cv2.ORB_create(nfeatures=500)

    # 2. Initialize the Brute Force Matcher (To connect the dots between frames)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    prev_img = None
    prev_kp = None
    prev_des = None

    print("Starting video feed. Click the video window and press 'q' to stop.")

    for img_path in image_files:
        # Read the image in grayscale (color wastes processing power for SLAM)
        curr_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

        # Step A: Detect corners (Keypoints) and their fingerprints (Descriptors)
        curr_kp, curr_des = orb.detectAndCompute(curr_img, None)

        # Step B: Match with the previous frame
        if prev_des is not None and curr_des is not None:
            # Find the matching fingerprints
            matches = bf.match(prev_des, curr_des)
            
            # Sort them by distance (closest matches are the most accurate)
            matches = sorted(matches, key=lambda x: x.distance)

            # Draw lines connecting the top 50 most accurate matches
            tracking_img = cv2.drawMatches(
                prev_img, prev_kp, 
                curr_img, curr_kp, 
                matches[:50], None, 
                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
            
            # Display the visual cortex
            cv2.imshow("Team #include: ORB Vision Tracker", tracking_img)
        else:
            # For the very first frame, just draw the dots
            display_img = cv2.drawKeypoints(curr_img, curr_kp, None, color=(0,255,0))
            cv2.imshow("Team #include: ORB Vision Tracker", display_img)

        # Wait 30ms before the next frame (simulates ~30fps video)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

        # Save current frame data to compare with the next one
        prev_img = curr_img
        prev_kp = curr_kp
        prev_des = curr_des

    cv2.destroyAllWindows()
    print("Vision tracking complete.")

if __name__ == "__main__":
    run_orb_tracking()