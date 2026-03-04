import cv2
import numpy as np
import os

def run_optical_flow():
    print("Initializing Bulletproof Self-Healing Optical Flow...")

    dataset_path = r"C:\Users\phani\Drone_SLAM\dataset\mav0\cam0\data"
    if not os.path.exists(dataset_path):
        print(f"ERROR: Cannot find dataset at {dataset_path}")
        return

    image_files = sorted([f for f in os.listdir(dataset_path) if f.endswith('.png')])
    
    feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
    lk_params = dict(winSize=(21, 21), maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    color = np.random.randint(0, 255, (100, 3))

    old_frame_path = os.path.join(dataset_path, image_files[0])
    old_gray = cv2.imread(old_frame_path, cv2.IMREAD_GRAYSCALE)
    
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
    mask = np.zeros((old_gray.shape[0], old_gray.shape[1], 3), dtype=np.uint8)

    print("Tracking engaged. Press 'q' to stop.")

    for i in range(1, len(image_files)):
        frame_path = os.path.join(dataset_path, image_files[i])
        frame_gray = cv2.imread(frame_path, cv2.IMREAD_GRAYSCALE)
        frame_color = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

        # Safety Check: Did we lose all points entirely?
        if p0 is not None:
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
            
            if p1 is not None:
                good_new = p1[st == 1]
                good_old = p0[st == 1]
            else:
                good_new = []
                good_old = []
        else:
            good_new = []
            good_old = []

        # --- THE TIME-ADVANCING REFRESH ---
        if len(good_new) < 40:
            print(f"Frame {i}: Tracking lost. Re-initializing features...")
            
            # 1. Scan the new frame for fresh points
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
            
            # 2. Advance time forward
            old_gray = frame_gray.copy()
            
            # 3. Wipe the old mask completely
            mask = np.zeros((frame_gray.shape[0], frame_gray.shape[1], 3), dtype=np.uint8)
            
            continue # Skip drawing this frame to let new points settle

        # Draw the tracks
        for i_track, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            
            c_idx = i_track % 100 
            mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[c_idx].tolist(), 2)
            frame_color = cv2.circle(frame_color, (int(a), int(b)), 5, color[c_idx].tolist(), -1)

        img = cv2.add(frame_color, mask)
        cv2.imshow("Optical Flow (Lucas-Kanade) - Drone Cam", img)
        
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

        # Advance time forward for the standard loop
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)

    cv2.destroyAllWindows()
    print("Optical Flow tracking complete.")

if __name__ == "__main__":
    run_optical_flow()