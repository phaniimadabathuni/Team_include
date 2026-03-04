import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

def compute_stereo_depth():
    print("Initializing Advanced SGBM Stereo Depth Perception...")
    
    path_left = r"C:\Users\phani\Drone_SLAM\mav0\cam0\data"
    path_right = r"C:\Users\phani\Drone_SLAM\mav0\cam1\data"
    
    if not os.path.exists(path_left) or not os.path.exists(path_right):
        print("ERROR: Could not find cam0 or cam1 folders.")
        return

    left_files = sorted([f for f in os.listdir(path_left) if f.endswith('.png')])
    right_files = sorted([f for f in os.listdir(path_right) if f.endswith('.png')])
    
    img_L = cv2.imread(os.path.join(path_left, left_files[500]), cv2.IMREAD_GRAYSCALE)
    img_R = cv2.imread(os.path.join(path_right, right_files[500]), cv2.IMREAD_GRAYSCALE)
    
    print("Stereo Images loaded. Engaging Semi-Global Block Matching (SGBM)...")
    
    # THE 10X UPGRADE: Tuning SGBM parameters for industrial environments
    window_size = 5
    min_disp = 0
    num_disp = 64 # Must be divisible by 16
    
    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        P1=8 * 3 * window_size**2,
        P2=32 * 3 * window_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )
    
    disparity = stereo.compute(img_L, img_R).astype(np.float32) / 16.0
    
    # Filter out the invalid pixels (the negative values)
    disparity[disparity <= 0] = 0.1 # Prevent divide by zero issues later
    
    disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, 
                                         norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    plt.title("Single Camera (The Illusion)")
    plt.imshow(img_L, cmap='gray')
    plt.axis('off')
    
    plt.subplot(1, 2, 2)
    plt.title("SGBM Disparity Map (Solid 3D Objects)")
    plt.imshow(disparity_normalized, cmap='magma') 
    plt.axis('off')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    compute_stereo_depth()