import cv2
import numpy as np

def run_vision_test():
    # 1. Initialize Webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Vision Test Running... Press 'q' to quit.")
    print("Window 1: Raw Feed + Obstacle Warning")
    print("Window 2: Canny Edge Detection (What the Map sees)")
    print("Window 3: Harris Corner Detection (What VINS sees)")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Flip frame for natural mirror view
        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # =================================================
        # TEST 1: Edge Detection (Used for Mapping)
        # =================================================
        # This shows structure (walls, doors) vs flat space
        edges = cv2.Canny(gray, 50, 150)

        # =================================================
        # TEST 2: Corner Detection (Used for VINS-Fusion)
        # =================================================
        # This detects 'features' to track movement
        corners = cv2.cornerHarris(gray, 2, 3, 0.04)
        corners = cv2.dilate(corners, None)
        
        # Create a copy to draw corners on
        corner_display = frame.copy()
        # Mark corners in RED (This is what VINS locks onto)
        corner_display[corners > 0.01 * corners.max()] = [0, 0, 255]

        # =================================================
        # TEST 3: Basic Obstacle Detection (Safety Zone)
        # =================================================
        # Define a 'Danger Zone' in the center
        center_x, center_y = width // 2, height // 2
        box_size = 100
        top_left = (center_x - box_size, center_y - box_size)
        bottom_right = (center_x + box_size, center_y + box_size)

        # Check edge density in the center box
        roi = edges[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
        edge_density = cv2.countNonZero(roi)
        
        color = (0, 255, 0) # Green (Safe)
        status = "PATH CLEAR"
        
        # If complex texture is in front, assume obstacle
        if edge_density > 500: 
            color = (0, 0, 255) # Red (Danger)
            status = "OBSTACLE!"

        cv2.rectangle(frame, top_left, bottom_right, color, 2)
        cv2.putText(frame, status, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        # Display
        cv2.imshow('1. Obstacle Logic', frame)
        cv2.imshow('2. Edges (Map Input)', edges)
        cv2.imshow('3. Features (VINS Input)', corner_display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_vision_test()