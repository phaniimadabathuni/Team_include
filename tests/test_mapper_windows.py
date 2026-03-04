import cv2
import numpy as np

class WindowsMapperTest:
    def __init__(self):
        self.resolution = 0.1  # 10cm per grid cell
        self.width = 100       # 10x10 meter map
        self.height = 100
        
        # Initialize Webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open webcam.")
            return

        print("Mapper Test Running... Press 'q' to quit.")
        print("Bright objects will appear as OBSTACLES on the grid.")

        self.run()

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # Flip for mirror effect
            frame = cv2.flip(frame, 1)

            # 1. CREATE FAKE DEPTH DATA
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Invert colors: usually camera sees light as high values (255). 
            # In depth maps, close objects are usually low values (0) or high values depending on encoding.
            # Let's assume BRIGHT = CLOSE (Obstacle) for this test.
            # We treat pixels > 200 brightness as "1 meter away", and < 50 as "Infinite far".
            
            # 2. SLICE THE IMAGE (The Logic from occupancy_mapper.py)
            center_row_start = int(gray.shape[0] * 0.4)
            center_row_end = int(gray.shape[0] * 0.6)
            depth_slice = gray[center_row_start:center_row_end, :]

            # 3. INITIALIZE BLANK GRID (Gray background)
            # 127 = Unknown, 0 = Free (White), 255 = Occupied (Black) for visualization
            grid_img = np.full((self.height * 5, self.width * 5), 127, dtype=np.uint8)

            # 4. POPULATE THE GRID (The Math)
            for col in range(depth_slice.shape[1]):
                # Get the brightest pixel in this column (Simulating closest object)
                max_brightness = np.max(depth_slice[:, col])
                
                # Threshold: Only treat it as an object if it's bright enough
                if max_brightness < 100: 
                    continue # Too dark, assume empty space

                # Simulate Distance: Brighter = Closer
                # Map 255 (Bright) to 0.5 meters, 100 (Dim) to 5.0 meters
                simulated_dist_m = 5.0 - ((max_brightness - 100) / 155.0 * 4.5)
                
                # Project to Grid Coordinates
                map_x = int((col / depth_slice.shape[1]) * self.width)
                map_y = int(simulated_dist_m / self.resolution)

                # Draw on the Visualization Image (scaling up by 5 for visibility)
                # We draw a line from the bottom (drone) to the obstacle
                
                # Visualization scaling
                vis_x = map_x * 5
                vis_y = (self.height - map_y) * 5 # Flip Y so drone is at bottom

                # Draw the obstacle (Black Block)
                if 0 <= vis_x < self.width * 5 and 0 <= vis_y < self.height * 5:
                    cv2.circle(grid_img, (vis_x, vis_y), 3, (0), -1)
                    
                    # Draw "Free Space" (White) below the obstacle
                    cv2.line(grid_img, (vis_x, self.height*5), (vis_x, vis_y), (255), 1)

            # 5. DISPLAY
            # Draw the "Slice" lines on the original camera feed so you see what it's analyzing
            cv2.line(frame, (0, center_row_start), (frame.shape[1], center_row_start), (0, 255, 0), 2)
            cv2.line(frame, (0, center_row_end), (frame.shape[1], center_row_end), (0, 255, 0), 2)

            cv2.imshow('1. Webcam Feed (Green Lines = Scan Area)', frame)
            cv2.imshow('2. Generated Occupancy Grid (Top-Down View)', grid_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    WindowsMapperTest()