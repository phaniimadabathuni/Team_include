import cv2
import numpy as np
import matplotlib.pyplot as plt
import heapq
import os

# --- KEEP YOUR EXISTING a_star_search AND heuristic FUNCTIONS HERE ---
def heuristic(a, b):
    return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def a_star_search(grid, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
            
        close_set.add(current)
        
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
                
            move_cost = np.sqrt(i**2 + j**2)
            tentative_g_score = gscore[current] + move_cost
            
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
                
    return False

# --- THE ULTIMATE 3D INTEGRATION ---
def run_true_3d_planner():
    print("Initializing Team #include True 3D Navigation Engine...")
    
    # 1. Load the Stereo Eyes
    path_left = r"C:\Users\phani\Drone_SLAM\mav0\cam0\data"
    path_right = r"C:\Users\phani\Drone_SLAM\mav0\cam1\data"
    
    left_files = sorted([f for f in os.listdir(path_left) if f.endswith('.png')])
    right_files = sorted([f for f in os.listdir(path_right) if f.endswith('.png')])
    
    img_L = cv2.imread(os.path.join(path_left, left_files[500]), cv2.IMREAD_GRAYSCALE)
    img_R = cv2.imread(os.path.join(path_right, right_files[500]), cv2.IMREAD_GRAYSCALE)
    
    # 2. Compute SGBM Depth
    stereo = cv2.StereoSGBM_create(minDisparity=0, numDisparities=64, blockSize=5,
                                   P1=8 * 3 * 25, P2=32 * 3 * 25, disp12MaxDiff=1,
                                   uniquenessRatio=10, speckleWindowSize=100, speckleRange=32)
    
    disparity = stereo.compute(img_L, img_R).astype(np.float32) / 16.0
    disparity[disparity <= 0] = 0.1 
    disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
    # 3. Downsample for Navigation Grid
    grid_width = 80
    grid_height = 50
    small_depth_map = cv2.resize(disparity_normalized, (grid_width, grid_height))
    
    # 4. DEPTH THRESHOLDING: Create the Obstacle Matrix
    # Any pixel brighter than 110 (physically close to the drone) becomes a solid wall (1).
    # Everything else (far away) becomes safe air (0).
    _, binary_grid = cv2.threshold(small_depth_map, 110, 1, cv2.THRESH_BINARY)
    
    # 5. Set Mission Parameters
    start_pos = (5, 5) 
    goal_pos = (grid_height - 5, grid_width - 10) 
    
    # Clear the helipads to ensure we don't spawn inside a wall
    binary_grid[start_pos[0]-2:start_pos[0]+3, start_pos[1]-2:start_pos[1]+3] = 0
    binary_grid[goal_pos[0]-2:goal_pos[0]+3, goal_pos[1]-2:goal_pos[1]+3] = 0

    print("Running A* algorithm on solid 3D depth data...")
    path = a_star_search(binary_grid, start_pos, goal_pos)

    # 6. Visualize the Victory
    plt.figure(figsize=(10, 6))
    
    # Show the 3D Depth Occupancy Grid
    plt.imshow(binary_grid, cmap='binary')
    
    plt.plot(start_pos[1], start_pos[0], 'go', markersize=10, label="Drone Start")
    plt.plot(goal_pos[1], goal_pos[0], 'ro', markersize=10, label="Target Goal")
    
    if path:
        print("Route successfully calculated through 3D space!")
        route_y = [p[0] for p in path]
        route_x = [p[1] for p in path]
        plt.plot(route_x, route_y, 'b-', linewidth=3, label="A* Flight Path")
    else:
        print("Mission Abort: No safe path available.")
        
    plt.title("Master Integration: 3D Stereo Depth + A* Navigation")
    plt.legend(loc="upper right")
    plt.show()

if __name__ == "__main__":
    run_true_3d_planner()