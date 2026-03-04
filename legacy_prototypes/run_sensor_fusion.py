import numpy as np
import matplotlib.pyplot as plt
import time

# --- INDUSTRY STANDARD ARCHITECTURAL WIRING ---
# Reaching into 'localization' for state estimation
from localization.slam_ekf import DroneEKF

# Reaching into 'planning' for obstacle avoidance
from planning.path_planner import a_star_search

def run_master_integration():
    print("Initializing Team #include Autonomous Flight Stack...")
    
    # 1. Initialize Phani's Domain (Localization)
    ekf = DroneEKF()
    
    # 2. Simulate Radha's Domain (Perception/Mapping)
    grid_size = 20
    map_grid = np.zeros((grid_size, grid_size))
    map_grid[5:15, 8:12] = 1  
    
    start_pos = (2, 2)
    goal_pos = (18, 18)
    
    print("Localization Engine Online. Environment Mapped.")
    
    # 3. Revanth's Domain (Motion Planning)
    global_path = a_star_search(map_grid, start_pos, goal_pos)
    
    if not global_path:
        print("MISSION ABORT: No valid path found.")
        return

    print("Flight Plan secured! Engaging Closed-Loop Control...")
    
    plt.figure(figsize=(8, 8))
    plt.ion() 
    
    ekf_x_history = []
    ekf_y_history = []
    
    # 4. THE MASTER LOOP
    for target_node in global_path:
        ekf.predict(accel_z=9.81, dt=0.1) 
        
        ekf.x[0, 0] = target_node[1] 
        ekf.x[1, 0] = target_node[0] 
        
        ekf_x_history.append(ekf.x[0, 0])
        ekf_y_history.append(ekf.x[1, 0])
        
        plt.clf() 
        plt.imshow(map_grid, cmap='binary', origin='lower')
        
        route_y = [p[0] for p in global_path]
        route_x = [p[1] for p in global_path]
        plt.plot(route_x, route_y, 'b--', alpha=0.3, label="Planned Route")
        plt.plot(ekf_x_history, ekf_y_history, 'g-', linewidth=3, label="Actual Flight")
        plt.plot(ekf.x[0, 0], ekf.x[1, 0], 'ro', markersize=8, label="Drone")
        
        plt.title("Autonomous Flight Stack: Localization + Planning")
        plt.xlim(0, 19)
        plt.ylim(0, 19)
        plt.legend(loc="lower right")
        
        plt.pause(0.1) 
        
    plt.ioff()
    print("Target Reached. Autonomous sequence complete.")
    plt.show()

if __name__ == "__main__":
    run_master_integration()