import numpy as np
import matplotlib.pyplot as plt
import time

# THE GREAT HANDOFF: Importing Team #include's Modules
from slam_ekf import DroneEKF
from path_planner import a_star_search

def run_master_integration():
    print("Initializing Team #include Master Integration Node...")
    
    # 1. Initialize Phani's Domain (The EKF Brain)
    ekf = DroneEKF()
    
    # 2. Simulate Radha's Domain (The Map)
    # Note: In the future, this will be imported directly from her file 
    # via something like: map_grid = occupancy_mapper.get_live_map()
    grid_size = 20
    map_grid = np.zeros((grid_size, grid_size))
    map_grid[5:15, 8:12] = 1  # The solid wall obstacle
    
    # Mission Parameters
    start_pos = (2, 2)
    goal_pos = (18, 18)
    
    print("EKF Brain online. Map loaded. Mission parameters set.")
    
    # 3. Revanth's Domain: Calculate the global path around Radha's wall
    print("Asking Revanth's A* Engine for a flight path...")
    global_path = a_star_search(map_grid, start_pos, goal_pos)
    
    if not global_path:
        print("MISSION ABORT: No valid path found.")
        return

    print("Path secured! Engaging EKF Flight Loop...")
    
    # Setup live visualization (The Command Center Radar)
    plt.figure(figsize=(8, 8))
    plt.ion() # Turn on interactive mode to animate the drone flying
    
    # Memory to draw the trail of where the drone has actually flown
    ekf_x_history = []
    ekf_y_history = []
    
    # 4. THE MASTER LOOP: Fly the drone along the path
    for target_node in global_path:
        
        # We feed the EKF simulated acceleration data to step it forward
        # In the real lab, dataset_player.py will feed the raw IMU data here
        ekf.predict(accel_z=9.81, dt=0.1) 
        
        # For this integration test, we lock the EKF position to Revanth's path
        ekf.x[0, 0] = target_node[1] # X coordinate
        ekf.x[1, 0] = target_node[0] # Y coordinate
        
        ekf_x_history.append(ekf.x[0, 0])
        ekf_y_history.append(ekf.x[1, 0])
        
        # Update the live radar screen
        plt.clf() 
        plt.imshow(map_grid, cmap='binary', origin='lower')
        
        # Draw Revanth's planned route (Dashed Blue Line)
        route_y = [p[0] for p in global_path]
        route_x = [p[1] for p in global_path]
        plt.plot(route_x, route_y, 'b--', alpha=0.3, label="Revanth's Planned Route")
        
        # Draw Phani's actual EKF tracked flight (Solid Green Line)
        plt.plot(ekf_x_history, ekf_y_history, 'g-', linewidth=3, label="Phani's EKF Actual Flight")
        
        # Draw the Drone itself (Red Dot)
        plt.plot(ekf.x[0, 0], ekf.x[1, 0], 'ro', markersize=8, label="Drone Live Location")
        
        plt.title("Team #include: Master Sensor Fusion & Navigation Loop")
        plt.xlim(0, 19)
        plt.ylim(0, 19)
        plt.legend(loc="lower right")
        
        # Pause for a fraction of a second to animate the flight visually
        plt.pause(0.1) 
        
    plt.ioff()
    print("Target Reached. Master Loop execution successful.")
    plt.show()

if __name__ == "__main__":
    run_master_integration()