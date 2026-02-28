import numpy as np
import matplotlib.pyplot as plt

def generate_3d_to_2d_map():
    print("Initializing Radha's Domain: 3D Depth to 2D Occupancy Grid...")

    # 1. Simulate an EMPTY room with a floor (Z = 0)
    num_floor_points = 10000
    points_x = np.random.uniform(0, 20, num_floor_points)
    points_y = np.random.uniform(0, 20, num_floor_points)
    points_z = np.zeros(num_floor_points) # All points flat on the floor

    # 2. Build a solid 3D wall in the middle of the room
    num_wall_points = 25000
    wall_x = np.random.uniform(8, 12, num_wall_points)
    wall_y = np.random.uniform(5, 15, num_wall_points)
    wall_z = np.random.uniform(0, 3, num_wall_points) # Wall goes from floor to 3m ceiling

    # Combine the floor and the wall into one massive point cloud
    points_x = np.concatenate((points_x, wall_x))
    points_y = np.concatenate((points_y, wall_y))
    points_z = np.concatenate((points_z, wall_z))

    print("Massive 3D Point Cloud Received.")
    print("Filtering by drone's current flight altitude...")

    # 3. Slice the 3D data (The Drone is hovering at Z = 1.0 meters)
    flight_altitude = 1.0
    slice_tolerance = 0.2

    # Find points strictly at our flight altitude
    danger_zone = (points_z > (flight_altitude - slice_tolerance)) & \
                  (points_z < (flight_altitude + slice_tolerance))

    filtered_x = points_x[danger_zone]
    filtered_y = points_y[danger_zone]

    print(f"Sliced out {len(filtered_x)} critical obstacle points at altitude {flight_altitude}m.")

    # 4. Squash into a 2D Occupancy Grid (20x20 meters)
    grid_size = 20
    occupancy_grid = np.zeros((grid_size, grid_size))

    # Map the dangerous X, Y coordinates to grid cells
    for x, y in zip(filtered_x, filtered_y):
        grid_x = int(np.floor(x))
        grid_y = int(np.floor(y))
        if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
            occupancy_grid[grid_y, grid_x] += 1 

    # Normalize: Anything with points becomes 1 (Solid Wall), empty space remains 0 (White)
    occupancy_grid = np.clip(occupancy_grid, 0, 1)

    # 5. Visualize the 2D Map for Revanth
    plt.figure(figsize=(8, 8))
    # 'cmap=binary' makes 0=White (Free Space) and 1=Black (Obstacle)
    plt.imshow(occupancy_grid, cmap='binary', origin='lower', extent=[0, 20, 0, 20])
    plt.title("Radha's Map: 3D Point Cloud Squashed to 2D Grid")
    plt.xlabel("X Coordinate (meters)")
    plt.ylabel("Y Coordinate (meters)")
    
    # Plot the drone's position
    plt.plot(2, 2, 'go', markersize=10, label="Drone Start Position")
    
    plt.grid(color='cyan', linestyle='-', linewidth=0.5)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    generate_3d_to_2d_map()