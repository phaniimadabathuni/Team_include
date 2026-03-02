#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

class AutonomousPathPlanner:
    def __init__(self):
        rospy.init_node('team_include_path_planner', anonymous=True)
        
        # 1. State Variables
        self.current_pose = None
        self.target_goal = None
        self.obstacle_map = None

        # 2. Subscribers (Listening to the Environment)
        # Listening to VINS-Fusion for exact location
        rospy.Subscriber('/vins_estimator/odometry', Odometry, self.vins_callback)
        # Listening to the target destination (e.g., from your ground station)
        rospy.Subscriber('/team_include/mission_goal', PoseStamped, self.goal_callback)
        # Listening to the 2D obstacle map (from RealSense depth data)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # 3. Publisher (Broadcasting the Route)
        self.path_pub = rospy.Publisher('/team_include/planned_path', Path, queue_size=1)
        
        print("Team_include: Path Planner initialized. Waiting for VINS and Mission Goal...")

    def vins_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.target_goal = msg.pose
        print("New mission goal received! Calculating route...")
        self.calculate_astar_path()

    def map_callback(self, msg):
        self.obstacle_map = msg

    def calculate_astar_path(self):
        if not self.current_pose or not self.target_goal or not self.obstacle_map:
            rospy.logwarn("Missing VINS data, Goal, or Map. Cannot plan path.")
            return

        print("Calculating optimal A* route...")

        # 1. Define the grid resolution and size based on your OccupancyGrid
        resolution = self.obstacle_map.info.resolution
        origin_x = self.obstacle_map.info.origin.position.x
        origin_y = self.obstacle_map.info.origin.position.y
        width = self.obstacle_map.info.width

        # Helper function to convert real-world (X,Y) to Grid (Col, Row)
        def world_to_grid(x, y):
            grid_x = int((x - origin_x) / resolution)
            grid_y = int((y - origin_y) / resolution)
            return (grid_x, grid_y)

        # Helper function to convert Grid (Col, Row) back to real-world (X,Y)
        def grid_to_world(grid_x, grid_y):
            x = (grid_x * resolution) + origin_x
            y = (grid_y * resolution) + origin_y
            return (x, y)

        start_grid = world_to_grid(self.current_pose.position.x, self.current_pose.position.y)
        goal_grid = world_to_grid(self.target_goal.position.x, self.target_goal.position.y)

        # 2. The A* Algorithm Setup
        open_set = {start_grid}
        came_from = {}
        
        # g_score: Cost from start to current node
        g_score = {start_grid: 0}
        
        # Helper: Calculate distance (heuristic)
        def heuristic(a, b):
            return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

        # f_score: g_score + heuristic (estimated total cost)
        f_score = {start_grid: heuristic(start_grid, goal_grid)}

        safe_route = Path()
        safe_route.header.stamp = rospy.Time.now()
        safe_route.header.frame_id = "map"

        # 3. The Search Loop
        while open_set:
            # Get the node in open_set with the lowest f_score
            current = min(open_set, key=lambda o: f_score.get(o, float('inf')))

            if current == goal_grid:
                # We found the target! Reconstruct the path backwards
                path_waypoints = []
                while current in came_from:
                    path_waypoints.append(current)
                    current = came_from[current]
                
                # Reverse the list so it goes from Start to Goal
                path_waypoints.reverse()

                # Convert grid points back to real-world coordinates for the Pixhawk
                for wp in path_waypoints:
                    pose = PoseStamped()
                    world_x, world_y = grid_to_world(wp[0], wp[1])
                    pose.pose.position.x = world_x
                    pose.pose.position.y = world_y
                    safe_route.poses.append(pose)
                
                self.path_pub.publish(safe_route)
                print(f"Path published! {len(safe_route.poses)} waypoints generated.")
                return

            open_set.remove(current)

            # Check all 8 neighbors (up, down, left, right, diagonals)
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Check if neighbor is an obstacle (Value > 50 in ROS OccupancyGrid)
                # Ensure we don't check outside the map boundaries
                map_index = neighbor[1] * width + neighbor[0]
                if map_index < 0 or map_index >= len(self.obstacle_map.data) or self.obstacle_map.data[map_index] > 50:
                    continue # It's a wall, ignore this neighbor

                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # This path to neighbor is better than any previous one
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal_grid)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

        rospy.logwarn("A* Search failed to find a path to the goal. Drone holding position.")