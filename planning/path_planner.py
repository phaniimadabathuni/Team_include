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
        if not self.current_pose or not self.target_goal:
            rospy.logwarn("Missing VINS data or Goal. Cannot plan path.")
            return

        # ==========================================
        # TODO: Insert your A* or RRT logic here!
        # You will use self.current_pose (start), self.target_goal (end), 
        # and self.obstacle_map to generate a list of safe waypoints.
        # ==========================================

        print("Path calculated. Publishing to Flight Controller...")
        
        # Create the ROS Path message
        safe_route = Path()
        safe_route.header.stamp = rospy.Time.now()
        safe_route.header.frame_id = "map"
        
        # Example: Adding a calculated waypoint to the path
        waypoint = PoseStamped()
        waypoint.pose.position.x = self.target_goal.position.x
        waypoint.pose.position.y = self.target_goal.position.y
        safe_route.poses.append(waypoint)

        # Broadcast the route to the flight controller
        self.path_pub.publish(safe_route)

if __name__ == '__main__':
    try:
        planner = AutonomousPathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass