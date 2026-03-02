#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import SetMode, CommandBool

class GPSDeniedFlightController:
    def __init__(self):
        rospy.init_node('team_include_flight_control', anonymous=True)

        # 1. Subscriber: Listening to our custom Path Planner
        rospy.Subscriber('/team_include/planned_path', Path, self.path_callback)

        # 2. Publisher: Sending physical velocity commands to the Pixhawk via MAVROS
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        # 3. State Variables
        self.current_path = None
        self.rate = rospy.Rate(20) # We must send commands at 20Hz or ArduPilot will trigger a failsafe

        print("Team_include: Flight Controller online. Waiting for safe route...")

    def path_callback(self, msg):
        self.current_path = msg.poses
        print(f"Received route with {len(self.current_path)} waypoints. Engaging motors...")
        self.execute_flight()

    def execute_flight(self):
        if not self.current_path:
            return

        # Grab the immediate next waypoint on the route
        next_target = self.current_path[0].pose.position

        # ==========================================
        # TODO: Insert your PID Control Logic here!
        # Compare current VINS position to next_target, 
        # and calculate how fast to move in X, Y, and Z.
        # ==========================================

        cmd = TwistStamped()
        
        # Example: Move forward at 0.5 meters per second
        cmd.twist.linear.x = 0.5 
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0 # Maintain altitude
        
        # Keep publishing the velocity command to keep the drone moving
        while not rospy.is_shutdown():
            self.velocity_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        flight_controller = GPSDeniedFlightController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass