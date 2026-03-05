#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class VinsToMavrosBridge:
    def __init__(self):
        rospy.init_node('vins_to_mavros_bridge', anonymous=True)
        
        # 1. Publisher: Where we send data TO the Pixhawk (ArduPilot Vision Pose)
        self.mavros_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        
        # 2. Subscriber: Where we get data FROM VINS-Fusion
        self.vins_sub = rospy.Subscriber('/vins_estimator/odometry', Odometry, self.vins_callback)
        
        print("Team #include: VINS-to-MAVROS Bridge Initialized. Waiting for odometry data...")

    def vins_callback(self, msg):
        # 3. Create a new PoseStamped message for MAVROS
        mavros_pose = PoseStamped()
        
        # Add the current timestamp (critical for ArduPilot's EKF)
        mavros_pose.header.stamp = rospy.Time.now()
        mavros_pose.header.frame_id = "map"
        
        # 4. Copy the Position (X, Y, Z)
        mavros_pose.pose.position.x = msg.pose.pose.position.x
        mavros_pose.pose.position.y = msg.pose.pose.position.y
        mavros_pose.pose.position.z = msg.pose.pose.position.z
        
        # 5. Copy the Orientation (Quaternion)
        mavros_pose.pose.orientation.x = msg.pose.pose.orientation.x
        mavros_pose.pose.orientation.y = msg.pose.pose.orientation.y
        mavros_pose.pose.orientation.z = msg.pose.pose.orientation.z
        mavros_pose.pose.orientation.w = msg.pose.pose.orientation.w
        
        # 6. Publish the translated data to the Pixhawk
        self.mavros_pub.publish(mavros_pose)

if __name__ == '__main__':
    try:
        bridge = VinsToMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass