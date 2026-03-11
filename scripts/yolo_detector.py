#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

def image_callback(msg):
    # TODO for Raj: Add YOLOv8 detection logic here!
    rospy.loginfo("Received image frame...")
    
    # TODO for Raj: If obstacle is < 2m, publish emergency brake to cmd_vel_pub

def main():
    rospy.init_node('yolo_perception', anonymous=True)
    
    # Subscribing to RealSense D435i
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    
    # Publishing Emergency dodges to Pixhawk
    cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", Twist, queue_size=10)
    
    rospy.loginfo("YOLO Perception Node Started. Waiting for video...")
    rospy.spin()

if __name__ == '__main__':
    main()