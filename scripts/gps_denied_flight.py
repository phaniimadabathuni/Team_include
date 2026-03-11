#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

# Global state to track if drone is armed and in offboard mode
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def vins_odom_cb(msg):
    # TODO for Revanth: Extract the drone's current X, Y, Z from VINS-Fusion
    pass

def main():
    rospy.init_node('flight_control_node', anonymous=True)

    # Subscribing to Pixhawk status and VINS odometry
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/vins_estimator/odometry", Odometry, vins_odom_cb)

    # Publishing waypoints to the Pixhawk 6C
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # MAVROS requires a continuous stream of setpoints (at least 2Hz, 20Hz is safer)
    rate = rospy.Rate(20.0) 

    rospy.loginfo("Flight Control Node Started. Waiting for OFFBOARD mode...")

    # Main flight loop
    while not rospy.is_shutdown():
        # TODO for Revanth: Create a PoseStamped message and publish the next waypoint
        # Example: local_pos_pub.publish(target_pose)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass