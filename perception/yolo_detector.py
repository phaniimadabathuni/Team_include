#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class YoloDetector:
    def __init__(self):
        rospy.init_node('team_include_yolo_detector', anonymous=True)
        
        # 1. Load the Model
        # We use the 'Nano' model for speed on the drone
        print("Loading YOLOv8 AI Model...")
        self.model = YOLO('yolov8n.pt') 
        
        self.bridge = CvBridge()

        # 2. Subscriber: Listen to the RealSense Color Feed
        # (We will remap this in the launch file later)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        # 3. Publisher: Broadcast what we see
        # Publishes text alerts: e.g., "Detected: person, chair"
        self.alert_pub = rospy.Publisher('/team_include/ai_alerts', String, queue_size=10)
        # Publishes the video feed with bounding boxes drawn on it
        self.debug_pub = rospy.Publisher('/team_include/ai_debug_video', Image, queue_size=1)

        print("YOLO Detector initialized. Waiting for video feed...")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(f"Error converting image: {e}")
            return

        # 4. Run AI Inference
        results = self.model(cv_image, verbose=False)

        # 5. Process Results
        detections = []
        annotated_frame = cv_image # Default fallback
        
        for result in results:
            # Get class names (e.g., 'person', 'car')
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                detections.append(class_name)

            # Draw the bounding boxes on the image
            annotated_frame = result.plot()

        # 6. Publish Alerts
        if detections:
            alert_msg = f"Detected: {', '.join(set(detections))}"
            self.alert_pub.publish(alert_msg)
            # print(alert_msg) # Uncomment for terminal debug

        # 7. Publish Debug Video
        try:
            ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.debug_pub.publish(ros_image)
        except Exception as e:
            print(f"Error publishing debug image: {e}")

if __name__ == '__main__':
    try:
        detector = YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass