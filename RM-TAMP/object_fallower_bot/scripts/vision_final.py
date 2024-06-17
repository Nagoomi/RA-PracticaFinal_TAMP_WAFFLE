#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
from ultralytics import YOLO
import os
import math

class RobotCamera():
    def __init__(self):
        rospy.init_node("follow_object_node")

        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_cb)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.speed = Twist()
        self.bridge = CvBridge()

        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, 'best.pt')
        
        rospy.loginfo(f"Script directory: {script_dir}")
        rospy.loginfo(f"Model path: {model_path}")
        
        self.model = YOLO(model_path)
        
        rospy.spin()

    def camera_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (640, 480))
        
        results = self.model(frame)

        if results:
            closest_object_dist = float('inf')
            closest_object_coords = None

            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    confidence = box.conf[0]
                    class_id = box.cls[0]
                    label = f'{self.model.names[int(class_id)]} {confidence:.2f}'

                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                    cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

                    center_x_obj = (x1 + x2) / 2
                    center_y_obj = (y1 + y2) / 2

                    center_x = frame.shape[1] / 2
                    center_y = frame.shape[0] / 2
                    
                    

                    error_x = center_x_obj - center_x
                    object_dist = math.sqrt(error_x ** 2 + (center_y_obj - center_y) ** 2)
                    
                    rospy.loginfo(f"Distance: {object_dist}")

                    if object_dist < closest_object_dist:
                        closest_object_dist = object_dist
                        closest_object_coords = (center_x_obj, center_y_obj)

            if closest_object_coords:
                target_x, target_y = closest_object_coords
                dist_threshold = 35  # Threshold distance to stop before the object

                if closest_object_dist > dist_threshold:
                    self.speed.linear.x = 0.01
                    self.speed.angular.z = -error_x / 1000
                    self.pub.publish(self.speed)
                    rospy.loginfo("Moving towards the object\n" + str(self.speed))
                else:
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = 0.0
                    self.pub.publish(self.speed)
                    rospy.loginfo("Stopping near the object")

        else:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            self.pub.publish(self.speed)
            rospy.loginfo("No objects detected, stopping")

        cv2.imshow("frame", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    RobotCamera()







