#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from ultralytics import YOLO

class RobotCamera():
    def __init__(self):
        rospy.init_node("follow_object_node")

        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_cb)
        
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
        self.speed = Twist()
        self.bridge = CvBridge()
        
        # Cargar el modelo YOLOv8
        self.model = YOLO("yolov8n.pt")  # Reemplaza "yolov8n.pt" con el modelo que desees usar
        
        rospy.spin()

    def camera_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (640, 480))
        
        ###################### DETECCIÓN DE OBJETOS ###############################
        results = self.model(frame)

        # Dibujar los resultados en la imagen
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                confidence = box.conf[0]
                class_id = box.cls[0]
                label = f'{self.model.names[int(class_id)]} {confidence:.2f}'

                # Dibujar el rectángulo alrededor del objeto detectado
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                # Poner la etiqueta con la confianza
                cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                
                # Calcular el centro del objeto detectado
                center_x_obj = (x1 + x2) / 2
                center_y_obj = (y1 + y2) / 2

                center_x = frame.shape[1] / 2
                center_y = frame.shape[0] / 2

                error_x = center_x_obj - center_x
                radius = (x2 - x1) / 2  # Aproximación del radio

                if radius <= 75:  # Ajustar este valor según sea necesario
                    self.speed.linear.x = 0.2  # Ajustar la velocidad lineal según sea necesario
                    self.speed.angular.z = -error_x / 1000  # Ajustar la constante para corregir el error
                    self.pub.publish(self.speed)
                    rospy.loginfo("Moviéndose hacia el objeto\n" + str(self.speed))
                elif radius > 75:
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = 0.0
                    self.pub.publish(self.speed)
                    rospy.loginfo("Deteniéndose")

        cv2.imshow("frame", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    RobotCamera()


