#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry  # Importamos Odometry desde nav_msgs.msg
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
from random import randrange
import time
import math

class RobotExplorer:
    def __init__(self):
        rospy.init_node("explore_and_follow_bot_node", log_level=rospy.DEBUG)

        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)  # Suscriptor para la odometría
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)  # Suscriptor para el LIDAR
        
        # Publishers
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
        # Action Client
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready") 
        
        # Other attributes
        self.bridge = CvBridge()
        self.speed = Twist()
        self.explore_mode = True
        self.exploring = False
        self.x = 0
        self.y = 0
        self.completion = 0
        self.map = OccupancyGrid()
        self.count = 0
        self.frame = None
        self.visitas = {}
        self.amarillo = False
        self.verde = False
        self.azul = False
        self.rojo = False
        self.current_x = 0
        self.current_y = 0
        self.obstacle_detected = False
        self.obstacle_distance_threshold = 0.1  # Ajusta este valor según la distancia mínima de seguridad
        self.obstacle_side = None

        # Timer to display the frame regularly
        rospy.Timer(rospy.Duration(0.1), self.show_frame)

        rospy.spin()
        
    def odometry_callback(self, odom):
        # Método callback para recibir la odometría del robot
        self.current_x = odom.pose.pose.position.x
        self.current_y = odom.pose.pose.position.y

    def map_callback(self, data):
        if self.explore_mode:
            valid = False
            while valid is False:
                map_size = randrange(len(data.data))
                self.map = data.data[map_size]

                edges = self.check_neighbors(data, map_size)
                if self.map != -1 and self.map <= 0.2 and edges is True:
                    valid = True

            row = map_size // 384
            col = map_size % 384

            self.x = col * 0.05 - 10  # column * resolution + origin_x
            self.y = row * 0.05 - 10  # row * resolution + origin_x
            
            if self.completion % 2 == 0:
                self.completion += 1
                self.set_goal()

    def set_goal(self, goal_x=None, goal_y=None):
        if goal_x is None:
            goal_x = self.x
        if goal_y is None:
            goal_y = self.y
        rospy.logdebug("Setting goal")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.w = 1.0
        rospy.logdebug(f"goal: {goal.target_pose.pose.position.x, goal.target_pose.pose.position.y}")
        self.move_base.send_goal(goal, self.goal_status)
        self.azul = False 
        self.rojo = False

    def goal_status(self, status, result):
        rospy.loginfo(f"Goal {status}, {result}")
        self.completion += 1
        if status == 3:
            rospy.loginfo("Goal succeeded")
        elif status == 4:
            rospy.loginfo("Goal aborted")
        elif status == 5:
            rospy.loginfo("Goal rejected")

    def check_neighbors(self, data, map_size):
        unknowns = 0
        obstacles = 0
        for x in range(-3, 4):
            for y in range(-3, 4):
                row = x * 384 + y
                try:
                    if data.data[map_size + row] == -1:
                        unknowns += 1
                    elif data.data[map_size + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
        return unknowns > 0 and obstacles < 2

    def scan_callback(self, data):
        # Detectar el obstáculo más cercano
        min_distance = min(data.ranges)
        if min_distance < self.obstacle_distance_threshold:
            self.obstacle_detected = True
            min_index = data.ranges.index(min_distance)
            if min_index < len(data.ranges) / 2:
                self.obstacle_side = "left"
            else:
                self.obstacle_side = "right"
        else:
            self.obstacle_detected = False
            self.obstacle_side = None

    def camera_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.frame = cv2.resize(self.frame, (640, 480))

        if not self.azul and not self.rojo:  
        

    

            # Definimos los rangos de color para el color1 y color2 en HSV
            if not self.amarillo:
            	lower_color1 = np.array([20, 100, 100])
            	upper_color1 = np.array([30, 255, 255])
            else:
            	lower_color1 = np.array([100, 100, 100])
            	upper_color1 = np.array([120, 255, 255])
            
            
            if not self.verde:
            	lower_color2 = np.array([40, 100, 100])
            	upper_color2 = np.array([80, 255, 255])

            else:
            	lower_color2 = np.array([0, 100, 100])
            	upper_color2 = np.array([10, 255, 255])


            
            
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        
            # Mascara para el color1
            mask_color1 = cv2.inRange(hsv, lower_color1, upper_color1)
            mask_color1 = cv2.erode(mask_color1, (5, 5), iterations=9)
            mask_color1 = cv2.medianBlur(mask_color1, 7)
            mask_color1 = cv2.morphologyEx(mask_color1, cv2.MORPH_OPEN, (5, 5))
            mask_color1 = cv2.dilate(mask_color1, (5, 5), iterations=1)
        
            # Mascara para el color2
            mask_color2 = cv2.inRange(hsv, lower_color2, upper_color2)
            mask_color2 = cv2.erode(mask_color2, (5, 5), iterations=9)
            mask_color2 = cv2.medianBlur(mask_color2, 7)
            mask_color2 = cv2.morphologyEx(mask_color2, cv2.MORPH_OPEN, (5, 5))
            mask_color2 = cv2.dilate(mask_color2, (5, 5), iterations=1)

            _, thresh_color1 = cv2.threshold(mask_color1, 127, 255, cv2.THRESH_BINARY)
            _, thresh_color2 = cv2.threshold(mask_color2, 127, 255, cv2.THRESH_BINARY)
    
            cnts_color1, _ = cv2.findContours(thresh_color1, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
            cnts_color2, _ = cv2.findContours(thresh_color2, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        
            center_x = self.frame.shape[1] / 2
            center_y = self.frame.shape[0] / 2

            # Verificamos si hemos encontrado ambos colores
            found_color1 = len(cnts_color1) > 0
            found_color2 = len(cnts_color2) > 0
        
            if found_color1 or found_color2:
                # Si se encuentra color1 o color2, dibujamos el círculo y calculamos error_x
                if found_color1:
                    c1 = max(cnts_color1, key=cv2.contourArea)
                    ((x1, y1), radius) = cv2.minEnclosingCircle(c1)
                    cv2.circle(self.frame, (int(x1), int(y1)), int(radius), (0, 255, 255), 2)  # Yellow color for circle
                    cv2.putText(self.frame, "X1: " + str(round(x1, 2)), (int(x1) + int(radius) + 5, int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(self.frame, "Y1: " + str(round(y1, 2)), (int(x1) + int(radius) + 5, int(y1) + 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.line(self.frame, (int(center_x), int(center_y)), (int(x1), int(y1)), (0, 0, 0), 3)
                
                    error_x = int(x1) - int(center_x)
                    error_theta = math.atan2(y1 - center_y, x1 - center_x)
            
                elif found_color2:
                    c2 = max(cnts_color2, key=cv2.contourArea)
                    ((x2, y2), radius) = cv2.minEnclosingCircle(c2)
                    cv2.circle(self.frame, (int(x2), int(y2)), int(radius), (0, 255, 255), 2)  # Yellow color for circle
                    cv2.putText(self.frame, "X2: " + str(round(x2, 2)), (int(x2) + int(radius) + 5, int(y2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(self.frame, "Y2: " + str(round(y2, 2)), (int(x2) + int(radius) + 5, int(y2) + 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.line(self.frame, (int(center_x), int(center_y)), (int(x2), int(y2)), (0, 0, 0), 3)
                
                    error_x = int(x2) - int(center_x)
                    error_theta = math.atan2(y2 - center_y, x2 - center_x)
 
                if radius <= 140:
                    self.explore_mode = False
                    distancia_objeto = 1.0  # Distancia deseada hacia adelante desde el robot (ajustar según necesidad)
                    target_x = self.current_x + distancia_objeto * math.cos(error_theta)
                    target_y = self.current_y + distancia_objeto * math.sin(error_theta)
                    rospy.loginfo(f"Current position: ({self.current_x:.2f}, {self.current_y:.2f})")
                    rospy.loginfo(f"Target position: ({target_x:.2f}, {target_y:.2f})")

                    # Utiliza el punto objetivo en tu lógica
                    self.set_goal(target_x, target_y)
                    
                elif radius > 140:
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = 0.0
                    self.cmd_pub.publish(self.speed)
                    
                    
                    
                    if found_color1 and self.amarillo:
                        if self.visitas:
                            self.set_goal(self.visitas['Amarillo'][0], self.visitas['Amarillo'][1])
                        self.azul = True
                        
                    elif found_color2 and self.verde:
                        if self.visitas:
                            self.set_goal(self.visitas['Verde'][0], self.visitas['Verde'][1])
                        self.rojo = True
                        
                    elif found_color1 and not self.amarillo: 
                    	 self.visitas['Amarillo']= (self.current_x, self.current_y)
                    	 self.amarillo = True
                    	 
                    elif found_color2 and not self.verde: 
                        self.visitas['Verde']= (self.current_x, self.current_y)
                        self.verde = True
                        
                    rospy.loginfo(f"Object reached {str(self.visitas)}")
                    
                    self.explore_mode = True

    def show_frame(self, event):
        if self.frame is not None:
            cv2.imshow("Camera View", self.frame)
            cv2.waitKey(1)

if __name__ == '__main__':
    try:
        RobotExplorer()
    except rospy.ROSInterruptException:
        pass


