#!/usr/bin/env python
# coding: utf-8


import rospy
import cv2
import math
import numpy as np
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes

class ObjectTracker():

    def __init__(self):
        self._cv_bridge = CvBridge()
        self._point_of_centroid = None
        self.image_width = 640
        self.image_height = 480

        # self._pub_cmdvel = rospy.Publisher("/icart_mini/cmd_vel", Twist, queue_size=1)
        self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


    def _calculate_centroid_point(self, msg):
        point = False
        # initialization
        box_xmin = 0
        box_xmax = 0
        box_ymin = 0
        box_ymax = 0
        probability = 0
        (x_center, y_center) = (0, 0)
        print("############")
#        bounding_box = len(msg.bounding_boxes)
#        if len(msg.bounding_boxes) >= 1:

        if(msg.bounding_boxes[0].Class != "None"):
            for box in msg.bounding_boxes:
                print("-------- for ----------")
                print(box)
#                print("box = " + str(box))
                box_xmin = float(box.xmin)
#                print("box_xmin : " + str(box_xmin))
                box_xmax = float(box.xmax)
#                print("box_xmax : " + str(box_xmax))
                box_ymin = float(box.ymin)
#                print("box_ymin : " + str(box_ymin))
                box_ymax = float(box.ymax)
#                print("box_ymax : " + str(box_ymax))
                probability = box.probability
                (x_center, y_center) = ((box_xmin + box_xmax)//2, (box_ymin + box_ymax)//2)
#                print("x_center : " + str(x_center))
#                print("y_center : " + str(y_center))

                point = (x_center, y_center)
                print(point)
                # print(type(point))
        else:
            # print("whiteline is not detected")
            point = False

        return point

    def _stop_threshold(self):
        stop_threshold = 48
        # not_stop_range = self.image_height - stop_threshold
        not_stop_range = stop_threshold
        return not_stop_range

    def _move_zone(self):
        #if self.point[1] <= self._stop_threshold():
        if self.point[1] >= self._stop_threshold():
            return True

    def _stop_zone(self):
        # if self.point[1] > self._stop_threshold():
        if self.point[1] < self._stop_threshold():
            return True

    def _rotation_velocity(self):
        VELOCITY = 0.25 * math.pi
        if self.point is False or self._stop_zone():
            return 0.0

        half_width = self.image_width / 2.0
        # pos_x_rate = (half_width - self.point[0]) / half_width
        pos_x_rate = (half_width - self.point[0]) / half_width
        pos_x_rate = pos_x_rate * -1
        rot_vel = pos_x_rate * VELOCITY
        return rot_vel
    
    def callback(self, msg):
        self.point = self._calculate_centroid_point(msg)

        if self._calculate_centroid_point is False:
            print("white_line is not detected")
        
        # print("center point calculate")
        cmd_vel = Twist()
        # print(type(self.point))
        # print(self.point)
        if type(self.point) == tuple:
            if self._move_zone():
                cmd_vel.linear.x = 0.1
                print("forward")
            if self._stop_zone():
                cmd_vel.linear.x = 0
                print("stay")
            cmd_vel.angular.z = self._rotation_velocity()
        else:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = self._rotation_velocity()
        self._pub_cmdvel.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('object_tracking')
    ot = ObjectTracker()
    rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, ot.callback, queue_size=1)
    rospy.spin()

