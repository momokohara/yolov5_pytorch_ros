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
from sensor_msgs.msg import LaserScan
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Bool


class ObjectTracker():

    def __init__(self):
        self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def ranges_callback(self, scan):
        msg = scan
        print("##################")
        #print(msg.ranges)

        for i in msg.ranges:
            if i <= 1.25:
                self.command = 0
            else:
                self.command = 1
        #print("##################")


        cmd_vel = Twist()
        if self.command == 1:   # None obstacle
            cmd_vel.linear.x = 0.2
            print("forward")
            print("_____________________None obstacle_________________________")
            print("##################")


        else: # Near obstacle
            cmd_vel.linear.x = 0
            print("stop")
            print("!!!!!!!!!obstacle stop!!!!!!!!!!!!")
            print("##################")

        self._pub_cmdvel.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('object_tracking')
    ot = ObjectTracker()
    rospy.Subscriber("/hokuyo_scan", LaserScan, ot.ranges_callback, queue_size = 1)
    rospy.spin()
