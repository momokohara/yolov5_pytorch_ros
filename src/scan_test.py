#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import sensor_msgs.msg import LaserScan

class ObjectTracker():
    
    def __init__(self):
        rospy.init_node("object_trancking")
        self.Subscriber("/scan", LaserScan, self.scan_callback, queue_size = 1)
        
    def scan_callback(self, msg):
        print(len(msg.ranges))

if __name__ == "__main__":
    ot = ObjectTracker()
    rospy.spin()
