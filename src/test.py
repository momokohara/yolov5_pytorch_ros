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
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan


class ObjectTracker():

    def __init__(self):
        self.whiteline = False

        rospy.init_node('object_tracking')
        self._cv_bridge = CvBridge()
        self._point_of_centroid = None
        self.image_width = 640
        self.image_height = 480
        #publisher
        #self._pub_cmdvel = rospy.Publisher("/icart_mini/cmd_vel", Twist, queue_size=1)
        self._pub_cmdvel = rospy.Publisher("/white_vel", Twist, queue_size=1)
        self._pub_bool = rospy.Publisher("/white_success", Bool, queue_size=1)
        #subscriber
        rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, self.boundingbox_callback, queue_size = 1)
        rospy.Subscriber("/start_white", Bool, self.main_callback, queue_size = 1)
        rospy.Subscriber("/hokuyo_scan", LaserScan, self.scan_callback, queue_size = 1)

    def boundingbox_callback(self, msg):
        self.point = False
        self.msg_boundingboxes = msg

        """
        # initialization
        self.box_xmin = 0
        self.box_xmax = 0
        self.box_ymin = 0
        self.box_ymax = 0
        self.probability = 0
        self.(x_center, y_center) = (0, 0)
        self.msg_boundingboxes = msg
        
        print("############")
        
        if(msg.bounding_boxes[0].Class != "None"):
            for box in msg.bounding_boxes:
                if box.Class == "white_line":
                    print("-------- for ----------")
                    print(box)
                    point = True
#                    print("box = " + str(box))
                    box_xmin = float(box.xmin)
#                	print("box_xmin : " + str(box_xmin))
                    box_xmax = float(box.xmax)
#                	print("box_xmax : " + str(box_xmax))
                    box_ymin = float(box.ymin)
#                	print("box_ymin : " + str(box_ymin))
                    box_ymax = float(box.ymax)
#                	print("box_ymax : " + str(box_ymax))
                    probability = box.probability
                    (x_center, y_center) = ((box_xmin + box_xmax)//2, (box_ymin + box_ymax)//2)
#                	print("x_center : " + str(x_center))
#                	print("y_center : " + str(y_center))
                    self.point = (x_center, y_center)
                    print(self.point)
                    print(type(self.point))
                else:
                    # print("whiteline is not detected")
            	    self.point = False

        else:
                self.point = False
        """

    def _stop_threshold(self):
        stop_threshold = 48
        not_stop_range = self.image_height - stop_threshold
        return not_stop_range

    def _move_zone(self):
        if self.point[1] <= self._stop_threshold():
            return True

    def _stop_zone(self):
        if self.point[1] > self._stop_threshold():
            return True

    def _rotation_velocity(self):
        VELOCITY = 0.25 * math.pi
        if self.point is False or self._stop_zone():
            return 0.0

        half_width = self.image_width / 2.0
        pos_x_rate = (half_width - self.point[0]) / half_width
        rot_vel = pos_x_rate * VELOCITY
        return rot_vel

    def scan_callback(self, scan):
        self.msg = scan
        """
        print("##################")
        print(min(msg.ranges))
        if min(msg.ranges) <= 0.05:
            self.command = 0
            print("near obstacle")
        else:
            self.command = 1
        print("##################")
        """
    
    def main_callback(self, msg):
        self.whiteline = True

        while self.whiteline == True:

            # whiteline recognition
            # initialization
            box_xmin = 0
            box_xmax = 0
            box_ymin = 0
            box_ymax = 0
            probability = 0
            (x_center, y_center) = (0, 0)
        
            print("############")
            if(self.msg_boundingboxes.bounding_boxes[0].Class != "None"):
                 for box in self.msg_boundingboxes.bounding_boxes:
#!/usr/bin/env python
# coding: utf-8                    if box.Class == "white_line":
                        print("-------- for ----------")
                        print(box)
                        self.point = True
#                    print("box = " + str(box))
                        box_xmin = float(box.xmin)
#                	print("box_xmin : " + str(box_xmin))
                        box_xmax = float(box.xmax)
#                	print("box_xmax : " + str(box_xmax))
                        box_ymin = float(box.ymin)
#                	print("box_ymin : " + str(box_ymin))
                        box_ymax = float(box.ymax)
#                	print("box_ymax : " + str(box_ymax))
                        probability = box.probability
                        (x_center, y_center) = ((box_xmin + box_xmax)//2, (box_ymin + box_ymax)//2)
#                	print("x_center : " + str(x_center))
#                	print("y_center : " + str(y_center))
                        self.point = (x_center, y_center)
                        print(self.point)
                        print(type(self.point))
                    else:
                    # print("whiteline is not detected")
            	        self.point = False

            else:
                self.point = False


            # judge ranges
            #print(len(self.msg.ranges))
            #print(self.msg.ranges[360:432])
            #print(min(self.msg.ranges))
            front_ranges = self.msg.ranges[360:432]
            min_front_ranges = min(front_ranges)
            print(front_ranges)
            print(min_front_ranges)
            #if min(self.msg.ranges) <= 0.45:
            if min_front_ranges <= 0.45:
                self.command = 1
            else:
                self.command = 0



            cmd_vel = Twist()
            bool_msg = Bool()
            
            if self.command == 0: #None obstacle
                if type(self.point) == tuple:
                
                    if self._move_zone():
                        cmd_vel.linear.x = 0.2
                        print("forward")
                    elif self._stop_zone():
                        cmd_vel.linear.x = 0
                        bool_msg.data = True
                        print("stay")
                        self.whiteline = False
                    cmd_vel.angular.z = self._rotation_velocity()
                    """
               else:
                    cmd_vel.linear.x = 0
                    print("!!!!!!!!!!!!!!!obstacle stop!!!!!!!!!!!!!!!!!!!!!!!!!")
                    """

                else:
                    print("!!!!!!!!white_line is not detected. Start search whiteline!!!!!!!!")
                    cmd_vel.linear.x = 0.2


            else: #Near obstacle
                cmd_vel.linear.x = 0
                print("!!!!!!!!!!!!!!!obstacle stop!!!!!!!!!!!!!!!!!!!!!!!!!")



            
            self._pub_cmdvel.publish(cmd_vel)
            self._pub_bool.publish(bool_msg)



if __name__ == '__main__':
    #rospy.init_node('object_tracking')
    ot = ObjectTracker()
    #rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, ot.boundingbox_callback, queue_size=1)
    #rospy.Subscriber("/start_white", Bool, ot.main_callback, queue_size=1)
    rospy.spin()
    """
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        ot()
        r.sleep()
    # rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, ot.boundingbox_callback, queue_size=1)
    # rospy.Subscriber("/start_white", Bool, ot.main_callback, queue_size=1)
    """
