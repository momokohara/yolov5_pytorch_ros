#!/home/raghavauppuluri/robot-arm/bin/python

# util + model imports
from utils.torch_utils import select_device, load_classifier, time_synchronized
from utils.plots import plot_one_box
from utils.general import check_img_size, check_requirements, non_max_suppression, apply_classifier, scale_coords, \
	xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.datasets import LoadStreams, LoadImages
from models.experimental import attempt_load

import argparse
import sys
import os
import time
from pathlib import Path

# Python imports
import numpy as np
import scipy.io as sio
import os
import sys
import cv2
import time
from skimage.transform import resize

# ROS imports
import rospy
import std_msgs.msg
from rospkg import RosPack
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError

# Deep Learning imports
import cv2
import torch
import torch.backends.cudnn as cudnn
from torch.autograd import Variable
from numpy import random
import yaml

package = RosPack()
package_path = package.get_path('yolov5_pytorch_ros')

class Detector:
	def __init__(self):
		# Load weights parameter
		weights_name = rospy.get_param('~weights_name')
		self.weights_path = os.path.join(package_path, 'weights', weights_name)
		rospy.loginfo("Found weights, loading %s", self.weights_path)

		# Raise error if it cannot find the model
		if not os.path.isfile(self.weights_path):
			raise IOError(('{:s} not found.').format(self.weights_path))

		# Load image parameter and confidence threshold
		self.image_topic = rospy.get_param(
			'~image_topic', '/camera/rgb/image_raw')

		log_str = "Subscribing to image topic: %s" % self.image_topic 
		rospy.loginfo(log_str)

		self.conf_thres = rospy.get_param('~confidence', 0.5)

		# Load other parameters
		self.device_name = 'cpu'
		self.device = select_device(self.device_name)
		self.gpu_id = rospy.get_param('~gpu_id', 0)
		self.network_img_size = rospy.get_param('~img_size', 416)
		self.publish_image = rospy.get_param('~publish_image')
		self.iou_thres = 0.45
		self.augment = True

		self.classes = None
		self.agnostic_nms = False

		self.w = 0
		self.h = 0

		# Second-stage classifier
		self.classify = False

		# Initialize
		self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

		# Load model
		self.model = attempt_load(
			self.weights_path, map_location=self.device)  # load FP32 model
		self.stride = int(self.model.stride.max())	# model stride
		self.network_img_size = check_img_size(
			self.network_img_size, s=self.stride)  # check img_size

		if self.half:
			self.model.half()  # to FP16

		if self.classify:
			self.modelc = load_classifier(name='resnet101', n=2)  # initialize
			self.modelc.load_state_dict(torch.load(
				'weights/resnet101.pt', map_location=self.device)['model']).to(self.device).eval()

		# Get names and colors
		self.names = self.model.module.names if hasattr(
			self.model, 'module') else self.model.names
		self.colors = [[random.randint(0, 255)
						for _ in range(3)] for _ in self.names]

		# Run inference
		if self.device.type != 'cpu':
			self.model(torch.zeros(1, 3, self.network_img_size, self.network_img_size).to(
				self.device).type_as(next(self.model.parameters())))  # run once

		# Load CvBridge
		self.bridge = CvBridge()
		
		# Load publisher topic
		self.detected_objects_topic = rospy.get_param(
			'~detected_objects_topic')
		self.published_image_topic = rospy.get_param('~detections_image_topic')


		# Define subscribers
		self.image_sub = rospy.Subscriber(
			self.image_topic, Image, self.imageCb, queue_size=1, buff_size=2**24)

		# Define publishers
		self.pub_ = rospy.Publisher(
			self.detected_objects_topic, BoundingBoxes, queue_size=10)
		self.pub_viz_ = rospy.Publisher(
			self.published_image_topic, Image, queue_size=10)
		rospy.loginfo("Launched node for object detection")

		# Spin
		rospy.spin()

	def imageCb(self, data):
		# Convert the image to OpenCV
		try:
			self.cv_img = self.bridge.imgmsg_to_cv2(data, "rgb8")
		except CvBridgeError as e:
			print(e)
		 # Initialize detection results

		detection_results = BoundingBoxes()
		detection_results.header = data.header
		detection_results.image_header = data.header
		input_img = self.imagePreProcessing(self.cv_img)
		input_img = Variable(input_img.type(torch.FloatTensor))

		# Get detections from network
		with torch.no_grad():
			detections = self.model(input_img)[0]
			detections = non_max_suppression(detections, self.conf_thres, self.iou_thres,
												classes=self.classes, agnostic=self.agnostic_nms)
		# Parse detections
		if detections[0] is not None:
			for detection in detections[0]:
				# Get xmin, ymin, xmax, ymax, confidence and class
				xmin, ymin, xmax, ymax, conf, det_class = detection
				pad_x = max(self.h - self.w, 0) * (self.network_img_size/max(self.h, self.w))
				pad_y = max(self.w - self.h, 0) * (self.network_img_size/max(self.h, self.w))
				unpad_h = self.network_img_size-pad_y
				unpad_w = self.network_img_size-pad_x
				xmin_unpad = ((xmin-pad_x//2)/unpad_w)*self.w
				xmax_unpad = ((xmax-xmin)/unpad_w)*self.w + xmin_unpad
				ymin_unpad = ((ymin-pad_y//2)/unpad_h)*self.h
				ymax_unpad = ((ymax-ymin)/unpad_h)*self.h + ymin_unpad

				# Populate darknet message
				detection_msg = BoundingBox()
				detection_msg.xmin = int(xmin_unpad)
				detection_msg.xmax = int(xmax_unpad)
				detection_msg.ymin = int(ymin_unpad)
				detection_msg.ymax = int(ymax_unpad)
				detection_msg.probability = float(conf)
				detection_msg.Class = self.names[int(det_class)]

				# Append in overall detection message
				detection_results.bounding_boxes.append(detection_msg)
		
		# Publish detection results
		self.pub_.publish(detection_results)

		# Visualize detection results
		if (self.publish_image):
			self.visualizeAndPublish(detection_results, self.cv_img)
		return True

	def imagePreProcessing(self, img):
		# Extract image and shape
		img = np.copy(img)
		img = img.astype(float)
		height, width, channels = img.shape
		if (height != self.h) or (width != self.w):
			self.h = height
			self.w = width
			# Determine image to be used
			self.padded_image = np.zeros(
				(max(self.h, self.w), max(self.h, self.w), channels)).astype(float)

		# Add padding
		if (self.w > self.h):
			self.padded_image[(self.w-self.h)//2: self.h +
								(self.w-self.h)//2, :, :] = img
		else:
			self.padded_image[:, (self.h-self.w)//2: self.w +
								(self.h-self.w)//2, :] = img
		# Resize and normalize
		input_img = resize(self.padded_image,
							(self.network_img_size, self.network_img_size, 3))/255.

		# Channels-first
		input_img = np.transpose(input_img, (2, 0, 1))

		# As pytorch tensor
		input_img = torch.from_numpy(input_img).float()
		input_img = input_img[None]

		return input_img

	def visualizeAndPublish(self, output, imgIn):
		# Copy image and visualize
		imgOut = imgIn.copy()
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontScale = 0.8
		thickness = 2
		for index in range(len(output.bounding_boxes)):
			label = output.bounding_boxes[index].Class
			x_p1 = output.bounding_boxes[index].xmin
			y_p1 = output.bounding_boxes[index].ymin
			x_p3 = output.bounding_boxes[index].xmax
			y_p3 = output.bounding_boxes[index].ymax
			confidence = output.bounding_boxes[index].probability

			# Set class color
			color = self.colors[self.names.index(label)]
			
			# Create rectangle
			cv2.rectangle(imgOut, (int(x_p1), int(y_p1)), (int(x_p3), int(y_p3)), (color[0],color[1],color[2]),thickness)
			text = ('{:s}: {:.3f}').format(label,confidence)
			cv2.putText(imgOut, text, (int(x_p1), int(y_p1+20)), font, fontScale, (255,255,255), thickness ,cv2.LINE_AA)

		# Publish visualization image
		image_msg = self.bridge.cv2_to_imgmsg(imgOut, "rgb8")
		image_msg.header.frame_id = "camera"
		image_msg.header.stamp = rospy.Time.now()
		self.pub_viz_.publish(image_msg)

if __name__ == '__main__':
	rospy.init_node("detector_manager_node")

	# Define detector object
	dm = Detector()
