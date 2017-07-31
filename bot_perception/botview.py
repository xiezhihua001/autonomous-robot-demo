import cv2
import numpy as np
import matplotlib.image as mpimg
from picamera.array import PiRGBArray     #As there is a resolution problem in raspberry pi, will not be able to capture frames by VideoCapture
from picamera import PiCamera
from perception import color_thresh_road,color_thresh_obstacles,perception_step
import time
import threading
import lcm
import math
import sys
sys.path.append('./lcmtypes')
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t

import numpy as np

PI = np.pi

ODOMETRY_CHANNEL = "ODOMETRY_POSE"
ROBOT_PATH_T_CHANNEL = "CONTROLLER_PATH"


class botview():
	def __init__(self):
		# initialize LCM
		self.lc = lcm.LCM()
		self.robot_pose = pose_xyt_t()
		self.robot_path = robot_path_t()
		self.subscription = self.lc.subscribe(ODOMETRY_CHANNEL, self.robot_pose_handler)
		self.waypoints = []#[[0.18,0.5,PI],[0.5,0.5,PI],[0.5,0,PI],[0,0.5,PI],[0,0.19,PI]]

		self.x = 0
		self.y = 0
		self.theta = 0
		self.yaw = None

		self.bot_publish()
		img_length = 640#160
		img_width = 480#120
		# Image output from perception step
		# Update this image to display your intermediate analysis steps
		# on screen in autonomous mode
		self.vision_image = np.zeros((img_width, img_length, 3), dtype=np.float) 
		# Worldmap
		# Update this image with the positions of navigable terrain
		# obstacles and rock samples
		self.worldmap = np.zeros((200, 200, 3), dtype=np.float) 

		self.img = None
		self.camera = PiCamera()
		self.camera.resolution = (img_length, img_width)#(640, 480)
		self.camera.framerate = 32
		self.rawCapture = PiRGBArray(self.camera, size=(img_length, img_width))#(640, 480)
		time.sleep(0.1)
		# Define a box in source (original) and 
		# destination (desired) coordinates
		# Right now source and destination are just 
		# set to equal the four corners
		# of the image so no transform is taking place
		# Try experimenting with different values!
		source = np.float32([[130, 233], [266 ,266],[236, 193], [147, 199]])
		# Define calibration box in source (actual) and destination (desired) coordinates
		# These source and destination points are defined to warp the image
		# to a grid where each 10x10 pixel square represents 1 square meter
		dst_size = 5 
		# Set a bottom offset to account for the fact that the bottom of the image 
		# is not the position of the rover but a bit in front of it
		bottom_offset = 0.4/0.21*10#6
		self.camera.capture('image_template.jpg')
		image = mpimg.imread('./image_template.jpg')
		destination = np.float32([[image.shape[1]/2-dst_size, image.shape[0]-bottom_offset], 
					[image.shape[1]/2+dst_size, image.shape[0]-bottom_offset], 
					[image.shape[1]/2+dst_size, image.shape[0]-2*dst_size-bottom_offset], 
					[image.shape[1]/2-dst_size, image.shape[0]-2*dst_size-bottom_offset]])
		self.M = cv2.getPerspectiveTransform(source, destination)
		self.dst_size = dst_size

		self.function_sanity_flag = True

		self.robot_pose_thread = threading.Thread(target=self.robot_pose_thread_function)
		self.robot_pose_thread.start()
		# self.robot_perception_thread = threading.Thread(target=self.robot_perception_thread_function)
		# self.robot_perception_thread.start()

		self.robot_perception_thread_function()

		self.end_function()

	def bot_publish(self):
		self.robot_path.path_length = len(self.waypoints)
		angle_check = 1
		if angle_check == 1:
			for i in range(self.robot_path.path_length):
				pose_temp = pose_xyt_t()
				pose_temp.utime = int(time.time() * 1e6)
				pose_temp.x = self.waypoints[i][0]
				pose_temp.y = self.waypoints[i][1]
				pose_temp.theta = self.waypoints[i][2]
				
				#print "radians:",cmd.position_radians
				self.robot_path.path.append(pose_temp)

			self.robot_path.utime = int(time.time() * 1e6)
			self.lc.publish(ROBOT_PATH_T_CHANNEL,self.robot_path.encode())
		else:
			print "Check not passed.\n"



	# called whenever a message is recieved
	def robot_pose_handler(self,channel, data):
		pose_temp = self.robot_pose.decode(data)
		self.x = pose_temp.x
		self.y = pose_temp.y
		self.theta = pose_temp.theta
		print "timestamp: %d x: %+1.2f y: %+1.2f theta: %+1.2f" % \
		(pose_temp.utime, pose_temp.x, pose_temp.y, pose_temp.theta)

	def robot_pose_thread_function(self):
		while self.function_sanity_flag:
			self.lc.handle_timeout(1000) #blocks until message recieved or time out
		return

	def robot_perception_thread_function(self):
		print "robot_perception_thread_function() starts.\n"
		cv2.namedWindow("window",1)
	#	cv2.namedWindow("map",1)
		#cv2.setMouseCallback("window",mouse_callback)
		for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
			# grab the raw NumPy array representing the image, then initialize the timestamp
			# and occupied/unoccupied text
			image = frame.array
			self.img = image

			#self.botview = perception_step(self)
			cv2.imshow('window', self.img)
			#cv2.imshow('window', self.vision_image)
	#		cv2.imshow('map', self.worldmap)
			# show the frame
			#cv2.imshow("Frame", image)
			key = cv2.waitKey(1) & 0xFF

			# clear the stream in preparation for the next frame
			self.rawCapture.truncate(0)

			# if the `q` key was pressed, break from the loop
			if key == ord("q"):
				self.function_sanity_flag = False
				break
		cv2.destroyAllWindows()
		print "robot_perception_thread_function() end.\n"
		return


	def end_function(self):
		print "end function.\n"
		self.lc.unsubscribe(self.subscription)
		print "lcm unsubscribe.\n"



if __name__ == "__main__":
	botview = botview()