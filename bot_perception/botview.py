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
import matplotlib.pyplot as plt
sys.path.append('./lcmtypes')
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t

import numpy as np

PI = np.pi

ODOMETRY_CHANNEL = "ODOMETRY_POSE"
ROBOT_PATH_T_CHANNEL = "CONTROLLER_PATH"

img_length = 160
img_width = 128
dist_camera_paper = 0.20# unit: meter
length_paper = 0.218# unit: meter
dst_size = 5 

# Set a bottom offset to account for the fact that the bottom of the image 
# is not the position of the rover but a bit in front of it
bottom_offset = dist_camera_paper/length_paper*10#6

class botview():
	def __init__(self):
		# initialize LCM
		self.lc = lcm.LCM()
		self.robot_pose = pose_xyt_t()
		self.robot_path = robot_path_t()
		self.subscription = self.lc.subscribe(ODOMETRY_CHANNEL, self.robot_pose_handler)
		self.waypoints = []#3*[[0.0,1,PI],[1,0.0,PI],[1,1,PI],[1,0,PI],[0,1,PI],[0,0,PI]]#[[0.0,0.5,PI],[0.5,0.0,PI],[0.5,0.5,PI],[0.5,0,PI],[0,0.5,PI],[0,0,PI]]

		self.stop_front_thresh = None
		self.nav_dists = None
		self.nav_angles = None

		self.x = 0
		self.y = 0
		self.theta = 0
		self.yaw = None
		self.scale = 10/length_paper
		# self.bot_publish()


		self.vision_image = np.zeros((img_width, img_length, 3), dtype=np.float) 
		self.img_view_threshold = np.zeros((img_width, img_length, 3), dtype=np.float)

		self.worldmap = np.zeros((100, 100, 3), dtype=np.float) 

		# left for picamera
		self.img = None
		self.camera = None
		self.rawCapture = None
		self.M = None
		self.dst_size = None

		self.function_sanity_flag = True

		#self.robot_pose_thread_function()

		self.robot_pose_thread = threading.Thread(target=self.robot_pose_thread_function)
		self.robot_pose_thread.start()

		self.robot_perception_thread_function()

		self.end_function()

	def bot_dir2path(self):
		# if math.isnan(self.nav_angles):
		# 	angle_temp = self.theta
		# 	x_temp = self.x
		# 	y_temp = self.y
		# else:
		angle_temp = self.nav_angles + self.theta
		x_temp = self.x + self.nav_dists*np.cos(angle_temp)
		y_temp = self.y + self.nav_dists*np.sin(angle_temp)
		self.waypoints = []
		self.waypoints.append([x_temp,y_temp,angle_temp])
#		print self.waypoints,"\n"
		#print [x_temp,y_temp,angle_temp],self.stop_front_thresh

	def bot_publish(self):
		self.robot_path.path_length = len(self.waypoints)
		angle_check = 1
		self.robot_path.path = []
		if angle_check == 1:
			for i in range(self.robot_path.path_length):
				pose_temp = pose_xyt_t()
				pose_temp.utime = int(time.time() * 1e6)
				pose_temp.x = self.waypoints[i][0]
				pose_temp.y = self.waypoints[i][1]
				pose_temp.theta = self.waypoints[i][2]
				#print "radians:",cmd.position_radians
#				print "pose x,y,stopfrontthresh:",pose_temp.x,pose_temp.y,self.stop_front_thresh,"\n"
				self.robot_path.path.append(pose_temp)
#			print "Path x",self.robot_path.path[0].x,"\n"
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
		# print "timestamp: %d x: %+1.2f y: %+1.2f theta: %+1.2f" % \
		# (pose_temp.utime, pose_temp.x, pose_temp.y, pose_temp.theta)

	def robot_pose_thread_function(self):
		while self.function_sanity_flag:
			self.lc.handle_timeout(1000) #blocks until message recieved or time out
		return

	def robot_perception_thread_function(self):
		print "robot_perception_thread_function() starts.\n"
		self.camera = PiCamera()
		self.camera.resolution = (img_length, img_width)#(640, 480)
		self.camera.framerate = 32
		self.rawCapture = PiRGBArray(self.camera, size=(img_length, img_width))#(640, 480)
		time.sleep(0.1)
		source = np.float32([[5, 117], [156 ,119],[117, 86], [44, 84]])
		self.camera.capture('calibration_template.jpg')
		image = mpimg.imread('./calibration_template.jpg')
		destination = np.float32([[image.shape[1]/2-dst_size, image.shape[0]-bottom_offset], 
					[image.shape[1]/2+dst_size, image.shape[0]-bottom_offset], 
					[image.shape[1]/2+dst_size, image.shape[0]-2*dst_size-bottom_offset], 
					[image.shape[1]/2-dst_size, image.shape[0]-2*dst_size-bottom_offset]])
		self.M = cv2.getPerspectiveTransform(source, destination)
		self.dst_size = dst_size

		cv2.namedWindow("window",1)
		#cv2.namedWindow("map",1)
		cv2.namedWindow("view img",1)
		cv2.namedWindow("view threshold",1)

		for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
			# grab the raw NumPy array representing the image, then initialize the timestamp
			# and occupied/unoccupied text
			image = frame.array
			self.img = image

			self.botview = perception_step(self)
			#cv2.imshow('window', self.img)

			output_loc = "x:%d, y:%d, theta:%d " % (self.x, self.y, self.theta)
			cv2.putText(self.worldmap,output_loc, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
			cv2.imshow('view img', self.img)
			cv2.imshow('window', self.vision_image)
			#cv2.imshow('map', self.worldmap)
			cv2.imshow('view threshold', self.img_view_threshold)
			

			# show the frame
			#cv2.imshow("Frame", image)

			# fig = plt.figure(figsize=(12,9))
			# plt.subplot(221)
			# plt.imshow(image)
			# plt.subplot(222)
			# plt.imshow(self.botview.vision_image)
			# plt.subplot(223)
			# plt.imshow(self.botview.worldmap)
			# plt.subplot(224)
			# plt.plot(self.botview.x, self.botview.y, '.')

			key = cv2.waitKey(1) & 0xFF
			# clear the stream in preparation for the next frame
			self.rawCapture.truncate(0)


			# update path information and publish 
			self.bot_dir2path()
			self.bot_publish()

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