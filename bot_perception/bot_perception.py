import cv2
import numpy as np
import matplotlib.image as mpimg
from picamera.array import PiRGBArray     #As there is a resolution problem in raspberry pi, will not be able to capture frames by VideoCapture
from picamera import PiCamera
import time

import lcm
import math
import sys


class botview():
	def __init__(self):
		    # initialize LCM
		self.lc = lcm.LCM()
		# Image output from perception step
		# Update this image to display your intermediate analysis steps
		# on screen in autonomous mode
		self.vision_image = np.zeros((160, 320, 3), dtype=np.float) 
		# Worldmap
		# Update this image with the positions of navigable terrain
		# obstacles and rock samples
		self.worldmap = np.zeros((200, 200, 3), dtype=np.float) 
		camera = PiCamera()
		camera.resolution = (640, 480)
		camera.framerate = 32
		rawCapture = PiRGBArray(camera, size=(640, 480))
		time.sleep(0.1)
		cv2.namedWindow("window",1)
		#cv2.setMouseCallback("window",mouse_callback)

		for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
			# grab the raw NumPy array representing the image, then initialize the timestamp
			# and occupied/unoccupied text
			image = frame.arrayimport lcm

			hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    

			# 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
			threshed = color_thresh(warped)
			obstacles = color_thresh_obstacles(warped)
			rocks = color_thresh_rocks(warped)
			# 4) Update Rover.vision_image (this will be displayed on left side of screen)
			Rover.vision_image[:,:,0] = obstacles*255
			Rover.vision_image[:,:,2] = threshed*255

			cv2.imshow('window', image)
			# show the frame
			#cv2.imshow("Frame", image)
			key = cv2.waitKey(1) & 0xFF

			# clear the stream in preparation for the next frame
			rawCapture.truncate(0)

			# if the `q` key was pressed, break from the loop
			if key == ord("q"):
				break
		cv2.destroyAllWindows()


if __name__ == "__main__":
	botview()