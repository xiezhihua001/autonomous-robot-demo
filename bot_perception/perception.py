import numpy as np
import cv2
import matplotlib.image as mpimg

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh_road(img, rgb_thresh=(160, 160, 160)):
	# Create an array of zeros same xy size as img, but single channel

	# Index the array of zeros with the boolean array and set to 1


	lower_road = np.array([0,0,200])
	upper_road = np.array([100,100,255])
	color_db = [[lower_road, upper_road]]
	hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv,color_db[0][0], color_db[0][1])
	closing_kenel_size = 5
	closing_kernel = np.ones((closing_kenel_size,closing_kenel_size),np.uint8)
	color_select = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,closing_kernel)

	# Return the binary image
	return color_select

def color_thresh_obstacles(img, rgb_thresh=(160, 160, 160)):
	lower_road = np.array([0,80,0])
	upper_road = np.array([50,255,100])
	color_db = [[lower_road, upper_road]]
	hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv,color_db[0][0], color_db[0][1])
	closing_kenel_size = 6
	closing_kernel = np.ones((closing_kenel_size,closing_kenel_size),np.uint8)
	color_select = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,closing_kernel)

	# Return the binary image
	return color_select

def color_thresh_parking(img, rgb_thresh=(110, 110, 50)):
	# Create an array of zeros same xy size as img, but single channel
	#Closing: dilation followed by Erosion
	closing_kenel_size = 1
	closing_kernel = np.ones((closing_kenel_size,closing_kenel_size),np.uint8)
	    
	lower_yellow = np.array([15,235,135])
	upper_yellow = np.array([35,255,155])
	color_db = [[lower_yellow, upper_yellow]]
	hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	mask = cv2.inRange(hsv,color_db[0][0], color_db[0][1])
	closing = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,closing_kernel)
	#    res = cv2.bitwise_and(img,img, mask= closing)
	return closing

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
	# Identify nonzero pixels
	ypos, xpos = binary_img.nonzero()
	# Calculate pixel positions with reference to the rover position being at the 
	# center bottom of the image.  
	x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
	y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
	return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
	# Convert (x_pixel, y_pixel) to (distance, angle) 
	# in polar coordinates in rover space
	# Calculate distance to each pixel
	dist = np.sqrt(x_pixel**2 + y_pixel**2)
	# Calculate angle away from vertical for each pixel
	angles = np.arctan2(y_pixel, x_pixel)
	return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
	# Convert yaw to radians
	yaw_rad = yaw * np.pi / 180
	xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
	                        
	ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
	# Return the result  
	return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
	# Apply a scaling and a translation
	xpix_translated = (xpix_rot / scale) + xpos
	ypix_translated = (ypix_rot / scale) + ypos
	# Return the result  
	return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
	# Apply rotation
	xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
	# Apply translation
	xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
	# Perform rotation, translation and clipping all at once
	x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
	y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
	# Return the result
	return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, M):
	#M = cv2.getPerspectiveTransform(src, dst)
	warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
	mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))# keep same size as input image
	return warped, mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(botview):
	# Perform perception steps to update Rover()
	# TODO: 
	# NOTE: camera image is coming to you in Rover.img
	# 1) Define source and destination points for perspective transform

	#initialized in driver_rover.py

	view_road = color_thresh_road(botview.img)
	view_obstacles = color_thresh_obstacles(botview.img)
	view_parking = color_thresh_parking(botview.img)
	botview.img_view_threshold[:,:,1] = view_road*255
	botview.img_view_threshold[:,:,2] = view_obstacles*255
	botview.img_view_threshold[:,:,0] = view_parking*255
	# 2) Apply perspective transform
	warped,_ = perspect_transform(botview.img, botview.M)
#	botview.img_view_threshold = warped
	# 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
	road = color_thresh_road(warped)
	obstacles = color_thresh_obstacles(warped)
	parking = color_thresh_parking(warped)
	# 4) Update botview.vision_image (this will be displayed on left side of screen)
	botview.vision_image[:,:,1] = road*255 #green
	botview.vision_image[:,:,0] = parking*255 #blue
	botview.vision_image[:,:,2] = obstacles*255 #red


	# 5) Convert map image pixel values to rover-centric coords
	xpix_road, ypix_road = rover_coords(road)
	xpix_obstacles, ypix_obstacles = rover_coords(obstacles)
	xpix_parking, ypix_parking = rover_coords(parking)


	# 6) Convert rover-centric pixel values to world coordinates
	world_size = botview.worldmap.shape[0]
	xpos = botview.x
	ypos = botview.y
	yaw = botview.theta#botview.yaw
	road_x_world, road_y_world = pix_to_world(xpix_road, ypix_road, xpos, ypos, yaw, world_size, botview.scale)
	obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obstacles, ypix_obstacles, xpos, ypos, yaw, world_size, botview.scale)
	parking_x_world, parking_y_world = pix_to_world(xpix_parking, ypix_parking, xpos, ypos, yaw, world_size, botview.scale)


	# 7) Update Rover worldmap (to be displayed on right side of screen)
		# Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
		#          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
		#          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
	# if rocks.any():
	#     Rover.vision_image[:,:,1] = rocks*255
	#     xpix_rocks, ypix_rocks = rover_coords(rocks)
	#     rock_x_world, rock_y_world = pix_to_world(xpix_rocks, ypix_rocks, xpos, ypos, yaw, world_size, scale)
	#     Rover.worldmap[rock_y_world, rock_x_world, 1] = 255 
	botview.worldmap[road_y_world, road_x_world, 2] =255#+= 10
	botview.worldmap[obstacle_y_world, obstacle_x_world, 0] =255#+= 1 
	nav_map = botview.worldmap[:,:,2] > 0
	botview.worldmap[nav_map,0] = 0


	# 8) Convert rover-centric pixel positions to polar coordinates
	# Update Rover pixel distances and angles
	# Calculate pixel values in rover-centric coords and distance/angle to all pixels
	
#	dist1, angles1 = to_polar_coords(xpix_road, ypix_road)


	index_pix_xlimit = xpix_road<(0.2*botview.scale) #2 m y #################
	dist, angles = to_polar_coords(xpix_road[index_pix_xlimit], ypix_road[index_pix_xlimit])
#	print "Length of angles:",len(angles1),len(angles),0.3*botview.scale,max(ypix_road),max(xpix_road),".\n"
	if len(angles) == 0:
		botview.nav_dists = 0
		botview.nav_angles = 0
	else:
		mean_dir = np.mean(angles)
		mean_dist = np.mean(dist)
		botview.nav_dists = 0.2#mean_dist
		botview.nav_angles = mean_dir
		#print mean_dir
		#print "max and min:",np.max(dist),np.min(dist),mean_dist
	index_pix = (xpix_road<10) & (xpix_road>-10)
	dist_select, angles_select = to_polar_coords(xpix_road[index_pix], ypix_road[index_pix])
	botview.stop_front_thresh = len(dist_select)
	# if botview.stop_front_thresh < 5:
	# 	botview.nav_dists = 0
	# 	botview.nav_angles = 0		
	return botview