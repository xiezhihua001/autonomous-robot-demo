import cv2
import numpy as np
import matplotlib.image as mpimg
from picamera.array import PiRGBArray     #As there is a resolution problem in raspberry pi, will not be able to capture frames by VideoCapture
from picamera import PiCamera
import time
#CAMERA CAPTURE
#initialize the camera and grab a reference to the raw camera capture

# camera.resolution = (160, 120)
# camera.framerate = 16
# rawCapture = PiRGBArray(camera, size=(160, 120))



def mouse_callback(event,x,y,flags,param):
  r = image[y][x][2]
  g = image[y][x][1]
  b = image[y][x][0]
  h = hsv[y][x][0]
  s = hsv[y][x][1]
  v = hsv[y][x][2]
  output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
  output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
  tmp = image.copy()
  cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
  cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
  cv2.imshow('window', tmp)
  if event == cv2.EVENT_LBUTTONDOWN:
      print ("hsv: (%d, %d, %d)" % (h,s,v),"rgb: (%d, %d, %d)" % (r,g,b))

def get_img():
    img = '../calibration_images/example_rock1.jpg'
    array = mpimg.imread(img)
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
if __name__ == "__main__":

    font = cv2.FONT_HERSHEY_SIMPLEX
    camera = PiCamera()
    camera.resolution = (320, 240)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(320, 240))
    time.sleep(0.1)
    img = camera.capture("test.jpg")
    #rock_img = get_img()


