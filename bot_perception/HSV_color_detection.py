import cv2
import numpy as np
import matplotlib.image as mpimg

def mouse_callback(event,x,y,flags,param):
  r = rock_img[y][x][2]
  g = rock_img[y][x][1]
  b = rock_img[y][x][0]
  h = hsv[y][x][0]
  s = hsv[y][x][1]
  v = hsv[y][x][2]
  output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
  output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
  tmp = rock_img.copy()
  cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
  cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
  cv2.imshow('window', tmp)
  if event == cv2.EVENT_LBUTTONDOWN:
      print ("hsv: (%d, %d, %d)" % (h,s,v),"rgb: (%d, %d, %d)" % (r,g,b))

def get_img():
    example_rock = 'color_test.jpg'
    array = mpimg.imread(example_rock)
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
if __name__ == "__main__":

    font = cv2.FONT_HERSHEY_SIMPLEX
    example_rock = 'color_test.jpg'
    rock_img = get_img()

    cv2.namedWindow("window",1)
    cv2.setMouseCallback("window",mouse_callback)
    
    while 1:
        #get a frame from RGB camera
        rock_img = get_img()
        hsv = cv2.cvtColor(rock_img, cv2.COLOR_BGR2HSV)       
        cv2.imshow('window', rock_img)

        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()


