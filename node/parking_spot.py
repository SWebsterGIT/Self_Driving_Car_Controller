#! /usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time
import os
import numpy as np

class parking_spot:

  def __init__(self):

    self.current_spot = 2

    self.license_plate_order = [2,3,4,5,6,1]

    self.CONTOUR_AREA_THRESHOLD = 5000

    self.license_plate_to_filter_mode = {
        2:"D",
        3:"B",
        4:"V",
        5:"V",
        6:"D",
        1:"D"
    }

    self.license_plate_to_reset_time = {
      2: 20,
      3: 60,
      4: 40,
      5: 30,
      6: 50,
      1: 40
    }

    self.reset_threshold = self.license_plate_to_reset_time.get(self.license_plate_order[0])

    self.current_index = 0

    self.detected = False
    self.iterations_since_last_plate = 0

    self.low_dark_thresh = (0, 0, 97)
    self.high_dark_thresh = (1, 1, 106)

    self.low_bright_thresh = (0, 0, 116)
    self.high_bright_thresh = (1, 1, 127)

    self.low_vbright_thresh = (0, 0, 197)
    self.high_vbright_thresh = (1, 1, 204)


    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.image_callback)
    self.spot_pub = rospy.Publisher('/parking_spot', String, queue_size=1)

    time.sleep(1)

    self.current_image = None

  def HSV_Processing_func(self, low_grey_thresh, high_grey_thresh):

    hsv_car = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)

    car_grey_strip_mask = cv2.inRange(hsv_car, low_grey_thresh,high_grey_thresh)

    contours, hierarchy = cv2.findContours(car_grey_strip_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    biggest_contour = 0
    approx = 0

    if len(contours) != 0:
      biggest_contour = max(contours, key = cv2.contourArea)
      
      perimeter = cv2.arcLength(biggest_contour, True)
      approx = cv2.approxPolyDP(biggest_contour, 0.05 * perimeter, True)

      return len(approx), cv2.contourArea(biggest_contour)

    else:
      return 0,0  

  def image_callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
      
    self.current_image = cv_image

    current_brightness = self.license_plate_to_filter_mode.get(self.license_plate_order[self.current_index])

    if(current_brightness == "D"):
        
      dark_corners, dark_area = self.HSV_Processing_func( self.low_dark_thresh, self.high_dark_thresh)

      if (dark_corners == 4) and (dark_area > self.CONTOUR_AREA_THRESHOLD):
            if not(self.detected):
                #print("P" + str(self.license_plate_order[self.current_index]))
                self.detected = True
            self.iterations_since_last_plate = 0
      else:
            self.iterations_since_last_plate +=1

    elif (current_brightness == "B"):
      
      bright_corners, bright_area = self.HSV_Processing_func( self.low_bright_thresh, self.high_bright_thresh)

      if (bright_corners == 4) and (bright_area > self.CONTOUR_AREA_THRESHOLD):
            if not(self.detected):
                #print("P" + str(self.license_plate_order[self.current_index]))
                self.detected = True
            self.iterations_since_last_plate = 0
      else:
            self.iterations_since_last_plate +=1
    else:
      
      vbright_corners, vbright_area = self.HSV_Processing_func( self.low_vbright_thresh, self.high_vbright_thresh)

      if (vbright_corners == 4) and (vbright_area > self.CONTOUR_AREA_THRESHOLD):
            if not(self.detected):
                #print("P" + str(self.license_plate_order[self.current_index]))
                self.detected = True
            self.iterations_since_last_plate = 0
      else:
            self.iterations_since_last_plate +=1
    
    if(self.iterations_since_last_plate > self.reset_threshold) and (self.detected):
        self.detected = False
        self.current_index += 1
        if (self.current_index == 6):
            self.current_index = 0
        self.reset_threshold = self.license_plate_to_reset_time.get(self.license_plate_order[self.current_index])

        self.current_spot = self.license_plate_order[self.current_index]

    # if (self.iterations_since_last_plate % 10 == 0):
    #   print("Loop Iterations Since Last Detected Plate :" + str(self.iterations_since_last_plate))

    self.spot_pub.publish(str(self.current_spot))



def main(args):
  rospy.init_node('parking_spot', anonymous=True)
  ps = parking_spot()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down now")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

