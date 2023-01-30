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

class crosswalk_detector:

  def __init__(self):
    self.img_folder_path = "/home/fizzer/cnn_trainer/imitation_learning_images/"
    self.crosswalk_pub = rospy.Publisher('/crosswalk', String, queue_size=1)
    self.safe2cross_pub = rospy.Publisher('/safe2cross', String, queue_size=1)
    self.license_plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.image_callback)
    self.crosswalk_detected = False
    self.safe2cross = False
    self.last_pixel_avg = -1

    time.sleep(1)

  def image_callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    blurred_color = cv2.GaussianBlur(cv_image,(5,5),cv2.BORDER_DEFAULT)

    hsv = cv2.cvtColor(blurred_color, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([95,120,50])
    upper_blue = np.array([110,160,100])

    lower_red = np.array([0,150,120])
    upper_red = np.array([10,255,255])


    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    height, width, channels = cv_image.shape

    first_red_height = 720
    height_set = False

    for i in range(500,height):
      if (255 in red_mask[i]) and (np.count_nonzero(red_mask[i] == 255)> 10) and not(height_set):
        first_red_height = i
        height_set = True

        break

    if (first_red_height != 720) and (first_red_height > 600):
      self.crosswalk_pub.publish("1")
      if not (self.crosswalk_detected):
        self.safe2cross = False
      self.crosswalk_detected = True
    else:
      self.crosswalk_pub.publish("0")
      self.crosswalk_detected = False

    pixel_list = []

    for i in range(height):
      if (255 in blue_mask[i]):
        result = np.where(blue_mask[i] == 255)
        pixel_list.extend(result[0].tolist())

    if (len(pixel_list)!=0):
      avg_loc = int(sum(pixel_list)/len(pixel_list))
    else:
      avg_loc = -1

    if (self.last_pixel_avg != -1) and (avg_loc != -1):
      if (avg_loc >= 640) and (self.last_pixel_avg <= 640):
        self.safe2cross = True
        self.safe2cross_pub.publish("1")
      elif (avg_loc <= 640) and (self.last_pixel_avg >= 640):
        self.safe2cross = True
        self.safe2cross_pub.publish("1")
      else:
        self.safe2cross_pub.publish("0")
    else:
      self.safe2cross_pub.publish("0")

    self.last_pixel_avg = avg_loc


    if (self.safe2cross):
      self.safe2cross = False


    # cv2.imshow("red mask", red_mask)
    # cv2.imshow("blue mask", blue_mask)
    # cv2.waitKey(1)

  def message(self, str):
    self.license_plate_pub.publish(str)


def main(args):
  rospy.init_node('crossswalk_detector', anonymous=True)
  cd = crosswalk_detector()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down now")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)