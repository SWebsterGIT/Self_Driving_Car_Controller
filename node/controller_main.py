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
from matplotlib import pyplot as plt
from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers
from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend

class controller_main:

  def __init__(self):
    self.twist_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
    self.license_plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.main_callback)
    self.crosswalk_sub = rospy.Subscriber('/crosswalk', String, self.crosswalk_callback)
    self.safe2cross_sub = rospy.Subscriber('/safe2cross', String, self.safe2cross_callback)
    self.PID_sub = rospy.Subscriber('/PID', Twist, self.PID_callback)
    self.cnn_model = models.load_model("/home/fizzer/cnn_trainer/imitation_learning_cnn/latest_model.h5")
    self.crosswalk_detected = False
    self.safe_to_cross = False
    self.PID_control = False
    self.state = "Drive"
    self.string_dict ={
        0:"F",
        1:"L",
        2:"R"
    }

    time.sleep(1)

  def main_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    move = Twist()

    if (self.state == "Drive" and not(self.crosswalk_detected)):
        blurred_color = cv2.GaussianBlur(cv_image,(5,5),cv2.BORDER_DEFAULT)
        hsv = cv2.cvtColor(blurred_color, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0,0,120])
        upper_white = np.array([179,15,255])

        height, width, channels = cv_image.shape
        mask = cv2.inRange(hsv, lower_white, upper_white)

        scale_factor = 12
        width = int(width / scale_factor)
        height = int(height / scale_factor)
        dim = (width, height)
        mask = cv2.resize(mask, dim, interpolation = cv2.INTER_AREA)
        cropped_mask = mask[int(height/2):height, 0:width]
        cropped_mask = cropped_mask / 255

        cropped_mask = np.expand_dims(np.expand_dims(cropped_mask ,axis=-1), axis=0)
        cnn_output = self.cnn_model.predict(cropped_mask)[0]

        max_index = np.argmax(cnn_output)
        direction = self.string_dict.get(max_index)

        #cv2.imshow("Raw", cv_image) 
        #cv2.waitKey(1)

        #print(direction)

        if (direction == "F"):
            move.angular.z = 0
            move.linear.x = .35
        elif (direction == "L"):
            move.angular.z = .75
            move.linear.x = 0
        else:
            move.angular.z = -.75
            move.linear.x = 0
        
        self.twist_pub.publish(move)

    elif (self.crosswalk_detected):
      move.linear.x = 0
      move.angular.z = 0
      self.twist_pub.publish(move)

      # move.angular.z = -1
      # self.twist_pub.publish(move)
      # rospy.sleep(.2)

      # move.angular.z = 0
      # self.twist_pub.publish(move)
      
      while (not (self.safe_to_cross)):
        pass
      
      self.PID_control = True
      rospy.sleep(4)
      self.PID_control = False

      self.crosswalk_detected = False

  def crosswalk_callback(self, data):
    if (str(data) == "data: \"0\"") and not(self.crosswalk_detected):
      self.crosswalk_detected = False
    elif (not(self.crosswalk_detected)):
      #print("Reached")
      self.crosswalk_detected = True

  def safe2cross_callback(self, data):
    if (str(data) == "data: \"0\""):
      self.safe_to_cross = False
    else:
      self.safe_to_cross = True

  def PID_callback(self, data):
    if (self.PID_control):
      move = Twist()
      move.linear.x = data.linear.x
      move.angular.z = data.angular.z
      #print(move.linear.x)
      #print(move.angular.z)
      self.twist_pub.publish(move)


  def message(self, str):
    self.license_plate_pub.publish(str)


def main(args):
  cm = controller_main()
  rospy.init_node('controller_main', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)