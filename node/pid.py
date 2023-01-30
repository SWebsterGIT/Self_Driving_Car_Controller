#! /usr/bin/env python3
from __future__ import print_function

import roslib
#roslib.load_manifest('my_controller')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time
import numpy as np

class PID:

  def __init__(self):
    #self.twist_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
    self.twist_pub = rospy.Publisher('/PID', Twist, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.callback)
    self.last_move = 'F'
    move = Twist() 
    self.crosswalk_detected = False
    move.angular.z = 0
    move.linear.x = 0
    self.twist_pub.publish(move)
    time.sleep(1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    line_height = 0

    blurred_color = cv2.GaussianBlur(cv_image,(5,5),cv2.BORDER_DEFAULT)

    hsv = cv2.cvtColor(blurred_color, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0,0,100])
    upper_white = np.array([179,15,255])

    lower_blue = np.array([120,128,101])
    upper_blue = np.array([121,255,201])

    lower_red = np.array([0,150,120])
    upper_red = np.array([10,255,255])

    height, width, channels = cv_image.shape

    mask = cv2.inRange(hsv, lower_white, upper_white)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)

    ROI = mask[int(7*height/8):height, 0:width]
    blue_ROI = blue_mask[int(7*height/8):height, 0:width]
    red_mask = red_mask[ int(2*height/4):int(3*height/4), 0:width]

    height, width = ROI.shape

    min_l = 0
    max_l = 0
    min_r =0
    max_r =0

    result = np.where( ROI[line_height] == 255)[0]
    blue_ids = np.where( blue_ROI[line_height] == 255)[0].tolist()

    height, width = red_mask.shape
    red_pixel_list = []

    for i in range(height):
      if (255 in red_mask[i]):
        result = np.where(red_mask[i] == 255)
        red_pixel_list.extend(result[0].tolist())

    if (len(red_pixel_list) > 20):
        avg_red_loc = int(sum(red_pixel_list)/len(red_pixel_list))
        self.crosswalk_detected = True
    else:
        self.crosswalk_detected = False


    if(len(blue_ids)!=0):
        blue_center = int(sum(blue_ids)/len(blue_ids))
    else:
        blue_center = -1


    sequence = 0

    for i in range(len(result)):
        if (i!=0):
            if result[i] == (result[i-1] +1):
                sequence += 1
            if result[i] != (result[i-1] + 1):
                if (sequence > 6):
                    sequence_start = result[i-1] - sequence
                    sequence_end = result[i-1]
                    
                    if(min_l == 0):
                        min_l = sequence_start
                        max_l = sequence_end
                        
                    if(sequence_start > min_r):
                        min_r = sequence_start
                        max_r = sequence_end

                sequence = 0
            if (i == len(result) - 1):
                if (sequence > 10):
                    sequence_start = result[i] - sequence
                    sequence_end = result[i]

                    if(min_l == 0):
                        min_l = sequence_start
                        max_l = sequence_end

                    min_r = sequence_start
                    max_r = sequence_end

    # print("min l "+ str(min_l))
    # print("max l "+ str(max_l))
    # print("min r "+ str(min_r))
    # print("max r "+ str(max_l))

    if not(self.crosswalk_detected):
        if(min_l != min_r):
            self.last_move = "F"
            move = Twist() 
            move.angular.z = 0
            move.linear.x = .5
            self.twist_pub.publish(move)
        elif( int(min_l+max_l)/2 > 640):
            if ((blue_center != -1) and (blue_center < 640)):
                self.last_move = "F"
                move = Twist() 
                move.angular.z = -1.5
                move.linear.x = .35
                self.twist_pub.publish(move)
            else:
                if (self.last_move == "R"):
                    move = Twist() 
                    move.angular.z = 0
                    move.linear.x = .5
                    self.twist_pub.publish(move)
                else:
                    int(min_l+min_r)/2 > 640
                    move = Twist() 
                    move.angular.z = 1
                    move.linear.x =0
                    self.twist_pub.publish(move)
                self.last_move = "L"
        else:
            if (blue_center != -1) and (blue_center > 640):
                self.last_move = "F"
                move = Twist() 
                move.angular.z = 1.5
                move.linear.x = .35
                self.twist_pub.publish(move)
            else:    
                if (self.last_move == "L"):
                    move = Twist() 
                    move.angular.z = 0
                    move.linear.x = .5
                    self.twist_pub.publish(move)
                else:    
                    move = Twist() 
                    move.angular.z = -1
                    move.linear.x =0
                    self.twist_pub.publish(move)
                self.last_move = "R"
    else:
        if (avg_red_loc <= 480):
            move = Twist() 
            move.angular.z = 1
            move.linear.x =0
            self.twist_pub.publish(move)
        elif (avg_red_loc < 800):
            move = Twist() 
            move.angular.z = 0
            move.linear.x =0.5
            self.twist_pub.publish(move)
        else:
            move = Twist() 
            move.angular.z = -1
            move.linear.x =0
            self.twist_pub.publish(move)

    cv2.imshow("Raw", cv_image) 
    cv2.waitKey(1)

  def stop_robot(self):
    move = Twist() 
    move.angular.z = 0
    move.linear.x = 0
    self.twist_pub.publish(move)

  def message(self, str):
    self.license_plate_pub.publish(str)


def main(args):
  rospy.init_node('pid', anonymous=True)
  robot = PID()


  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)