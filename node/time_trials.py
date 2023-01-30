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

class time_trials:

  def __init__(self):
    self.twist_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
    self.license_plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.callback)
    time.sleep(1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Robot POV", cv_image)
    cv2.waitKey(1)

  def move_robot(self):
    move = Twist() 
    move.angular.z = 10
    self.twist_pub.publish(move)

  def stop_robot(self):
    move = Twist() 
    move.angular.z = 0
    self.twist_pub.publish(move)

  def message(self, str):
    self.license_plate_pub.publish(str)


def main(args):
  rospy.init_node('time_trial', anonymous=True)
  tt = time_trials()
  
  tt.message(str("TeamRed,multi12,0,XR58"))
  tt.move_robot()
  rospy.sleep(15.)
  tt.stop_robot()
  tt.message(str("TeamRed,multi21,-1,XR58"))

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)