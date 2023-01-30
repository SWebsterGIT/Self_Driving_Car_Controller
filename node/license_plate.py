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
import os
import numpy as np

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers
from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend

class license_plate:

  def __init__(self):
    self.conv_model = models.load_model("/home/fizzer/cnn_trainer/license_plate_cnn/PLATE_MODEL_SMALL_WITH_AUG_14_EPOCH")
    self.bridge = CvBridge()
    self.parking_spot = "2"
    self.current_plate_guess = "AA00"
    self.team = "Neam"
    self.team_password = "chongus"
    self.pred_changed = False

    self.plate_to_highest_confidence = {
      "1":[0,0,0,0],
      "2":[0,0,0,0],
      "3":[0,0,0,0],
      "4":[0,0,0,0],
      "5":[0,0,0,0],
      "6":[0,0,0,0]
    }

    self.plate_to_best_pred = {
      "1":["A", "A", "0", "0"],
      "2":["A", "A", "0", "0"],
      "3":["A", "A", "0", "0"],
      "4":["A", "A", "0", "0"],
      "5":["A", "A", "0", "0"],
      "6":["A", "A", "0", "0"]
    }

    self.max_contoursize = 0
    self.highest_confidences = [0,0,0,0]
    self.highest_index = [0,0,0,0]
    
    self.current_image = None
    self.plate_img = None
    self.y_is_pos = False

    self.license_plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)
    self.parking_spot_sub = rospy.Subscriber('/parking_spot', String, self.parking_spot_callback)
    self.image_sub = rospy.Subscriber('/R1/pi_camera/image_raw', Image, self.image_callback)

    time.sleep(1)


  def image_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.current_image = cv_image

    #cv2.imshow("Robot POV", self.current_image)

    self.HSV_Processing_func()
    cv2.waitKey(1)

  def parking_spot_callback(self,data):
    if (self.parking_spot != str(data)[-2]):
      for i in range(0,4):
            if self.plate_to_highest_confidence[self.parking_spot][i] < self.highest_confidences[i]:
              self.plate_to_best_pred[self.parking_spot][i] = self.current_plate_guess[i]
              self.plate_to_highest_confidence[self.parking_spot][i] = self.highest_confidences[i]
              self.pred_changed = True
      if (self.pred_changed):
        plate =""
        for i in range(0,4):
          plate += self.plate_to_best_pred[self.parking_spot][i]
        self.post_license_plate(plate)
      self.max_contoursize = 0
      self.highest_confidences = [0,0,0,0]
      self.highest_index = [0,0,0,0]
      self.pred_changed = False
      print(str(data)[-2])

    self.parking_spot = str(data)[-2]
    
  def message(self, str):
    self.license_plate_pub.publish(str)

  def start_timer(self):
    self.message("Team" + self.team + "," + self.team_password + ",0,XR58")

  def end_timer(self):
    self.message("Team" + self.team + "," + self.team_password + ",-1,XR58")

  def post_license_plate(self, plate):
    self.message( "Team" + self.team + "," + self.team_password + "," + str(self.parking_spot) + "," + str(plate) )

  

  def HSV_Processing_func(self):

    low_grey_thresh = (0, 0, 90)
    high_grey_thresh = (179, 1, 115)

    if(self.parking_spot == '3'):
      low_grey_thresh = (0, 0, 116)
      high_grey_thresh = (1, 1, 127)

    if(self.parking_spot == '4' or self.parking_spot == '5'):
      low_grey_thresh = (0, 0, 197)
      high_grey_thresh = (1, 1, 204)

    hsv_car = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)

    #hsv_car = cv2.bilateralFilter(hsv_car,5,10,10) ADD SOME KIND OF BLUR

    car_grey_strip_mask = cv2.inRange(hsv_car, low_grey_thresh,high_grey_thresh)

    car_grey_strip_hsv = cv2.bitwise_and(hsv_car, hsv_car, mask = car_grey_strip_mask)

    car_grey_strip_bgr = cv2.cvtColor(car_grey_strip_hsv,cv2.COLOR_HSV2BGR)

    contours, hierarchy = cv2.findContours(car_grey_strip_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0:
      # draw in blue the contours that were found
      #cv2.drawContours(car_grey_strip_bgr, contours, -1, 255, 3)

      # find the biggest countour (c) by the area
      biggest_contour = max(contours, key = cv2.contourArea)

      #print(cv2.contourArea(biggest_contour))

      cv2.drawContours(car_grey_strip_bgr, biggest_contour, -1, 255, 3)
      xb,yb,wb,hb = cv2.boundingRect(biggest_contour)

      
      perimeter = cv2.arcLength(biggest_contour, True)
      approx = cv2.approxPolyDP(biggest_contour, 0.05 * perimeter, True)

      pov_clone = self.current_image.copy()

      pov_clone2 = self.current_image.copy()

      # drawing points on corners
      for point in approx:
          x, y = point[0]
          cv2.circle(pov_clone, (x, y), 3, (0, 255, 0), -1)

      font = cv2.FONT_HERSHEY_SIMPLEX
      org = (50, 50)
      fontScale = 1
      color = (255, 0, 0)
      thickness = 2
        
      # Using cv2.putText() method
      corners_numbered = cv2.putText(pov_clone, str(len(approx)), org, font, 
                        fontScale, color, thickness, cv2.LINE_AA)

      # drawing quadrilateral that is approximating the contours
      cv2.drawContours(pov_clone, [approx], -1, (0, 255, 0))
      #cv2.imshow("corners", corners_numbered)

      if (len(approx) == 4):
        pts1 = np.float32([point[0] for point in approx])
        pts1_x_sorted = pts1[pts1[:,0].argsort()]

        lower_x = pts1_x_sorted[0:2]
        lower_x_sorted = lower_x[lower_x[:,1].argsort()]
        
        higher_x = pts1_x_sorted[2:]
        higher_x_sorted = higher_x[higher_x[:,1].argsort()]
      
        pts1_fully_sorted = np.concatenate((lower_x_sorted, higher_x_sorted), axis=0)

        SCALE = 1
        WIDTH = 600*SCALE
        HEIGHT = 1500*SCALE
        H_FRAC = 0.80

        pts2_crop = np.float32([[0,0],[0,HEIGHT*H_FRAC],[WIDTH,0],[WIDTH,HEIGHT*H_FRAC]])

        matrix2 = cv2.getPerspectiveTransform(pts1_fully_sorted,pts2_crop)

        warped_img2 = cv2.warpPerspective(pov_clone2,matrix2,(WIDTH,HEIGHT))

        #cv2.imshow("CROPPED", warped_img2)

        if (cv2.contourArea(biggest_contour) > self.max_contoursize):
          self.plate_img = warped_img2[int(HEIGHT*H_FRAC):,:]
          #cv2.imshow("plate", self.plate_img)
          self.max_contoursize = cv2.contourArea(biggest_contour)

          #hsv filtering to get letters on their own: can blur the image a bit, though.
          
          # hsv_plate = cv2.cvtColor(self.plate_img, cv2.COLOR_BGR2HSV)
          # low_blue_thresh = (72,125,30)
          # high_blue_thresh = (179,255,255)
          # plate_img_binary = cv2.inRange(hsv_plate, low_blue_thresh,high_blue_thresh)
          # cv2.imshow("binary plate", plate_img_binary)

          w = 100
          x0 = 45
          x1= x0 + w
          x2= x1 + w
          x3 = 345
          x4 = x3+w
          x5 = x4+w
          y0 = 80
          y1 = 250

          X_dataset_orig =[]
          Y_dataset =[]

          cropped_img1 = self.plate_img[y0:y1, x0:x1].copy()
          #cv2.imshow("1", cropped_img1)
          X_dataset_orig.append(cropped_img1)

          cropped_img2 = self.plate_img[y0:y1, x1:x2].copy()
          #cv2.imshow("2", cropped_img2)
          X_dataset_orig.append(cropped_img2)

          cropped_img3 = self.plate_img[y0:y1, x3:x4].copy()
          #cv2.imshow("3", cropped_img3)
          X_dataset_orig.append(cropped_img3)

          cropped_img4 = self.plate_img[y0:y1, x4:x5].copy()
          #cv2.imshow("4", cropped_img4)
          X_dataset_orig.append(cropped_img4)

          X_dataset = np.array(X_dataset_orig)/255

          #Get predictions for each image in the data set
          y_predict = self.conv_model.predict(X_dataset)

          y_pred =[]
          #Gets the predicted index for each image
          for i in range(0,4):
            if (i==0) or (i==1):
              pred = y_predict[i][:26]
              y_pred.append(np.argmax(pred))
            if (i == 2) or (i==3):
              pred = y_predict[i][26:]
              y_pred.append(np.argmax(pred)+26)

          #y_pred = [np.argmax(pred) for pred in y_predict]
          conf_max = [np.max(pred) for pred in y_predict]

          # implement miti's strat here

          for i in range(0,4):
            if conf_max[i] > self.highest_confidences[i]:
              self.highest_index[i] = y_pred[i]
              self.highest_confidences[i] = conf_max[i]

          char_dict={
            0:"A",
            1:"B",
            2:"C",
            3:"D",
            4:"E",
            5:"F",
            6:"G",
            7:"H",
            8:"I",
            9:"J",
            10:"K",
            11:"L",
            12:"M",
            13:"N",
            14:"O",
            15:"P",
            16:"Q",
            17:"R",
            18:"S",
            19:"T",
            20:"U",
            21:"V",
            22:"W",
            23:"X",
            24:"Y",
            25:"Z",
            26:"0",
            27:"1",
            28:"2",
            29:"3",
            30:"4",
            31:"5",
            32:"6",
            33:"7",
            34:"8",
            35:"9"
          }

          letters = [char_dict.get(arg) for arg in self.highest_index]
          #print(self.highest_confidences)
          #print(str(letters))

          new_string = ""

          for c in letters:
            new_string += c

          self.current_plate_guess = new_string

          #print(self.current_plate_guess)

          prediction = [char_dict.get(arg) for arg in y_pred]
          #print(conf_max)
          #print(prediction)


      # draw the biggest contour's bounding rectangle in green
      #cv2.rectangle(car_grey_strip_bgr,(xb,yb),(xb+wb,yb+hb),(0,255,0),2)
      #cv2.imshow("Contours", car_grey_strip_bgr)

def main(args):
  rospy.init_node('license_plate', anonymous=True)
  lp = license_plate()

  lp.start_timer()

  rospy.sleep(90)

  lp.end_timer()

  print(lp.plate_to_highest_confidence)
  print(lp.plate_to_best_pred)
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)