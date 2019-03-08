#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def nothing(x):
  pass

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.namedWindow("Image window")

      grayscaled = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

      cv2.createTrackbar("INV","Image window",0,255,nothing)

      Mask = cv2.getTrackbarPos("INV","Image window")

      retval, threshold = cv2.threshold(grayscaled, Mask, 255, cv2.THRESH_BINARY_INV)

      cv2.imshow("Image window", threshold)

    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", threshold)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)