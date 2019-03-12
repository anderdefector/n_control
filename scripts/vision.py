#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def nothing(x):
  pass

class image_converter:

  def __init__(self):
    self.MX = rospy.Publisher("/MX",Int32,queue_size=10)
    self.MY = rospy.Publisher("/MY",Int32,queue_size=10)
    self.Radio = rospy.Publisher("/Radio",Int32,queue_size=10)
    self.CNT = rospy.Publisher("/Contorno",Int32,queue_size=10)
    
    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
    # self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw",Image,self.callback)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.namedWindow("Invertida")
      cv2.namedWindow("Original")

      grayscaled = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

      cv2.createTrackbar("INV","Invertida",0,255,nothing)

      Mask = cv2.getTrackbarPos("INV","Invertida")

      retval, threshold = cv2.threshold(grayscaled, Mask, 255, cv2.THRESH_BINARY_INV)

      im2, contours, hierarchy = cv2.findContours(threshold,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

      font = cv2.FONT_HERSHEY_SIMPLEX
      d=0

      if(len(contours) == 0):
        cv2.putText(cv_image,'NO HAY CONTORNOS',(15,30), font, 1,(255,0,0),2,cv2.LINE_AA)
        c = 0
      else:
        #Obtenecion del area maxima encontrada con los contornos
        area_max = 0.0
        for x in range(0, len(contours)):
          area = cv2.contourArea(contours[x])
          if (area > area_max):
            area_max = area
            r = int(math.sqrt( area_max/3.1416 ))
            d = x
        #Momento del area maxima
        M = cv2.moments(contours[d])
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #Radio del area y esquinas del rectangulo
        # r = int(math.sqrt( area_max/3.1416 ))
        y=(r)*(math.sin(math.radians(45)))
        x=(r)*(math.cos(math.radians(45)))
        x1 = int(cx - x)
        x2 = int(cx + x)
        y1 = int(cy - y)
        y2 = int(cy + y)
        #Presentacion datos sobre imagen
        texto = "Cx = " + str(cx) + " Cy = " + str(cy)
        cv2.putText(cv_image,texto,(15,30), font, 1,(255,0,0),2,cv2.LINE_AA)
        cv2.drawContours(cv_image, contours[d], -1, (255,0,0), 3)
        cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,255,0),3)
        cv2.circle(cv_image,(cx,cy), r, (0,255,0), 3)
        cv2.circle(cv_image,(cx,cy), 7, (255,255,0), -1)
        c = 1

        #Publicacion de mensaje
        self.MX.publish(cx)
        self.MY.publish(cy)
        self.Radio.publish(r)
      self.CNT.publish(c)
      cv2.imshow("Invertida", threshold)
      cv2.imshow("Original", cv_image)

    except CvBridgeError as e:
      print(e)

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