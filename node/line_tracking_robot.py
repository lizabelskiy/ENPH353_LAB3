#!/usr/bin/env python3

import roslib
roslib.load_manifest('enph353_ros_lab')
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np
from numpy.lib.function_base import median

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    (rows,cols,channels) = cv_image.shape

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    median_filtered_image = cv2.medianBlur(gray_image, 3)
    _, thresholded_image= cv2.threshold(median_filtered_image, 80, 255, cv2.THRESH_BINARY)

    edges = cv2.Canny(thresholded_image[rows-50:,:], 100, 200)
    _, non_zero_x = np.nonzero(edges)
    
    center_x = int(np.mean(non_zero_x))
    deviation = center_x - cols/2

    speed_factor = 0.6
    deviation_factor = -18
    
    move = Twist()
    move.linear.x = speed_factor * (0.5 - abs(deviation)/cols)
    move.angular.z = deviation_factor * (deviation)/cols
    self.vel_pub.publish(move)

    cv2.circle(cv_image, (center_x, rows-30), 20, (0, 0, 255), -1)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_to tracking_converter', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
