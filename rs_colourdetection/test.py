#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2", Image)
 
    
    self.bridge = CvBridge()
    """
    Change this to the appropriate topic to obtain the image from
    """
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

  def detect_colors(self, cv_image):
    # Initialize lists to store coordinates
    red_coordinates = []
    green_coordinates = []
    blue_coordinates = []
    
    # Define the lower and upper bounds for red, green, and blue colors in BGR format
    # Refer to resources in README.md for RGB chart map
    lower_red = np.array([0, 0, 96]) # Blue = 0, Green = 0, Red = 100
    upper_red = np.array([0, 0, 224])
    lower_green = np.array([0, 96, 0])
    upper_green = np.array([96, 255, 96])
    lower_blue = np.array([128, 0, 0])
    upper_blue = np.array([255, 32, 32])

    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Create masks for each color
    mask_red = cv2.inRange(cv_image, lower_red, upper_red)
    mask_green = cv2.inRange(cv_image, lower_green, upper_green)
    mask_blue = cv2.inRange(cv_image, lower_blue, upper_blue)

    # Find contours in the masks
    #findContour(source_image, retrieval_method, approximation_method)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find centroids
    for contour in contours_red:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            red_coordinates.append((cX, cY))

    for contour in contours_green:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            green_coordinates.append((cX, cY))

    for contour in contours_blue:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            blue_coordinates.append((cX, cY))

    return red_coordinates, green_coordinates, blue_coordinates

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Detect colors and get coordinates
    red_coords, green_coords, blue_coords = self.detect_colors(cv_image)

    for coords, color in [(red_coords, (0, 0, 0)), (green_coords, (0, 0, 0)), (blue_coords, (0, 0, 0))]:
        for x, y in coords:
            cv2.circle(cv_image, (x, y), 10, color, -1)  # Draw a filled circle
            cv2.putText(cv_image, 'ColourDetected', (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

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
