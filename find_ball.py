#!/usr/bin/env python

# Image segmentation and publication of pixel position of the ping pong ball via ROS

from __future__ import print_function

import roslib
roslib.load_manifest('camera1')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class find_ball_camera_rf:

  def __init__(self):
      
    # publishes image via ROS
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    
    # publishes coordinates of the center of the ping pong ball via ROS
    self.pub = rospy.Publisher('pixpos', Int32MultiArray, queue_size=1)
    
    #instance of CvBridge, CvBridge converts ROS image data to OpenCV image data
    self.bridge = CvBridge()
    
    #subscribe to the depth camera, everytime the depth camera publishes a new frame the callback function will be run
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)



  def callback(self,data):
      
    try:
        
      # convert the image published by the camera via ROS to a OpenCV image, specifically in BGR format
      original_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
      # display the image for debug purposes
      cv2.imshow("Image window r", original_image)
      
    except CvBridgeError as e:
        
        # if there was an error when converting the image, let the user know
        print(e)
      
        
    ########     IMAGE SEGMENTATION STARTS HERE     ########
    
    # bounds for thershold considering BRG image
    lower_blue = np.array([105,0,0]) 
    upper_blue = np.array([230,230,230])  
        
    # binary mask, pixels in blue threshold set to white, pixels outside threshold set to black
    mask = cv2.inRange(original_image, lower_blue, upper_blue)
    
    # all pixels that are not in the mask, convert to black, all pixels in the mask leave as the original color (blue) 
    blue_only_image = cv2.bitwise_and(original_image, original_image, mask=mask)
    
    # convert the image to grayscale in order to use thresholding function 
    gray_image = cv2.cvtColor(blue_only_image, cv2.COLOR_BGR2GRAY)
    
    # if pixel is not black set it to white (if pixel value exceeds 0 (not black) set to white)
    ret, binary_image = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY)
    
    # make contours around blue pixel areas, then find largest
    contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #take the contour with the max area and select as desired contour
    c = max(contours, key = cv2.contourArea)

    # use moments to find center of contour, equation similar to sum of pixel coordinates in each direction divided by number of pixels
    M = cv2.moments(c)  
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    pix_coord = [cX,cY]
    
    # build message that is of acceptable ROS format and send the pixel coordinates and depth via the ROS master
    msg = Int32MultiArray()
    msg.data = pix_coord
    self.pub.publish(msg)
    
    # draw circle at center of moment
    cv2.circle(binary_image, (cX,cY), 7, (0,0,0), -1)
    # label center
    cv2.putText(binary_image, "center", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
    
    # add the max contour to the image for visualization purposes
    cv2.drawContours(binary_image, [c], 0, (80,0,0), 3)
    
    cv2.imshow("Image window", binary_image)


    try:
        
      # publish final image via ROS master for debugging
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(binary_image, "mono8"))
      
    except CvBridgeError as e:
        
      print(e)


def main(args):
  
  # initialization ROS node so comminications with ROS master can begin
  rospy.init_node('find_ball_camera_rf', anonymous=True)
  
  # upon initialization, a subscriber is made which executes the callback function when new data is recieved
  find_ball = find_ball_camera_rf()
  
  try:
      
    # keeps python from exiting until this node is stopped
    rospy.spin()
    
  except KeyboardInterrupt:
      
    # stop node when user decides
    print("Shutting down")
    
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
