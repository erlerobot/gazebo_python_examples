#!/usr/bin/env python

import roslib
import sys
import rospy
import tf
import cv2
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class map_visualizer:

  def __init__(self):
    self.bridge = CvBridge()
    self.x = 0.0
    self.y = 0.0
    self.ang = 0.0
    self.pos_sub = rospy.Subscriber("/slam_out_pose", PoseStamped, self.pos_callback)
    self.image_sub = rospy.Subscriber("/map_image/full",Image,self.img_callback)

  def pos_callback(self,data):
    self.x = data.pose.position.x
    self.y = data.pose.position.y
    quaternion = data.pose.orientation
    euler = tf.transformations.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
    self.ang = euler[2]

  def img_callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def img_update(self):
    while not rospy.is_shutdown():
      try:
        height, width, channels = self.cv_image.shape
        posx, posy = (width/2 + int(self.x*20)), (height/2 - int(self.y*20))
        cv2.circle(self.cv_image,(int(posx), int(posy)), 10,(255,0,0),10)
        angx = float(math.trunc(50*math.cos(-self.ang)))
        angy = float(math.trunc(50*math.sin(-self.ang)))
        print "[" + str(posx) + "," + str(posy) + "] -> [" + str(posx+angx) + "," + str(posy+angy) + "]"
        cv2.line(self.cv_image,(int(posx), int(posy)), (int(posx + angx), int(posy + angy)), (0,0,255), 10)
        cv2.imshow("Image window", self.cv_image)
        cv2.waitKey(1)
      except AttributeError as e:
        cv2.waitKey(1)  

def main(args):
  mv = map_visualizer()
  cv2.namedWindow('Image window', cv2.WINDOW_OPENGL)
  rospy.init_node('map_visualizer', anonymous=True)
  mv.img_update()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
