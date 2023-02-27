#!/usr/bin/env python

# Import necessary ROS libraries
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def talker():
  pub = rospy.Publisher("aeb/image", Image, queue_size=10)
  rospy.init_node('image', anonymous=False)
  bridge = CvBridge()
  rate = rospy.Rate(0.20)
  img = cv2.imread('000007.png')
  msg = bridge.cv2_to_imgmsg(img, 'bgr8')
  while not rospy.is_shutdown():
    rospy.loginfo('Trying\n')
    pub.publish(msg)
    rate.sleep()



# Run the node
if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass