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
  sn = int(sys.argv[1]) if len(sys.argv)>1 else 7 #default 0-7517
  name = '%06d'%sn # 6 digit zeropadding
  img = cv2.imread(f'{name}.png')
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