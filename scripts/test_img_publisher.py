#!/usr/bin/env python

# Import necessary ROS libraries
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TestImageReadNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('test_image_read_node')

        # Initialize the publisher of the projected image
        # idk what topic we are using so this one is a 
        # placeholder
        rospy.Publisher("aeb/image", Image, queue_size=10)

        sn = int(sys.argv[1]) if len(sys.argv)>1 else 7 #default 0-7517
        name = '%06d'%sn # 6 digit zeropadding
        self.img = cv2.imread(f'{name}.png', "bgr8")

    def run(self):
    # Set the loop rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Publish the resulting image
            self.pub.publish('projected_image', self.bridge.cv2_to_imgmsg(self.img, "bgr8"))

            rate.sleep()

# Run the node
if __name__ == '__main__':
  try:
    node = TestImageReadNode()
    node.run()
  except rospy.ROSInterruptException:
    pass