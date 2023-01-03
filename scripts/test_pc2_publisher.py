#!/usr/bin/env python

# Import necessary ROS libraries
import rospy
import sys
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import numpy as np


def talker():
    pub = rospy.Publisher('aeb/point_cloud', PointCloud2, queue_size=10)
    rospy.init_node('pc2', anonymous=False)
    rate = rospy.Rate(0.20)
    sn = int(sys.argv[1]) if len(sys.argv)>1 else 7 #default 0-7517
    name = '%06d'%sn # 6 digit zeropadding
    binary = (f'{name}.bin')
    points = np.fromfile(binary, dtype=np.float32).reshape((-1,4))
    points = points[:, 0:3]

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()
    
    pc2 = point_cloud2.create_cloud_xyz32(header, points)
    
    while not rospy.is_shutdown():
        rospy.loginfo('Trying\n')
        pub.publish(pc2)
        rate.sleep()



# Run the node
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass