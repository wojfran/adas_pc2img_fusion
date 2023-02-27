#!/usr/bin/env python

# Import necessary ROS libraries
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
import cv2

# Define the ProjectionNode class
class ProjectionNode:
  def __init__(self):
    # Initialize the node
    rospy.init_node('projection_node')

    # Initialize the publisher of the projected image
    # idk what topic we are using so this one is a 
    # placeholder
    self.pub = rospy.Publisher("aeb/projected_image", Image, queue_size=10)

    # Initialize the bridge to convert ROS images to OpenCV images
    self.bridge = CvBridge()

    # Subscribe to the point cloud topic
    # topic name is a placeholder
    rospy.Subscriber('aeb/point_cloud', PointCloud2, self.point_cloud_callback)

    # Subscribe to the image topic
    # topic name is a paceholder
    rospy.Subscriber('aeb/image', Image, self.image_callback)

    # Initialize the point cloud and image variables
    self.point_cloud = None
    self.image = None


  def point_cloud_callback(self, data):
    # Convert the point cloud message to a NumPy array
    self.point_cloud = np.array(list(pc2.read_points(data)))

  def image_callback(self, data):
    # Convert the image message to a OpenCV image
    self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

  def run(self):
    # Set the loop rate
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
      rospy.loginfo('Trying\n')
     # Check if we have both the point cloud and image
      if self.point_cloud is not None and self.image is not None:
        rospy.loginfo('Trying harder\n')
        # Project the point cloud onto the image
        projected_image = self.project_point_cloud(self.point_cloud, self.image)

        msg = self.bridge.cv2_to_imgmsg(projected_image, "bgr8")
        rospy.loginfo('Publishing\n')
        # Publish the resulting image
        self.pub.publish(msg)

      rate.sleep()

  def project_point_cloud(self, point_cloud, image):

    # Projection matrix (3 x 4) for left eye color Camera 2
    P2 = np.matrix('7.215377000000e+02 0.000000000000e+00 6.095593000000e+02 0; '
                  +'0.000000000000e+00 7.215377000000e+02 1.728540000000e+02 0; '
                  +'0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0')
    
    # Rotation matrix of the camera frame axes
    # We could ommit it as we are translating the pc to
    # the camera's reference frame and thus already rotating the
    # axes of the lidar to fit the camera's axes, however, the kitti
    # people used this rotation to make the result better, idk why and how
    # it is almost and identity matrix
    # Edit: it might be due to the fact that the Tr_velo_to_cam matrix
    # translates the pc coordinates to Camera 0 reference frame and
    # the image is from Camera 2
    R0_rect = np.matrix('0.9999239 0.00983776 -0.007445048; '
                      +'-0.009869795 0.9999421 -0.004278459; '
                      +'0.007402527 0.004351614 0.9999631')
    
    # Add a 1 in bottom-right, reshape to 4 x 4
    R0_rect = np.insert(R0_rect,3,values=[0,0,0],axis=0)
    R0_rect = np.insert(R0_rect,3,values=[0,0,0,1],axis=1)
    
    # The leftmost part of the matrix (3x3) is the rotation matrix R, while the rightmost vector is the origin translation vector
    # This matrix is used to translate and rotate the pointcloud from the lidar's reference frame to the camera's reference frame
    Tr_velo_to_cam = np.matrix('7.533745000000e-03 -9.999714000000e-01 -6.166020000000e-04 -4.069766000000e-03; '
                              +'1.480249000000e-02 7.280733000000e-04 -9.998902000000e-01 -7.631618000000e-02; '
                              +'9.998621000000e-01 7.523790000000e-03 1.480755000000e-02 -2.717806000000e-01')
    
    # Add a 1 in bottom-right, reshape to 4 x 4
    Tr_velo_to_cam = np.insert(Tr_velo_to_cam,3,values=[0,0,0,1],axis=0)

    points = point_cloud
    velo = np.insert(points,3,1,axis=1).T
    velo = np.delete(velo,np.where(velo[0,:]<0),axis=1)
    
    
    cam = P2 * R0_rect * Tr_velo_to_cam * velo
    
    
    cam = np.delete(cam,np.where(cam[2,:]<0)[1],axis=1)
    # get u,v,z
    cam[:2] /= cam[2,:]
    # do projection staff
    fig = Figure()
    canvas = FigureCanvas(fig)
    ax = fig.gca()

    ax.text(0.0,0.0,"Test", fontsize=45)
    ax.axis('off')

    canvas.draw()  

    plt.figure(figsize=(12,5),dpi=96,tight_layout=True)
    png = image
    IMG_H,IMG_W,_ = png.shape
    # restrict canvas in range
    plt.axis([0,IMG_W,IMG_H,0])
    plt.imshow(png)
    # filter point out of canvas
    u,v,z = cam
    u_out = np.logical_or(u<0, u>IMG_W)
    v_out = np.logical_or(v<0, v>IMG_H)
    outlier = np.logical_or(u_out, v_out)
    cam = np.delete(cam,np.where(outlier),axis=1)
    # generate color map from depth
    u,v,z = cam
    plt.scatter([u],[v],c=[z],cmap='rainbow_r',alpha=0.5,s=2)
    plt.title("test")
    plt.savefig(f'test.png',bbox_inches='tight')

    image = cv2.imread('test.png')

    return image

# Run the node
if __name__ == '__main__':
  try:
    node = ProjectionNode()
    node.run()
  except rospy.ROSInterruptException:
    pass
