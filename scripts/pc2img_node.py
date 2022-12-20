# Import necessary ROS libraries
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Define the ProjectionNode class
class ProjectionNode:
  def __init__(self):
    # Initialize the node
    rospy.init_node('projection_node')

    # Initialize the publisher of the projected image
    # idk what topic we are using so this one is a 
    # placeholder
    rospy.Publisher("aeb/projected_image?", Image, queue_size=10)

    # Initialize the bridge to convert ROS images to OpenCV images
    self.bridge = CvBridge()

    # Subscribe to the point cloud topic
    # topic name is a placeholder
    rospy.Subscriber('aeb/point_cloud?', PointCloud2, self.point_cloud_callback)

    # Subscribe to the image topic
    # topic name is a paceholder
    rospy.Subscriber('aeb/image?', Image, self.image_callback)

    # Initialize the point cloud and image variables
    self.point_cloud = None
    self.image = None

    # Set the camera matrix and distortion coefficients
    # This will need to be evaluated once we know the position
    # of the camera and the lidar on the car
    self.camera_matrix = np.array([[fx, 0, cx],
                                   [0, fy, cy],
                                   [0, 0, 1]])
    
    # In theory the distortion coefficients can be not given
    # to the cv2.projectPoints function but this assumes that
    # the camera has no distortion
    self.dist_coeffs = np.array([k1, k2, p1, p2, k3])

    # Set the rotation and translation vectors
    # This will need to be evaluated once we know the position
    # of the camera and the lidar on the car
    self.rvec = rvec
    self.tvec = tvec

    # Create the lookup table for the colors
    self.color_lut = self.create_color_lut()

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
      # Check if we have both the point cloud and image
      if self.point_cloud is not None and self.image is not None:
        # Project the point cloud onto the image
        projected_image = self.project_point_cloud(self.point_cloud, self.image)

        # Publish the resulting image
        self.pub.publish('projected_image', self.bridge.cv2_to_imgmsg(projected_image, "bgr8"))

      rate.sleep()

  def project_point_cloud(self, point_cloud, image):
    # Convert the point cloud to homogeneous coordinates
    homogeneous_points = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))

    # Transform the points to the camera frame using the extrinsic parameters
    camera_points, _ = cv2.projectPoints(homogeneous_points, self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)

    # Convert the points back to non-homogeneous coordinates
    camera_points = np.int32(camera_points.reshape(-1, 2))

    # Compute the distance of each point from the camera
    distances = np.linalg.norm(point_cloud, axis=1)

    # Use the lookup table to map the distances to colors
    colors = self.color_lut[distances.astype(np.uint8)]

    # Create a list of keypoints from the points and colors
    keypoints = [cv2.KeyPoint(x, y, 2, _class_id=i) for i, (x, y) in enumerate(camera_points)]

    # Draw the keypoints on the image
    cv2.drawKeypoints(image, keypoints, image, color=colors, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    return image

  def create_color_lut(self):
    # Define the color map
    color_map = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]

    # Normalize the distances between 0 and 255
    distances = np.linalg.norm(self.point_cloud, axis=1)
    distances = (distances - np.min(distances)) / (np.max(distances) - np.min(distances))
    distances *= 255

    # Create the lookup table
    color_lut = np.empty((256, 3)
    for i in range(256):
      index = int(i / 255 * (len(color_map) - 1))
      color_lut[i] = color_map[index]

    return color_lut

# Run the node
if __name__ == '__main__':
  try:
    node = ProjectionNode()
    node.run()
  except rospy.ROSInterruptException:
    pass
