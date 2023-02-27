# PointCloud2_to_Image_Converter_ROS_Node
## Contents:
All ROS nodes are stored in the scripts folder, which contains:
* "pc2_to_img.py" - node which is actually responsible for the projection of the pointcloud2 points onto the image.
* "test_img_publisher.py" - testing node used to publish an image from the KITTI dataset onto the "aeb/image" topic 
subscribed to by the node "pc2_to_img.py".
* "test_pc2_publisher.py" - testing node used to transform a binary file from the KITTI dataset, representing a 
pointloud corresponding to the camera frame of the image published by the previous node. It transforms the data 
into PointCloud2 ROS message and then publishes it onto the "aeb/point_cloud" topic subscribed to by the node "pc2_to_img.py".
* "000007.png" - the aforementioned image from the KITTI dataset.
* "000007.bin" - the aforementioned bin file from the KITTI dataset, storing point cloud data.

The Tests" folder stores images with points projected onto them, they show how the modification or lack of certain elements
in the projection formula affects the final result. I used them to find an understanding of what each matrix really does. It 
contains: 
* "copyFromAzureology.png" - result image acquired by simply replicating the code from https://github.com/azureology/kitti-velo2cam 
and making it work as a ROS node.
* "noR0_rect.png" - result image acquired by not including the R0_rect matrix in the projection formula.
* "P2_rigtRow_setTo_Zeros.png" - result image acquired by setting the right most values of the intrinsic camera projection matrix to 0.
* "modifiedP2_noR0_rect.png" - result image acquired by setting the right most values of the intrinsic camera projection matrix to 0 and
not including the R0_rect matrix in the projection formula.

## Explanation of the projection method:
In the projection equation used in the code i.e. 'cam = P2 * R0_rect * Tr_velo_to_cam * velo' several matrices are used to multiply the 
PointCloud2 points (velo) in order to project them into camera coordinates. Those matrices are:
* Tr_velo_to_cam - This is the extrinsic transformation matrix of the lidar. It is used to transform the lidar's reference frame to the camera's 
reference frame i.e. rotate its axes to match the camera's axes and translate its origin to match the camera's point of origin. The leftmost 
part of the matrix (3x3) is the rotation matrix R, while the rightmost vector is the origin translation vector T. The problem here is that in the case
of the KITTI calibration file the given extrinsic matrix transforms lidar's reference frame to the reference frame of Camera 0 and not Camera 2 (from 
which the image onto which we are trying to project the points originates, due to that some other rectifying steps have to be taken.
* P2 - This is the intrinsic projection matrix (3 x 4) for left eye color Camera 2, it uses the camera's properties such as focal length
to undertake the projection, this is contained to the leftmost 3x3 part of the matrix, however, due to the fact that the lidar's extrinsic 
transformation matrix is aimed at Camera 0 and not Camera 2, an additional column had to be added which basically rectifies that.
* R0_rect - This is a rotation matrix of the camera, from what I could gather, when it is multiplied with the added column of the intrinsic projection
matrix P2 of Camera 2 it rectifies the error of 'Tr_velo_to_cam' being a transformation matrix for Camera 0 instead of Camera 2.

## What it means for us:
In our case the projection formula should be much simpler, as long as we will be fusing only one camera to the lidar data. Our formula should look
something like this :

'cam = Intrinsic * Extrinsic * pc'

With Intrinsic being a 3x3 camera intrinsic projection matrix and Extrinsic being the extrinsic transformation matrix of the lidar. If the axes of the 
camera and the lidar will match naturally then the rotation matrix part of Extrinsic will be most likely a 3x3 identity matrix. The rest will be 
calculated by me in the future.

