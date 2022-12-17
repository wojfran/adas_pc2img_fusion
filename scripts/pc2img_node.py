#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib as plt
import message_filters
import cv2
import os
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image

def read_calib_file(filepath):
    """
    Read in a calibration file and parse into a dictionary.
    Ref: https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
    """
    data = {}
    with open(filepath, 'r') as f:
        for line in f.readlines():
            line = line.rstrip()
            if len(line) == 0: continue
            key, value = line.split(':', 1)
            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass

    return data

def load_velo_scan(velo_filename):
    scan = np.fromfile(velo_filename, dtype=np.float32)
    scan = scan.reshape((-1, 4))
    return scan

def project_velo_to_cam2(calib):
    P_velo2cam_ref = np.vstack((calib['Tr_velo_to_cam'].reshape(3, 4), np.array([0., 0., 0., 1.])))  # velo2ref_cam
    R_ref2rect = np.eye(4)
    R0_rect = calib['R0_rect'].reshape(3, 3)  # ref_cam2rect
    R_ref2rect[:3, :3] = R0_rect
    P_rect2cam2 = calib['P2'].reshape((3, 4))
    proj_mat = P_rect2cam2 @ R_ref2rect @ P_velo2cam_ref
    return proj_mat

def project_to_image(points, proj_mat):
    """
    Apply the perspective projection
    Args:
        pts_3d:     3D points in camera coordinate [3, npoints]
        proj_mat:   Projection matrix [3, 4]
    """
    num_pts = points.shape[1]

    # Change to homogenous coordinate
    points = np.vstack((points, np.ones((1, num_pts))))
    points = proj_mat @ points
    points[:2, :] /= points[2, :]
    return points[:2, :]

def render_lidar_on_image(pts_3d, img, calib, img_width, img_height):
    # projection matrix (project from velo2cam2)
    proj_lidar2cam2 = project_velo_to_cam2(calib)

    # apply projection
    pts_2d = project_to_image(pts_3d.transpose(), proj_lidar2cam2)

    # if the filters end up not working they can be either commented out
    # or replaced with o3d_cloud_pass_through_filter funcions from
    # open3d-ros-helper

    # Filter lidar points to be within image FOV
    inds = np.where((pts_2d[0, :] < img_width) & (pts_2d[0, :] >= 0) &
                    (pts_2d[1, :] < img_height) & (pts_2d[1, :] >= 0) &
                    (pts_3d[:, 0] > 0)
                    )[0]

    # Filter out pixels points
    imgfov_pc_pixel = pts_2d[:, inds]

    # Retrieve depth from lidar
    imgfov_pc_lidar = pts_3d[inds, :]
    imgfov_pc_lidar = np.hstack((imgfov_pc_lidar, np.ones((imgfov_pc_lidar.shape[0], 1))))
    imgfov_pc_cam2 = proj_lidar2cam2 @ imgfov_pc_lidar.transpose()

    cmap = plt.cm.get_cmap('hsv', 256)
    cmap = np.array([cmap(i) for i in range(256)])[:, :3] * 255

    for i in range(imgfov_pc_pixel.shape[1]):
        depth = imgfov_pc_cam2[2, i]
        color = cmap[int(640.0 / depth), :]
        cv2.circle(img, (int(np.round(imgfov_pc_pixel[0, i])),
                         int(np.round(imgfov_pc_pixel[1, i]))),
                   2, color=tuple(color), thickness=-1)
    plt.imshow(img)
    plt.yticks([])
    plt.xticks([])
    plt.show()
    return img

def converter_callback(pointcloud, image):
    # Load image, calibration file, label bbox
    rgb = cv2.cvtColor(cv2.imread(os.path.join('data/000114_image.png')), cv2.COLOR_BGR2RGB)
    img_height, img_width, img_channel = rgb.shape

    # Load calibration
    # idk if it won't be better to just write the calibration matrix 
    # directly here or somewhere else in the node
    calib = read_calib_file('data/000114_calib.txt')

    # Load Lidar PC
    # This will be replaced with pointcloud 3d point data
    # from the pointcloud topic
    pc_velo = load_velo_scan('data/000114.bin')[:, :3]

    render_lidar_on_image(pc_velo, rgb, calib, img_width, img_height)


if __name__ == '__main__':
    rospy.init_node("pointcloud_to_image_converter")
    pub = rospy.Publisher("/aeb/deepimage?", Image, queue_size=10)
    pc2_sub = message_filters.Subscriber('pointcloud', point_cloud2, callback=converter_callback)
    img_sub = message_filters.Subscriber('image', Image, callback=converter_callback)
    rospy.loginfo("Converter node has started.")


    ts = message_filters.TimeSynchronizer([pc2_sub, img_sub],10)
    ts.registerCallback(converter_callback)
    rospy.spin()