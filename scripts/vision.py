import json
import numpy as np
import sys
import os
import copy
import cv2
import open3d as o3d
from time import time

import rospy
from ultralytics import YOLO
from cv_bridge import CvBridge
from ur5_project.msg import ObjectPose, ObjectPoseArray
from sensor_msgs.msg import Image, PointCloud2
from ur5_project.srv import VisionResults, VisionResultsResponse

model = YOLO('/home/leo/best.pt', "v8")

#pcd = overall point cloud scene
#from pcd is possible to find every objects point cluod (pcds)

#callback for RGB
def rgb_callback(d):
    global rgb_message
    rgb_message = d

    #convert the ros image to opencv image
    cv_image = CvBridge().imgmsg_to_cv2(rgb_message, desired_encoding = "bgr8")
    #detection with yolo
    detections = model.detect_objects(cv_image)

    for detection in detections:
        label = detection['label']
        confidence = detection['confidence']

        # add label to labels array
        labels.append(label)

#callback fo point cloud
def pc_callback(data):
    global pc_message
    pc_message = data

    #convert PointCloud2 msg to open3d PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(pc_message.data))
    
    #get point cloud for the scene 
    pcds = get_point_clouds(pcd)

    #find the rotation of every object by computing the eigenvectors
    for obj in pcds:
        center = obj["center"]
        dimensions = obj["dimensions"]
        print("Center: ", center)
        
        rotation_matrix = compute_rotation(obj["point_cloud"])

         # save the yaw in "rotations" field of models
         rotations.append(yaw_from_rotation_matrix(rotation_matrix))
         positions.append(center)



#calculate rotation using eigenvalues and eigenvectors
def compute_rotation(pcd):
    #compute covariance matrix
    covariance_matrix = np.cov(np.array(pcd.points).T)
    #compute eigenvectors and eigenvalues of the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
    #find index of maximum eigenvalue
    max_eigenvalue_index = np.argmax(eigenvalues)
    #extract corresponding eigenvector
    rotation_vector = eigenvectors[:, max_eigenvalue_index]
    #convert rotation vector to rotation matrix
    rotation_matrix = np.array([rotation_vector])
    return rotation_matrix

#get point clouds for the scene
def get_point_clouds(scene):
    o3d.utility.set_verbosity_level(o3d.cpu.pybind.utility.VerbosityLevel.Error)
    pcds = []

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(scene.cluster_dbscan(eps=0.02, min_points=10, print_progress=False, ))

    if len(labels) > 0:
        n_label = labels.max() + 1
        objects = []
        for i in range(n_label):
            objects.append([])
        scene_points = np.asarray(scene.points)
        for i, point in enumerate(scene_points):
            objects[labels[i]].append(point)
        for o in objects:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(o)
            bb = pcd.get_axis_aligned_bounding_box()
            center_x = (bb.max_bound[0] + bb.min_bound[0]) / 2.0
            center_y = (bb.max_bound[1] + bb.min_bound[1]) / 2.0
            length = bb.max_bound[0] - bb.min_bound[0]
            width = bb.max_bound[1] - bb.min_bound[1]
            height = bb.max_bound[2] - bb.min_bound[2]
            pcds.append({"point_cloud": pcd, "center": np.array([center_x, center_y]), "dimensions": np.array([length, width, height]), "yolo_data": []})
    return pcds    

#extract yaw (in degrees) from rotation matrix
def yaw_from_rotation_matrix(rot_matrix):
    #extract the elements of the rotation matrix
    r11, r12, r21, r22 = rot_matrix[0, 0], rot_matrix[0, 1], rot_matrix[1, 0], rot_matrix[1, 1]
    #compute the yaw angle using arctan2(angle in radians)
    yaw = np.arctan2(r21, r11)          #radians
    #convert radians to degrees
    yaw_deg = np.degrees(yaw)           #degrees
    
    return yaw_deg


# service handler, it sends the ObjectPoseArray
def handler(req):
    res = ObjectPoseArray()
    poses = []

    # let's iterate position, rotation and label
    for label, position, rotation in zip(labels, positions, rotations):
        object = ObjectPose()
        object.name = label
        object.pose.x = position[0]
        object.pose.y = position[1]
        object.pose.theta = rotation
        object.face = 0

        poses.append(object)
    
    res.poses = poses
    return res


#if main
if __name__ == "__main__":
    print("Starting... ", end="")
    sys.stdout.flush()

    global rgb_subscriber, models, labels, positions, rotations, bridge#, measures_matrix, models

    rospy.init_node('vision_server')

   # bridge = CvBridge()

    rgb_subscriber = rospy.Subscriber('/ur5/zed_node/left_raw/image_raw_color', Image, rgb_callback, queue_size=1)
    pc_subscriber = rospy.Subscriber('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2, pc_callback, queue_size=1)
    #VisionResults = service on which we write an object 
    s = rospy.Service('vision', VisionResults, handler)
    print("Ready.")
    rospy.spin()

