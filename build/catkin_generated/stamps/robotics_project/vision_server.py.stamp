#!/usr/bin/env python3

import json
import sys
import os
import numpy as np
from time import time
import open3d as o3d
import copy

import rospy
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image, PointCloud2
from robotics_project.srv import VisionResults, VisionResultsResponse
from robotics_project.msg import ObjectPose, ObjectPoseArray

from Vision.Block import Block
from Vision.Registration import Registration
from Vision.utilities import *
from Vision.match_blocks import match_block
from Vision.yolo_detection import yolo_detection
from Vision.interpret_rotation import interpret_rotation

DATASET_PATH = "/home/alessandro/last.pt"


def rgbCallback(data):
    global rgb_message
    rgb_message = data


def pcCallback(data):
    global pc_message
    pc_message = data


def handle_detection(req):
    global model, rgb_message, pc_message, models
    global measures_matrix

    img_rgb = bridge.imgmsg_to_cv2(rgb_message, 'bgr8')
    yolo_predictions = yolo_detection(model, measures_matrix, img_rgb)
    # print(yolo_predictions)

    pcds = get_point_clouds(get_gazebo_pc(pc_message))

    for yolo_prediction in yolo_predictions:
        dist_min = -1
        index_dist_min = -1
        for i, pcd in enumerate(pcds):
            dist = np.linalg.norm(pcd["center"] - yolo_prediction["position"])
            if i == 0 or dist < dist_min:
                dist_min = dist
                index_dist_min = i
        pcds[index_dist_min]["yolo_data"].append(
            {"class_name": yolo_prediction["class_name"], "position": yolo_prediction["position"]})

    objects = []
    for pcd in pcds:
        yolo_data = pcd["yolo_data"]
        if not yolo_data:
            continue
        if len(yolo_data) == 1:
            print("Matching... ")
            sys.stdout.flush()

            start = time()
            transformation, model_index = match_block(pcd["point_cloud"], pcd["center"], pcd["dimensions"], models,
                                                      optimise=True)
            end = time()

            print(f"Matched in {round(end - start, 2)}s")

            translation = transformation[0:3, 3:4].flatten()
            block_pcd = copy.deepcopy(models[model_index].pcd)
            block_pcd.transform(transformation)

            np.set_printoptions(suppress=True)
            print("Block:", models[model_index].name)
            print("Position:", translation.round(2))
            print("RPY:", (rot2rpy(transformation) / pi * 180).round(2))
            bb = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.array([
                [0.0, 0.0, -0.01],
                [1.02, 0.645, 0.00]
            ])))
            bb.color = (0, 0, 1)
            # o3d.visualization.draw_geometries([pcd["point_cloud"], block_pcd, bb])

            objects.append({"class_name": models[model_index].name,
                            "position": np.array([transformation[0, 3], transformation[1, 3]]),
                            # "pose": rot2rpy(transformation) / pi * 180})
                            "rotation": transformation[:3, :3]})
        else:
            for yd in yolo_data:
                print("Matching... ")
                sys.stdout.flush()
                start = time()

                transformation, model_index = match_block(pcd["point_cloud"], yd["position"], pcd["dimensions"], models,
                                                          optimise=False)

                end = time()
                print(f"Matched in {round(end - start, 2)}s")

                translation = transformation[0:3, 3:4].flatten()
                block_pcd = copy.deepcopy(models[model_index].pcd)
                block_pcd.transform(transformation)

                np.set_printoptions(suppress=True)
                print("Block:", models[model_index].name)
                print("Position:", translation.round(2))
                print("RPY:", (rot2rpy(transformation) / pi * 180).round(2))
                bb = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.array([
                    [0.0, 0.0, -0.01],
                    [1.02, 0.645, 0.00]
                ])))
                bb.color = (0, 0, 1)
                # o3d.visualization.draw_geometries([pcd["point_cloud"], block_pcd, bb])

                objects.append({"class_name": models[model_index].name,
                                "position": np.array([transformation[0, 3], transformation[1, 3]]),
                                # "pose": rot2rpy(transformation) / pi * 180})
                                "rotation": transformation[:3, :3]})

    print("\n\n\n")
    poses = []
    for object in objects:
        class_name = object["class_name"]
        position = object["position"]
        rotation = object["rotation"]
        face, angle = interpret_rotation(rotation, class_name)

        op = ObjectPose()
        op.name = class_name
        op.pose.x = position[0]
        op.pose.y = position[1]
        op.pose.theta = angle
        op.face = face
        poses.append(op)

        print("Class name:", class_name)
        print("Position:", position)
        # print("Rotation:\n", rotation)
        print(face, angle)

    res = ObjectPoseArray()
    res.poses = poses
    return res


if __name__ == "__main__":
    print("Starting... ", end="")
    sys.stdout.flush()


    global model, rgb_subscriber, bridge, measures_matrix, models

    rospy.init_node('vision_server')

    bridge = CvBridge()
    model = YOLO('/home/alessandro/Downloads/dataset_alberto/scripts/runs/detect/yolov8n_custom5/weights/best.pt', "v8")
    with open("src/robotics_project/scripts/Vision/measures.json") as f:
        measures = json.loads(f.read())
        measures_matrix = []
        for i in range(15):
            measures_matrix.append([k for k in measures[10 * i:10 * (i + 1)]])

    rgb_subscriber = rospy.Subscriber('/ur5/zed_node/left_raw/image_raw_color', Image, rgbCallback, queue_size=1)
    pc_subscriber = rospy.Subscriber('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2, pcCallback,
                                     queue_size=1)

    models_path = 'src/robotics_project/scripts/Vision/Models/'
    models = []
    for entry in os.listdir(models_path):
        if os.path.isfile(os.path.join(models_path, entry)) and entry.endswith(".ply"):
            models.append(Block(models_path + entry, models_path + "poses/" + entry[:-4] + ".json", entry[:-4]))
    print("done!")

    s = rospy.Service('vision', VisionResults, handle_detection)
    print("Ready.")
    rospy.spin()
