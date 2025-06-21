#!/usr/bin/env python3

import os.path
import json

import sys
import cv2
import depthai as dai
import numpy as np
import time

import rospy
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import rospkg

import tf
import tf2_ros
import geometry_msgs.msg

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




def publisher_bbox(bbox_data_list):
    """
    Function to publish the bounding box information as a Detection2DArray message
    """
    msg = Detection2DArray()
    header = Header()
    header.stamp = rospy.Time.now()
    msg.header = header
    detection_2d_list = []

    for i in bbox_data_list:
        detection_2d_msg = Detection2D()
        detection_2d_msg.header = header
        bounding_box_2d = BoundingBox2D()
        pose_2d = Pose2D()
        pose_2d.x = i["x"]
        pose_2d.y = i["y"]
        pose_2d.theta = i["confidence"]
        bounding_box_2d.center = pose_2d
        bounding_box_2d.size_x = i["width"]
        bounding_box_2d.size_y = i["height"]
        detection_2d_msg.bbox = bounding_box_2d
        detection_2d_list.append(detection_2d_msg)

    msg.detections = detection_2d_list
    pub_bbox.publish(msg)

def publisher_position(data):
    """
    Function to publish the position information as a tf message
    data: (x,y) position
    """
    br = tf.TransformBroadcaster()
    br.sendTransform((data["x"], data["y"], 0),(0,0,0,1),rospy.Time.now(),"Mercator_1", "odom")

def publisher_images_post_proc(frame):
    """
    Function to publish the frame as ROS messages
    """
    bridge = CvBridge()
    if frame is None:
        return
    frame_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    pub_frame.publish(frame_msg)

def get_config_from_json(json_filename):
    """
    Function to read configuration parameters from a JSON file
    """
    f = open(json_filename)
    jsonData = json.load(f)
    f.close()
    confidenceThreshold =  0.01 # jsonData["nn_config"]["NN_specific_metadata"]["confidence_threshold"]
    numClasses = jsonData["nn_config"]["NN_specific_metadata"]["classes"]
    anchors = jsonData["nn_config"]["NN_specific_metadata"]["anchors"]
    anchorMasks = jsonData["nn_config"]["NN_specific_metadata"]["anchor_masks"]
    coordinateSize = jsonData["nn_config"]["NN_specific_metadata"]["coordinates"]
    iouThreshold = jsonData["nn_config"]["NN_specific_metadata"]["iou_threshold"]
    inputSize = jsonData["nn_config"]["input_size"]
    inputSizeX, inputSizeY = inputSize.split("x")
    inputSizeX = int(inputSizeX)
    inputSizeY = int(inputSizeY)
    labelMap = jsonData["mappings"]["labels"]

    return confidenceThreshold, numClasses, anchors, anchorMasks, \
        coordinateSize, iouThreshold, inputSizeX, inputSizeY, labelMap

def visualize_detection(frame, detection, labelMap, box_position):
    try:
        label = labelMap[detection.label]  # Get the label of the detection from the label map
    except:
        label = detection.label
    x1, y1, x2, y2 = box_position
    cv2.putText(frame, str(label), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))  # Draw the label on the frame
    cv2.putText(frame, f"Conf: {int(detection.confidence * 100)} %", (x1, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.3, (0, 0, 0))  # Draw the confidence percentage on the frame
    cv2.putText(frame, f"x: {int(detection.spatialCoordinates.x/10)} cm", (x1, y1 + 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.3, (0, 0, 0))  # Draw the x-coordinate of the spatial position on the frame
    cv2.putText(frame, f"y: {int(detection.spatialCoordinates.z/10)} cm", (x1, y1 + 40), cv2.FONT_HERSHEY_SIMPLEX,
                0.3, (0, 0, 0))  # Draw the y-coordinate of the spatial position on the frame
    cv2.putText(frame, f"z: {int(detection.spatialCoordinates.y/10)} cm", (x1, y1 + 50), cv2.FONT_HERSHEY_SIMPLEX,
                0.3, (0, 0, 0))  # Draw the z-coordinate of the spatial position on the frame

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,0,255), cv2.FONT_HERSHEY_SIMPLEX)  # Draw the bounding box on the frame



def start_oak_camera(blob_filename, json_filename, visualize, compressed=True, offset=(0,0), IR=False):

    (confidenceThreshold, numClasses, anchors, anchorMasks,
     coordinateSize, iouThreshold, inputSizeX, inputSizeY, labelMap) = get_config_from_json(json_filename)

    # Define camera pipeline and its components
    pipeline = dai.Pipeline()

    # Define needed components
    camRgb = pipeline.create(dai.node.ColorCamera)
    yolo_spatial_detection_network = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    nnNetworkOut = pipeline.create(dai.node.XLinkOut)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutNN = pipeline.create(dai.node.XLinkOut)
    xoutDepth = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")
    xoutNN.setStreamName("detections")
    xoutDepth.setStreamName("depth")
    nnNetworkOut.setStreamName("nnNetwork")

    # RGB camera setup
    camRgb.setPreviewSize(inputSizeX, inputSizeY)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    # If NN take horizontally compressed images as input. Increases FOV
    if compressed == True:
        camRgb.setPreviewKeepAspectRatio(False)

    # Setting resolution for depth 800P OR 400P
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # setting node configs
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # Change the depth frame alignment to the center camera, by default set to the right camera
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

    # Setting neural network parameters
    yolo_spatial_detection_network.setBlobPath(blob_filename)
    yolo_spatial_detection_network.setConfidenceThreshold(confidenceThreshold)
    yolo_spatial_detection_network.input.setBlocking(False)
    yolo_spatial_detection_network.setBoundingBoxScaleFactor(0.5)
    yolo_spatial_detection_network.setDepthLowerThreshold(100)
    yolo_spatial_detection_network.setDepthUpperThreshold(10000)
    # Calculation method for depth : can be set to AVERAGE,MIN,MAX,MEDIAN,MODE
    yolo_spatial_detection_network.setSpatialCalculationAlgorithm(dai.SpatialLocationCalculatorAlgorithm.AVERAGE)

    # Yolo specific parameters
    # https://docs.luxonis.com/projects/api/en/latest/components/nodes/yolo_spatial_detection_network/
    yolo_spatial_detection_network.setNumClasses(numClasses)
    yolo_spatial_detection_network.setCoordinateSize(coordinateSize)
    yolo_spatial_detection_network.setAnchors(anchors)
    yolo_spatial_detection_network.setAnchorMasks(anchorMasks)
    yolo_spatial_detection_network.setIouThreshold(iouThreshold)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    camRgb.preview.link(yolo_spatial_detection_network.input)
    yolo_spatial_detection_network.passthrough.link(xoutRgb.input)
    yolo_spatial_detection_network.out.link(xoutNN.input)
    stereo.depth.link(yolo_spatial_detection_network.inputDepth)
    yolo_spatial_detection_network.passthroughDepth.link(xoutDepth.input)
    yolo_spatial_detection_network.outNetwork.link(nnNetworkOut.input)

    # Connect to device and start processing
    with dai.Device(pipeline) as device:
        if IR == True:
            device.setIrLaserDotProjectorBrightness(800)  # in mA, 0..1200
            # device.setIrFloodLightBrightness(200)

        previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False);

        previous_time = time.time()
        counter = 0
        fps = 0

        while True:
            inPreview = previewQueue.get()
            inDet = detectionNNQueue.get()
            depth = depthQueue.get()

            frame = inPreview.getCvFrame()

            counter += 1
            current_time = time.time()
            if (current_time - previous_time) > 1:
                fps = counter / (current_time - previous_time)
                previous_time = current_time
                counter = 0
            detections = inDet.detections

            height, width = frame.shape[0], frame.shape[1]

            detections_position = PoseArray()
            bbox_data_list = []
            for detection in detections:
                bbox_data = dict()
                position_data = dict()

                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)

                # Bounding box info for detection ros msg
                bbox_data["x"] = (x1 + x2) / 2
                bbox_data["y"] = (y1 + y2) / 2
                bbox_data["width"] = x2 - x1
                bbox_data["height"] = y2 - y1
                bbox_data["confidence"] = detection.confidence

                # Position info for tf message msg
                offset_x, offset_y = offset
                position_data["x"] = ((detection.spatialCoordinates.x) + offset_x) / 1000
                position_data["y"] = ((detection.spatialCoordinates.z) + offset_y) / 1000

                detections_position.poses.append(Pose(Point(position_data["x"], position_data["y"], 0), geometry_msgs.msg.Quaternion(0, 0, 0, 1)))
                bbox_data_list.append(bbox_data)
                publisher_position(position_data)  # Publish the position data

                if visualize == True:
                    visualize_detection(frame, detection, labelMap,
                                        (x1, y1, x2, y2))  # Visualize the detection on the frame
                    # Publish frame as ROS Image message
                    publisher_images_post_proc(frame)
            rospy.loginfo(f"[oak_detector] {len(bbox_data_list)} bounding boxes prêtes à publier") 
            pub_bbox.publish(Detection2DArray(header=Header(stamp=rospy.Time.now()), detections=[]))
            # Publish the position data of each detection in a PoseArray message to the cam_poses topic    
            pub_poses.publish(detections_position)
            publisher_bbox(bbox_data_list) # todo: salman
            if visualize == True:
                cv2.putText(frame, f"FPS: {int(fps)}", (0, frame.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX,
                           0.5, (0, 0, 0))  # Draw the FPS on the frame
                # Publish frame as ROS Image message
                # publisher_images_post_proc(frame)
                cv2.imshow("Detections with position", frame)
                if cv2.waitKey(1) == ord('q'):
                    break


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: /path/oak_detector.py /path/blob_file_name /path/json_file_name")
        exit(1)
    else:
        blob_filename = sys.argv[1]
        json_filename = sys.argv[2]
    if len(sys.argv) == 4 and sys.argv[3] == "visualize":
        visualize = True
    else:
        visualize = False

    rospy.init_node("oak_detector", anonymous=True)
    pub_bbox = rospy.Publisher("oak", Detection2DArray, queue_size=1)
    pub_frame = rospy.Publisher("oak_frames", Image, queue_size=1)
    pub_poses = rospy.Publisher("cam_poses", PoseArray, queue_size=1)

    start_oak_camera(blob_filename, json_filename, visualize)

