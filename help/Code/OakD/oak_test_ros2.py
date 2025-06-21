#!/usr/bin/env python3
import sys
import json
import time

import rospy
import depthai as dai
import cv2

from std_msgs.msg       import Header
from sensor_msgs.msg    import Image
from vision_msgs.msg    import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg  import PoseArray, Pose, Point, Quaternion
import tf
from cv_bridge import CvBridge

def publisher_bbox(bbox_data_list):
    """
    Publish a Detection2DArray on /oak containing all bounding boxes.
    """
    msg = Detection2DArray()
    msg.header = Header(stamp=rospy.Time.now())
    for bb in bbox_data_list:
        det2d = Detection2D()
        det2d.header = msg.header
        box = BoundingBox2D()
        box.center.x = bb["x"]
        box.center.y = bb["y"]
        box.size_x   = bb["width"]
        box.size_y   = bb["height"]
        det2d.bbox = box
        msg.detections.append(det2d)
    pub_bbox.publish(msg)

def publisher_position(data):
    """
    Publish a TF between "odom" and "Mercator_1" representing the robot position.
    """
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (data["x"], data["y"], 0),
        (0, 0, 0, 1),
        rospy.Time.now(),
        "Mercator_1",
        "odom"
    )

def publisher_images_post_proc(frame):
    """
    Convert an OpenCV frame to ROS Image and publish on /oak_frames.
    """
    bridge = CvBridge()
    img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    img_msg.header = Header(stamp=rospy.Time.now())
    pub_frame.publish(img_msg)

def get_config_from_json(json_filename):
    """
    Read all the necessary parameters from the JSON.
    """
    with open(json_filename, 'r') as f:
        j = json.load(f)
    meta = j["nn_config"]["NN_specific_metadata"]
    input_w, input_h = map(int, j["nn_config"]["input_size"].split("x"))
    return {
        "confidenceThreshold": meta["confidence_threshold"],
        "numClasses":         meta["classes"],
        "anchors":            meta["anchors"],
        "anchorMasks":        meta["anchor_masks"],
        "coordinateSize":     meta["coordinates"],
        "iouThreshold":       meta["iou_threshold"],
        "inputW":             input_w,
        "inputH":             input_h
    }

def visualize_detection(frame, detection, box_position):
    """
    Draw label, confidence and rectangle on frame.
    """
    x1, y1, x2, y2 = box_position
    cv2.rectangle(frame, (x1,y1), (x2,y2), (0,0,255), 2)
    conf = int(detection.confidence * 100)
    cv2.putText(frame, f"{conf}%", (x1, y1-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

def start_oak_camera(blob_fn, json_fn, visualize=False, IR=False):
    # 1) Configuration
    cfg = get_config_from_json(json_fn)
    conf_thresh = cfg["confidenceThreshold"]
    num_classes = cfg["numClasses"]
    anchors     = cfg["anchors"]
    masks       = cfg["anchorMasks"]
    coord_size  = cfg["coordinateSize"]
    iou_thresh  = cfg["iouThreshold"]
    w, h        = cfg["inputW"], cfg["inputH"]

    # 2) Pipeline
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setPreviewSize(w, h)
    cam.setInterleaved(False)

    # Spatial network (vous pouvez aussi passer à YoloDetectionNetwork 2D si vous n’avez pas besoin de profondeur)
    yolo = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    yolo.setBlobPath(blob_fn)
    yolo.setConfidenceThreshold(conf_thresh)
    yolo.setNumClasses(num_classes)
    yolo.setCoordinateSize(coord_size)
    yolo.setAnchors(anchors)
    yolo.setAnchorMasks(masks)
    yolo.setIouThreshold(iou_thresh)
    cam.preview.link(yolo.input)

    # XLink pour les détections
    xout_det = pipeline.create(dai.node.XLinkOut)
    xout_det.setStreamName("detections")
    yolo.out.link(xout_det.input)

    # XLink pour les frames
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    yolo.passthrough.link(xout_rgb.input)

    # 3) Exécution
    with dai.Device(pipeline) as device:
        if IR:
            device.setIrLaserDotProjectorBrightness(800)

        q_det   = device.getOutputQueue("detections", 4, blocking=False)
        q_frame = device.getOutputQueue("rgb",        4, blocking=False)
        rate    = rospy.Rate(20)

        while not rospy.is_shutdown():
            # a) Récupère et publie l'image
            in_frame = q_frame.tryGet()
            if in_frame:
                frame = in_frame.getCvFrame()
                publisher_images_post_proc(frame)

            # b) Récupère les détections
            in_det = q_det.tryGet()
            dets   = in_det.detections if in_det else []

            # c) Prépare et publie bounding boxes + TF + PoseArray
            bbox_list         = []
            detections_pose   = PoseArray()
            detections_pose.header = Header(stamp=rospy.Time.now())

            for d in dets:
                x1 = int(d.xmin * w)
                x2 = int(d.xmax * w)
                y1 = int(d.ymin * h)
                y2 = int(d.ymax * h)
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                sx = x2 - x1
                sy = y2 - y1

                bbox_list.append({
                    "x":      cx,
                    "y":      cy,
                    "width":  sx,
                    "height": sy
                })

                # position spatiale (en m)
                pos = {
                    "x": d.spatialCoordinates.x / 1000,
                    "y": d.spatialCoordinates.z / 1000
                }
                publisher_position(pos)
                detections_pose.poses.append(
                    Pose(Point(pos["x"], pos["y"], 0),
                         Quaternion(0,0,0,1))
                )

                if visualize and in_frame:
                    visualize_detection(frame, d, (x1,y1,x2,y2))

            pub_poses.publish(detections_pose)
            publisher_bbox(bbox_list)

            if visualize and in_frame:
                cv2.putText(frame, f"FPS: {int(rate.sleep_dur.to_sec()**-1)}",
                            (10, h-10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255,255,255), 1)
                cv2.imshow("Oak Detection", frame)
                if cv2.waitKey(1) == ord('q'):
                    break

            rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: oak_detector_complete.py <model.blob> <config.json> [visualize]")
        sys.exit(1)

    blob_file = sys.argv[1]
    json_file = sys.argv[2]
    visualize = (len(sys.argv) == 4 and sys.argv[3] == "visualize")

    rospy.init_node("oak_detector_complete", anonymous=True)
    pub_bbox  = rospy.Publisher("oak",        Detection2DArray, queue_size=1)
    pub_frame = rospy.Publisher("oak_frames", Image,            queue_size=1)
    pub_poses = rospy.Publisher("cam_poses",  PoseArray,         queue_size=1)

    start_oak_camera(blob_file, json_file, visualize)
