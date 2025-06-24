#!/usr/bin/env python3
import sys
import math
import json
import time
import depthai as dai
import rospy
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header

# ----- CONFIGURATION ----- #
THRESHOLD = 1.0  # en mètres

# Charge les paramètres du JSON généré par DepthAI
def load_config(fn):
    d = json.load(open(fn))
    m = d["nn_config"]["NN_specific_metadata"]
    w,h = map(int, d["nn_config"]["input_size"].split("x"))
    return {
        "confidence_threshold": m.get("confidence_threshold", 0.5),
        "num_classes": m["classes"],
        "anchors": d["nn_config"]["NN_specific_metadata"]["anchors"],
        "anchor_masks": d["nn_config"]["NN_specific_metadata"]["anchor_masks"],
        "coordinate_size": d["nn_config"]["NN_specific_metadata"]["coordinates"],
        "iou_threshold": m["iou_threshold"],
        "input_w": w,
        "input_h": h
    }

# Construit le pipeline DepthAI
def build_pipeline(cfg, blob_fn):
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setPreviewSize(cfg["input_w"], cfg["input_h"])
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    cam.setPreviewKeepAspectRatio(False)

    monoL = p.create(dai.node.MonoCamera)
    monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoR = p.create(dai.node.MonoCamera)
    monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

    nn = p.create(dai.node.YoloSpatialDetectionNetwork)
    nn.setBlobPath(blob_fn)
    nn.setConfidenceThreshold(cfg["confidence_threshold"])
    nn.setNumClasses(cfg["num_classes"])
    nn.setCoordinateSize(cfg["coordinate_size"])
    nn.setAnchors(cfg["anchors"])
    nn.setAnchorMasks(cfg["anchor_masks"])
    nn.setIouThreshold(cfg["iou_threshold"])
    nn.input.setBlocking(False)
    nn.setBoundingBoxScaleFactor(0.5)
    nn.setDepthLowerThreshold(100)
    nn.setDepthUpperThreshold(10000)

    cam.preview.link(nn.input)
    stereo.depth.link(nn.inputDepth)
    monoL.out.link(stereo.left)
    monoR.out.link(stereo.right)

    xout = p.create(dai.node.XLinkOut)
    xout.setStreamName("detections")
    nn.out.link(xout.input)

    return p

class SwarmMapSharer:
    def __init__(self, blob, cfg_file):
        # ROS init
        rospy.init_node("map_sharer", anonymous=True)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.map_pub = rospy.Publisher("/shared_map", OccupancyGrid, queue_size=1)
        self.last_map = None

        # DepthAI init
        cfg = load_config(cfg_file)
        pipeline = build_pipeline(cfg, blob)
        self.device = dai.Device(pipeline)
        self.q = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)

    def map_cb(self, msg):
        self.last_map = msg

    def spin(self):
        rate = rospy.Rate(20)
        rospy.loginfo("[map_sharer] Threshold = %.2f m", THRESHOLD)
        while not rospy.is_shutdown():
            dets = self.q.get().detections
            for d in dets:
                x = d.spatialCoordinates.x / 1000.0
                z = d.spatialCoordinates.z / 1000.0
                dist = math.hypot(x, z)
                if dist < THRESHOLD and self.last_map:
                    rospy.loginfo("[map_sharer] Partage de la carte (objet à %.2f m)", dist)
                    to_send = OccupancyGrid()
                    to_send.header = Header(stamp=rospy.Time.now(), frame_id=self.last_map.header.frame_id)
                    to_send.info = self.last_map.info
                    to_send.data = self.last_map.data
                    self.map_pub.publish(to_send)
                    break
            rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: map_sharer.py <model.blob> <config.json>")
        sys.exit(1)
    blob, cfg = sys.argv[1], sys.argv[2]
    sharer = SwarmMapSharer(blob, cfg)
    sharer.spin()
