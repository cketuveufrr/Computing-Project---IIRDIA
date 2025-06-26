#!/usr/bin/env python3
import sys, json, time
import rospy
import depthai as dai
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D

def publisher_bbox(header, bbox_data_list):
    msg = Detection2DArray()
    msg.header = header
    for i in bbox_data_list:
        det2d = Detection2D()
        det2d.header = header
        bb = BoundingBox2D()
        bb.center.x = i["x"]
        bb.center.y = i["y"]
        bb.size_x   = i["width"]
        bb.size_y   = i["height"]
        det2d.bbox = bb
        msg.detections.append(det2d)
    pub_bbox.publish(msg)

def start_oak_camera(blob_fn, json_fn):
    # Charge la config
    cfg = json.load(open(json_fn))
    meta = cfg["nn_config"]["NN_specific_metadata"]
    w, h = map(int, cfg["nn_config"]["input_size"].split("x"))
    conf_thresh = meta["confidence_threshold"]
    num_classes = meta["classes"]
    anchors     = meta["anchors"]
    masks       = meta["anchor_masks"]
    coord_size  = meta["coordinates"]
    iou_thresh  = meta["iou_threshold"]

    # Pipeline DepthAI
    pipeline = dai.Pipeline()
    cam      = pipeline.createColorCamera()
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setPreviewSize(w, h)
    cam.setInterleaved(False)

    yolo = pipeline.create(dai.node.YoloDetectionNetwork)
    yolo.setBlobPath(blob_fn)
    yolo.setConfidenceThreshold(conf_thresh)
    yolo.setNumClasses(num_classes)
    yolo.setCoordinateSize(coord_size)
    yolo.setAnchors(anchors)
    yolo.setAnchorMasks(masks)
    yolo.setIouThreshold(iou_thresh)
    cam.preview.link(yolo.input)

    xout = pipeline.createXLinkOut()
    xout.setStreamName("det")
    yolo.out.link(xout.input)

    device = dai.Device(pipeline)
    q_det  = device.getOutputQueue("det", 4, blocking=False)
    rate   = rospy.Rate(20)

    while not rospy.is_shutdown():
        header = Header(stamp=rospy.Time.now())
        in_det = q_det.tryGet()
        dets   = in_det.detections if in_det else []

        # Prépare la liste de bbox en pixels
        bbox_list = []
        for d in dets:
            cx = (d.xmin + d.xmax)/2 * w
            cy = (d.ymin + d.ymax)/2 * h
            sx = (d.xmax - d.xmin) * w
            sy = (d.ymax - d.ymin) * h
            bbox_list.append({"x": cx, "y": cy, "width": sx, "height": sy})

        publisher_bbox(header, bbox_list)
        rate.sleep()

if __name__ == "__main__":
    if len(sys.argv)!=3:
        print("Usage: oak_ros_test_noimg.py <model.blob> <config.json>")
        sys.exit(1)

    rospy.init_node("oak_detector_noimg", anonymous=True)
    pub_bbox = rospy.Publisher("oak", Detection2DArray, queue_size=1)
    start_oak_camera(sys.argv[1], sys.argv[2])
