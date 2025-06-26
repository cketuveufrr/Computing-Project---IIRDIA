#!/usr/bin/env python3
import depthai as dai
import cv2, os

# Pipeline minimal
pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
xout = pipeline.createXLinkOut()
xout.setStreamName("rgb")
cam.video.link(xout.input)

# Exécution
with dai.Device(pipeline) as device:
    q = device.getOutputQueue("rgb", maxSize=1, blocking=True)
    frame = q.get().getCvFrame()
    fname = "oak_frame.jpg"
    cv2.imwrite(fname, frame)
    print("Image capturée :", os.path.abspath(fname))
