#!/usr/bin/env python3
import depthai as dai, json, sys, time

if len(sys.argv)!=3:
    print("Usage: debug_2d.py model.blob config.json")
    sys.exit(1)
blob, cfg = sys.argv[1], sys.argv[2]

# Lecture JSON
with open(cfg) as f: cfgj = json.load(f)
meta = cfgj["nn_config"]["NN_specific_metadata"]
w, h = map(int, cfgj["nn_config"]["input_size"].split("x"))
conf = 0.1  # seuil bas pour test

# Pipeline
pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam.setPreviewSize(w, h)
cam.setInterleaved(False)

# 2D network
yolo = pipeline.create(dai.node.YoloDetectionNetwork)
yolo.setBlobPath(blob)
yolo.setConfidenceThreshold(conf)
yolo.setNumClasses(meta["classes"])
yolo.setCoordinateSize(meta["coordinates"])
yolo.setAnchors(meta["anchors"])
yolo.setAnchorMasks(meta["anchor_masks"])
yolo.setIouThreshold(meta["iou_threshold"])
cam.preview.link(yolo.input)

# Sortie
xout = pipeline.createXLinkOut()
xout.setStreamName("det")
yolo.out.link(xout.input)
# Exécution
with dai.Device(pipeline) as dev:
    q = dev.getOutputQueue("det", 4, True)
    
    print("Démarrage 2D, montrez un RVR…")
    while True:
        dets = q.get().detections
        print(f"{len(dets)} détection(s): {[round(d.confidence,2) for d in dets]}")
        time.sleep(0.5)
