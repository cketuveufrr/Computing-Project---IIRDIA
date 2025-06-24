#!/usr/bin/env python3

""" Alerte de proximité avec un modèle YOLO spatial sur DepthAI. """
"""Attention ! C'est pas le script qui envoie les cartes"""
"""Attention ! Mettre le threshold à 0.8 sinon overfitting"""

import sys
import json
import math
import time
import depthai as dai

def load_config(json_fn):
    with open(json_fn) as f:
        d = json.load(f)
    meta = d["nn_config"]["NN_specific_metadata"]
    input_w, input_h = map(int, d["nn_config"]["input_size"].split("x"))
    cfg = {
        "confidence_threshold": float(meta.get("confidence_threshold", 0.5)),
        "num_classes": d["nn_config"]["NN_specific_metadata"]["classes"],
        "anchors": d["nn_config"]["NN_specific_metadata"]["anchors"],
        "anchor_masks": d["nn_config"]["NN_specific_metadata"]["anchor_masks"],
        "coordinate_size": d["nn_config"]["NN_specific_metadata"]["coordinates"],
        "iou_threshold": d["nn_config"]["NN_specific_metadata"]["iou_threshold"],
        "input_w": input_w,
        "input_h": input_h,
        # On récupère la liste des étiquettes, si présente dans le JSON
        "labels": d["nn_config"].get("labels", [])
    }
    return cfg


def build_pipeline(cfg, blob_path):
    p = dai.Pipeline()

    # --- caméra couleur (RGB) ---
    cam = p.create(dai.node.ColorCamera)
    cam.setPreviewSize(cfg["input_w"], cfg["input_h"])
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    # si tu veux toute la FOV, conserve l'aspect non-centré :
    cam.setPreviewKeepAspectRatio(False)

    # --- monocaméras pour la profondeur ---
    monoL = p.create(dai.node.MonoCamera)
    monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoR = p.create(dai.node.MonoCamera)
    monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    # --- calcul de la carte de profondeur stéréo ---
    stereo = p.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # *** très important ***
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

    # --- réseau YOLO spatial --- => vient du json de configuration
    nn = p.create(dai.node.YoloSpatialDetectionNetwork)
    nn.setBlobPath(blob_path)
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

    # --- connections ---
    cam.preview.link(nn.input)         
    stereo.depth.link(nn.inputDepth)   
    monoL.out.link(stereo.left) 
    monoR.out.link(stereo.right)

    # sortie des détections
    xout = p.create(dai.node.XLinkOut)
    xout.setStreamName("detections")
    nn.out.link(xout.input)

    return p

def main(blob_path, config_path, threshold):
    cfg = load_config(config_path)
    pipeline = build_pipeline(cfg, blob_path)
    with dai.Device(pipeline) as dev:
        q = dev.getOutputQueue(name="detections", maxSize=4, blocking=False)
        print(f"[INFO] Proximity threshold: {threshold:.2f} m\n")
        while True:
            dets = q.get().detections
            for d in dets:
                # Récupération du nom de la classe
                # Si labels non défini, on retombe sur l'ID
                label_name = cfg["labels"][d.label] if cfg["labels"] else str(d.label)

                # Calcul de la distance (en m)
                x_m = d.spatialCoordinates.x / 1000.0
                y_m = d.spatialCoordinates.z / 1000.0
                dist = math.hypot(x_m, y_m)

                # Log du seul label et de la confiance (sans alerte de proximité)
                print(f"[DETECTION] {label_name} (ID : {d.label}, conf. : {d.confidence:.2f})")
            time.sleep(0.01)

if __name__ == "__main__":
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print("Usage: python3 proxi_test.py <model.blob> <config.json> [threshold_m]")
        sys.exit(1)
    blob_file = sys.argv[1]
    json_file = sys.argv[2]
    thresh = float(sys.argv[3]) if len(sys.argv) == 4 else 1.0
    main(blob_file, json_file, thresh)
