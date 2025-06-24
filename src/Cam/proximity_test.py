#!/usr/bin/env python3
import sys, json, math, time, depthai as dai

def load_config(fn):
    d = json.load(open(fn))
    meta = d["nn_config"]["NN_specific_metadata"]
    w, h = map(int, d["nn_config"]["input_size"].split("x"))
    return {
        "thr": float(meta.get("confidence_threshold", 0.5)),
        "cls": meta["classes"],
        "anc": meta["anchors"],
        "mask": meta["anchor_masks"],
        "coo": meta["coordinates"],
        "iou": meta["iou_threshold"],
        "w": w, "h": h
    }

def build_pipeline(c, blob):
    p = dai.Pipeline()
    cam = p.createColorCamera(); cam.setPreviewSize(c["w"], c["h"]); cam.setInterleaved(False)
    monoL = p.createMonoCamera(); monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B); monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoR = p.createMonoCamera(); monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C); monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    stereo = p.createStereoDepth(); stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    nn = p.createYoloSpatialDetectionNetwork()
    nn.setBlobPath(blob); nn.setConfidenceThreshold(c["thr"]); nn.setNumClasses(c["cls"])
    nn.setCoordinateSize(c["coo"]); nn.setAnchors(c["anc"]); nn.setAnchorMasks(c["mask"]); nn.setIouThreshold(c["iou"])
    cam.preview.link(nn.input); stereo.depth.link(nn.inputDepth)
    monoL.out.link(stereo.left); monoR.out.link(stereo.right)
    xout = p.createXLinkOut(); xout.setStreamName("det"); nn.out.link(xout.input)
    return p

def main(blob, cfg, threshold):
    cfg = load_config(cfg)
    try:
        pipeline = build_pipeline(cfg, blob)
        device = dai.Device(pipeline)
    except RuntimeError:
        print("✗ Aucun périphérique DepthAI détecté. Vérifiez que l’OAK est bien branchée.")
        sys.exit(1)

    q = device.getOutputQueue(name="det", maxSize=4, blocking=False)
    print(f"[INFO] Seuil de proximité : {threshold:.2f} m\n")
    try:
        while True:
            dets = q.get().detections
            for d in dets:
                x = d.spatialCoordinates.x/1000.0
                y = d.spatialCoordinates.z/1000.0
                dist = math.hypot(x, y)
                if dist < threshold:
                    print(f"[ALERT] Objet détecté à {dist:.2f} m (<{threshold:.2f} m)")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nArrêt utilisateur, sortie.")

if __name__ == "__main__":
    if len(sys.argv) not in (3,4):
        print("Usage: python3 proxi_test.py <modèle.blob> <config.json> [seuil_m]")
        sys.exit(1)
    blob, cfg = sys.argv[1], sys.argv[2]
    thr = float(sys.argv[3]) if len(sys.argv)==4 else 1.0
    main(blob, cfg, thr)
