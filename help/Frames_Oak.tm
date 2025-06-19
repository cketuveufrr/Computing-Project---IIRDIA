<TeXmacs|2.1.4>

<style|<tuple|generic|french>>

<\body>
  Voici un tutoriel exhaustif, issu de notre échange, que vous pourrez
  transmettre tel quel :

  <hrule>

  <chapter*|Détection de Sphero RVR avec Oak-D et ROS Noetic sur deux
  Raspberry Pi>

  <section*|1. Présentation du projet>

  Ce tutoriel détaille comment configurer deux Raspberry Pi :

  <\itemize>
    <item>Un <strong|Master> qui fait tourner <code*|roscore> et recueille
    les données

    <item>Une <strong|Camera Pi> équipée d'une caméra Oak-D (Movidius
    MyriadX) pour la détection
  </itemize>

  On y couvre l'installation des dépendances, la configuration réseau ROS,
  les tests de la caméra, la mise au point d'un pipeline DepthAI/YOLOv5,
  l'intégration ROS, et le débogage des problèmes courants (under-voltage,
  cv_bridge, dimensions de réseau, etc.).

  <hrule>

  <section*|2. Matériel et architecture réseau>

  <\itemize>
    <item><strong|Raspberry Pi Master> (ROS Noetic) : IP
    <code*|192.168.1.102>

    <item><strong|Raspberry Pi Caméra> (Oak-D + DepthAI) : IP
    <code*|192.168.1.142>

    <item>Câble USB 3.0 court pour l'Oak-D, alimentation officielle 5 V
    \<geq\> 3 A
  </itemize>

  <hrule>

  <section*|3. Installation des dépendances>

  <subsection*|3.1. Mise à jour système et paquets communs>

  Sur <strong|chaque> Pi :

  <code|<\code*>
    sudo apt update

    sudo apt install -y python3-pip python3-opencv libopencv-dev \\

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ ros-noetic-vision-msgs

    \;
  </code*>>

  <subsection*|3.2. Installer ROS Noetic (si pas déjà fait)>

  Suivre la doc officielle : <hlink|http://wiki.ros.org/noetic/Installation/Ubuntu|http://wiki.ros.org/noetic/Installation/Ubuntu>

  <subsection*|3.3. Installer DepthAI et SDK Python>

  Sur la <strong|Camera Pi> :

  <code|<\code*>
    # dépendances système DepthAI

    sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh \| bash

    \;

    # SDK Python

    python3 -m pip install --upgrade depthai depthai-sdk

    \;
  </code*>>

  <subsection*|3.4. Règles udev pour la Oak-D>

  Sur la <strong|Camera Pi> :

  <code|<\code*>
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \\

    \ \ \| sudo tee /etc/udev/rules.d/80-movidius.rules

    \;

    sudo udevadm control --reload-rules && sudo udevadm trigger

    # (rebrancher l'Oak-D)

    \;
  </code*>>

  <hrule>

  <section*|4. Configuration ROS réseau>

  <subsection*|4.1. Master (192.168.1.102)>

  Ajoutez à <code*|~/.bashrc> :

  <code|<\code*>
    export ROS_MASTER_URI=http://192.168.1.102:11311

    export ROS_IP=192.168.1.102

    \;
  </code*>>

  Puis <code*|source ~/.bashrc>.

  <subsection*|4.2. Camera Pi (192.168.1.142)>

  Ajoutez à son <code*|~/.bashrc> :

  <code|<\code*>
    export ROS_MASTER_URI=http://192.168.1.102:11311

    export ROS_IP=192.168.1.142

    \;
  </code*>>

  Puis <code*|source ~/.bashrc>.

  <hrule>

  <section*|5. Vérifications de base>

  <\enumerate>
    <item><strong|Ping> entre les Pi :

    <code|<\code*>
      ping -c3 192.168.1.102 \ \ # depuis .142

      ping -c3 192.168.1.142 \ \ # depuis .102

      \;
    </code*>>

    <item><strong|roscore> (Master) :

    <code|<\code*>
      roscore

      \;
    </code*>>

    <item><strong|rosnode list> (Camera Pi) :

    <code|<\code*>
      rosnode list

      \;
    </code*>>

    <item><strong|Détection USB Oak-D> (Camera Pi) :

    <code|<\code*>
      lsusb \| grep 03e7

      dmesg \| tail -n20

      \;
    </code*>>
  </enumerate>

  <hrule>

  <section*|6. Test minimal de la caméra Oak-D>

  Créez <code*|test_oak_minimal.py> :

  <code|<\code*>
    #!/usr/bin/env python3

    import depthai as dai, cv2, os

    \;

    pipeline = dai.Pipeline()

    cam = pipeline.createColorCamera()

    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    xout = pipeline.createXLinkOut(); xout.setStreamName("rgb")

    cam.video.link(xout.input)

    \;

    with dai.Device(pipeline) as dev:

    \ \ \ \ q = dev.getOutputQueue("rgb", maxSize=1, blocking=True)

    \ \ \ \ frame = q.get().getCvFrame()

    \ \ \ \ cv2.imwrite("oak_test.jpg", frame)

    \ \ \ \ print("Image capturée \<rightarrow\> oak_test.jpg")

    \;
  </code*>>

  <code|<\code*>
    chmod +x test_oak_minimal.py

    python3 -m pip install opencv-python-headless

    ./test_oak_minimal.py

    ls -lh oak_test.jpg

    \;
  </code*>>

  <hrule>

  <section*|7. Test du modèle 2D sans ROS>

  <subsection*|7.1. Adapter le JSON>

  Assurez-vous que dans <code*|rvr_stretched_1.json> vous avez :

  <code|<\code*>
    "input_size": ``256x256",

    "NN_specific_metadata": {

    \ \ ``confidence_threshold": 0.3, \ \ // ou 0.1\U0.2 pour test

    \ \ ``classes": 1,

    \ \ ``anchors": [\<ldots\>],

    \ \ ``anchor_masks": {

    \ \ \ \ ``side32": [0,1,2],

    \ \ \ \ ``side16": [3,4,5],

    \ \ \ \ ``side8": \ [6,7,8]

    \ \ },

    \ \ ``coordinates": 4,

    \ \ ``iou_threshold": 0.5

    },

    \;
  </code*>>

  <subsection*|7.2. Script de test 2D (<code*|debug_2d.py>)>

  <code|<\code*>
    #!/usr/bin/env python3

    import depthai as dai, json, sys, time

    \;

    if len(sys.argv)!=3:

    \ \ \ \ print("Usage: debug_2d.py model.blob config.json")

    \ \ \ \ sys.exit(1)

    blob, cfg = sys.argv[1], sys.argv[2]

    j = json.load(open(cfg))

    m = j["nn_config"]["NN_specific_metadata"]

    w,h = map(int, j["nn_config"]["input_size"].split("x"))

    conf = m["confidence_threshold"]

    \;

    pipeline = dai.Pipeline()

    cam = pipeline.createColorCamera()

    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    cam.setPreviewSize(w,h); cam.setInterleaved(False)

    \;

    yolo = pipeline.create(dai.node.YoloDetectionNetwork)

    yolo.setBlobPath(blob)

    yolo.setConfidenceThreshold(conf)

    yolo.setNumClasses(m["classes"])

    yolo.setCoordinateSize(m["coordinates"])

    yolo.setAnchors(m["anchors"])

    yolo.setAnchorMasks(m["anchor_masks"])

    yolo.setIouThreshold(m["iou_threshold"])

    cam.preview.link(yolo.input)

    \;

    xout = pipeline.createXLinkOut(); xout.setStreamName("det")

    yolo.out.link(xout.input)

    \;

    with dai.Device(pipeline) as dev:

    \ \ \ \ q = dev.getOutputQueue("det", 4, True)

    \ \ \ \ print("Montrez un RVR\<ldots\>")

    \ \ \ \ while True:

    \ \ \ \ \ \ \ \ dets = q.get().detections

    \ \ \ \ \ \ \ \ print(f"{len(dets)} détection(s) \V
    {[round(d.confidence,2) for d in dets]}")

    \ \ \ \ \ \ \ \ time.sleep(0.5)

    \;
  </code*>>

  <code|<\code*>
    chmod +x debug_2d.py

    ./debug_2d.py models/rvr_stretched_1.blob models/rvr_stretched_1.json

    \;
  </code*>>

  Baissez <code*|conf> si nécessaire (jusqu'à 0.1 ou 0.0).

  <hrule>

  <section*|8. Script ROS complet (<code*|oak_detector_complete.py>)>

  Le pipeline final :

  <code|<\code*>
    #!/usr/bin/env python3

    import sys, json, time, rospy, depthai as dai, cv2, tf

    from std_msgs.msg \ \ \ \ \ import Header

    from sensor_msgs.msg \ \ import Image

    from vision_msgs.msg \ \ import Detection2DArray, Detection2D,
    BoundingBox2D

    from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

    from cv_bridge import CvBridge

    \;

    # --- Publishers globaux (initialisés dans __main__)

    pub_bbox, pub_frame, pub_poses = None, None, None

    \;

    def publisher_bbox(bbox_data_list):

    \ \ \ \ msg = Detection2DArray(); msg.header = Header(rospy.Time.now())

    \ \ \ \ for bb in bbox_data_list:

    \ \ \ \ \ \ \ \ d2 = Detection2D(header=msg.header)

    \ \ \ \ \ \ \ \ box = BoundingBox2D(center=Point(bb["x"],bb["y"],0),

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ size_x=bb["width"],
    size_y=bb["height"])

    \ \ \ \ \ \ \ \ d2.bbox = box

    \ \ \ \ \ \ \ \ msg.detections.append(d2)

    \ \ \ \ pub_bbox.publish(msg)

    \;

    def publisher_position(pos):

    \ \ \ \ br = tf.TransformBroadcaster()

    \ \ \ \ br.sendTransform((pos["x"],pos["y"],0),(0,0,0,1),

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ rospy.Time.now(),"Mercator_1","odom")

    \;

    def publisher_images_post_proc(frame):

    \ \ \ \ bridge = CvBridge()

    \ \ \ \ img = bridge.cv2_to_imgmsg(frame, ``bgr8")

    \ \ \ \ img.header = Header(rospy.Time.now())

    \ \ \ \ pub_frame.publish(img)

    \;

    def get_config(json_path):

    \ \ \ \ j = json.load(open(json_path))

    \ \ \ \ m = j["nn_config"]["NN_specific_metadata"]

    \ \ \ \ w,h = map(int, j["nn_config"]["input_size"].split("x"))

    \ \ \ \ return {**m, ``inputW":w, ``inputH":h}

    \;

    def visualize_detection(frame,d,xyxy):

    \ \ \ \ x1,y1,x2,y2 = xyxy

    \ \ \ \ cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),2)

    \ \ \ \ cv2.putText(frame,f"{int(d.confidence*100)}%",(x1,y1-5),

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)

    \;

    def start_oak_camera(blob_fn,json_fn,visualize=False,IR=False):

    \ \ \ \ cfg = get_config(json_fn)

    \ \ \ \ pipeline = dai.Pipeline()

    \ \ \ \ cam = pipeline.create(dai.node.ColorCamera)

    \ \ \ \ cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    \ \ \ \ cam.setPreviewSize(cfg["inputW"],cfg["inputH"]);
    cam.setInterleaved(False)

    \;

    \ \ \ \ yolo = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

    \ \ \ \ yolo.setBlobPath(blob_fn)

    \ \ \ \ yolo.setConfidenceThreshold(cfg["confidence_threshold"])

    \ \ \ \ yolo.setNumClasses(cfg["classes"])

    \ \ \ \ yolo.setCoordinateSize(cfg["coordinates"])

    \ \ \ \ yolo.setAnchors(cfg["anchors"])

    \ \ \ \ yolo.setAnchorMasks(cfg["anchor_masks"])

    \ \ \ \ yolo.setIouThreshold(cfg["iou_threshold"])

    \ \ \ \ cam.preview.link(yolo.input)

    \;

    \ \ \ \ xout_det = pipeline.create(dai.node.XLinkOut);
    xout_det.setStreamName("det")

    \ \ \ \ yolo.out.link(xout_det.input)

    \ \ \ \ xout_rgb = pipeline.create(dai.node.XLinkOut);
    xout_rgb.setStreamName("rgb")

    \ \ \ \ yolo.passthrough.link(xout_rgb.input)

    \;

    \ \ \ \ with dai.Device(pipeline) as dev:

    \ \ \ \ \ \ \ \ if IR: dev.setIrLaserDotProjectorBrightness(800)

    \ \ \ \ \ \ \ \ q_det \ \ = dev.getOutputQueue("det",4,False)

    \ \ \ \ \ \ \ \ q_frame = dev.getOutputQueue("rgb",4,False)

    \ \ \ \ \ \ \ \ rate \ \ \ = rospy.Rate(20)

    \;

    \ \ \ \ \ \ \ \ while not rospy.is_shutdown():

    \ \ \ \ \ \ \ \ \ \ \ \ header = Header(rospy.Time.now())

    \ \ \ \ \ \ \ \ \ \ \ \ # 1) Frame \<rightarrow\> /oak_frames (toujours)

    \ \ \ \ \ \ \ \ \ \ \ \ in_fr = q_frame.tryGet()

    \ \ \ \ \ \ \ \ \ \ \ \ frame = in_fr.getCvFrame() if in_fr else None

    \ \ \ \ \ \ \ \ \ \ \ \ if frame is not None:

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ publisher_images_post_proc(frame)

    \;

    \ \ \ \ \ \ \ \ \ \ \ \ # 2) Détections \<rightarrow\> /oak + /cam_poses

    \ \ \ \ \ \ \ \ \ \ \ \ in_dt = q_det.tryGet(); dets = in_dt.detections
    if in_dt else []

    \ \ \ \ \ \ \ \ \ \ \ \ bbox_list, poses = [], PoseArray(header=header)

    \ \ \ \ \ \ \ \ \ \ \ \ for d in dets:

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ x1=int(d.xmin*cfg["inputW"]);
    x2=int(d.xmax*cfg["inputW"])

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ y1=int(d.ymin*cfg["inputH"]);
    y2=int(d.ymax*cfg["inputH"])

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ cx,cy=(x1+x2)/2,(y1+y2)/2;
    sx,sy=x2-x1,y2-y1

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ bbox_list.append({"x":cx,"y":cy,"width":sx,"height":sy})

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ pos={"x":d.spatialCoordinates.x/1000,

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ ``y":d.spatialCoordinates.z/1000}

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ publisher_position(pos)

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ poses.poses.append(Pose(Point(pos["x"],pos["y"],0),

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ Quaternion(0,0,0,1)))

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ if visualize and frame is not None:

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ visualize_detection(frame,d,(x1,y1,x2,y2))

    \;

    \ \ \ \ \ \ \ \ \ \ \ \ pub_poses.publish(poses)

    \ \ \ \ \ \ \ \ \ \ \ \ publisher_bbox(bbox_list)

    \;

    \ \ \ \ \ \ \ \ \ \ \ \ # 3) Affichage local si demandé

    \ \ \ \ \ \ \ \ \ \ \ \ if visualize and frame is not None:

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ cv2.imshow("Oak", frame)

    \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ \ if cv2.waitKey(1)==ord('q'): break

    \ \ \ \ \ \ \ \ \ \ \ \ rate.sleep()

    \;

    if __name__=="__main__":

    \ \ \ \ if len(sys.argv)\<less\>3:

    \ \ \ \ \ \ \ \ print("Usage: oak_detector_complete.py
    \<less\>blob\<gtr\> \<less\>json\<gtr\> [visualize]")

    \ \ \ \ \ \ \ \ sys.exit(1)

    \ \ \ \ rospy.init_node("oak_detector_complete",anonymous=True)

    \ \ \ \ pub_bbox \ = rospy.Publisher("oak",
    \ \ \ \ \ \ \ Detection2DArray, queue_size=1)

    \ \ \ \ pub_frame = rospy.Publisher("oak_frames", Image,
    \ \ \ \ \ \ \ \ \ \ \ queue_size=1)

    \ \ \ \ pub_poses = rospy.Publisher("cam_poses", \ PoseArray,
    \ \ \ \ \ \ \ \ queue_size=1)

    \ \ \ \ bf, jf = sys.argv[1], sys.argv[2]

    \ \ \ \ viz \ \ = (len(sys.argv)==4 and sys.argv[3]=="visualize")

    \ \ \ \ start_oak_camera(bf,jf,viz)

    \;
  </code*>>

  <hrule>

  <section*|9. Visualisation et dépannages>

  <\itemize>
    <item><strong|Afficher uniquement <code*|detections>> :

    <code|<\code*>
      rostopic echo /oak/detections

      \;
    </code*>>

    <item><strong|Afficher <code*|/oak> filtré> (vide ou non) :

    <code|<\code*>
      rostopic echo /oak --filter 'len(msg.detections)\<gtr\>0'

      \;
    </code*>>

    <item><strong|Flux image> (avec GUI) :

    <code|<\code*>
      rosrun rqt_image_view rqt_image_view /oak_frames

      \;
    </code*>>

    <item><strong|RViz> :

    <\itemize>
      <item>Display \<rightarrow\> <code*|TF> pour voir vos frames

      <item>Display \<rightarrow\> <code*|Detection2DArray> \<rightarrow\>
      topic <code*|/oak>
    </itemize>
  </itemize>

  <hrule>

  <section*|10. Résolution des problèmes courants>

  <\itemize>
    <item><strong|Under-voltage> (coupe USB) : alimentation officielle, câble
    court, hub alimenté.

    <item><strong|Segfault DepthAI> : dimensions et anchor masks doivent
    correspondre au JSON.

    <item><strong|cv_bridge> :

    <\itemize>
      <item>Installer <code*|python3-cv-bridge>/<code*|libcv-bridge1d> via
      apt

      <item>Sourcer <code*|/opt/ros/noetic/setup.bash>

      <item>En dernière option, recompiler <code*|cv_bridge> dans un
      workspace Catkin.
    </itemize>

    <item><strong|Pas de frames> dans ROS \<rightarrow\> vérifiez que
    <code*|publisher_images_post_proc(frame)> est <strong|toujours> appelé
    (ou conditionné sur <code*|dets> comme souhaité).
  </itemize>

  <hrule>

  Vous disposez désormais d'un guide complet pour déployer et déboguer la
  détection de Sphero RVR avec Oak-D et ROS Noetic sur deux Raspberry Pi.
</body>

<\initial>
  <\collection>
    <associate|page-medium|paper>
  </collection>
</initial>

<\references>
  <\collection>
    <associate|auto-1|<tuple|?|1>>
    <associate|auto-10|<tuple|<with|mode|<quote|math>|\<bullet\>>|2>>
    <associate|auto-11|<tuple|<with|mode|<quote|math>|\<bullet\>>|2>>
    <associate|auto-12|<tuple|<with|mode|<quote|math>|\<bullet\>>|2>>
    <associate|auto-13|<tuple|4|3>>
    <associate|auto-14|<tuple|4|3>>
    <associate|auto-15|<tuple|4|3>>
    <associate|auto-16|<tuple|4|4>>
    <associate|auto-17|<tuple|4|4>>
    <associate|auto-18|<tuple|4|7>>
    <associate|auto-19|<tuple|<with|mode|<quote|math>|<rigid|\<circ\>>>|7>>
    <associate|auto-2|<tuple|?|1>>
    <associate|auto-3|<tuple|<with|mode|<quote|math>|\<bullet\>>|1>>
    <associate|auto-4|<tuple|<with|mode|<quote|math>|\<bullet\>>|1>>
    <associate|auto-5|<tuple|<with|mode|<quote|math>|\<bullet\>>|1>>
    <associate|auto-6|<tuple|<with|mode|<quote|math>|\<bullet\>>|1>>
    <associate|auto-7|<tuple|<with|mode|<quote|math>|\<bullet\>>|1>>
    <associate|auto-8|<tuple|<with|mode|<quote|math>|\<bullet\>>|2>>
    <associate|auto-9|<tuple|<with|mode|<quote|math>|\<bullet\>>|2>>
  </collection>
</references>

<\auxiliary>
  <\collection>
    <\associate|toc>
      <vspace*|2fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|font-size|<quote|1.19>|Détection
      de Sphero RVR avec Oak-D et ROS Noetic sur deux Raspberry Pi>
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-1><vspace|1fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|1.
      Présentation du projet> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-2><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|2.
      Matériel et architecture réseau> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-3><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|3.
      Installation des dépendances> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-4><vspace|0.5fn>

      <with|par-left|<quote|1tab>|3.1. Mise à jour système et paquets communs
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-5>>

      <with|par-left|<quote|1tab>|3.2. Installer ROS Noetic (si pas déjà
      fait) <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-6>>

      <with|par-left|<quote|1tab>|3.3. Installer DepthAI et SDK Python
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-7>>

      <with|par-left|<quote|1tab>|3.4. Règles udev pour la Oak-D
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-8>>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|4.
      Configuration ROS réseau> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-9><vspace|0.5fn>

      <with|par-left|<quote|1tab>|4.1. Master (192.168.1.102)
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-10>>

      <with|par-left|<quote|1tab>|4.2. Camera Pi (192.168.1.142)
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-11>>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|5.
      Vérifications de base> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-12><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|6.
      Test minimal de la caméra Oak-D> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-13><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|7.
      Test du modèle 2D sans ROS> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-14><vspace|0.5fn>

      <with|par-left|<quote|1tab>|7.1. Adapter le JSON
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-15>>

      <with|par-left|<quote|1tab>|7.2. Script de test 2D
      (<with|font-family|<quote|tt>|debug_2d.py>)
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-16>>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|8.
      Script ROS complet (<with|font-family|<quote|tt>|oak_detector_complete.py>)>
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-17><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|9.
      Visualisation et dépannages> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-18><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|10.
      Résolution des problèmes courants> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-19><vspace|0.5fn>
    </associate>
  </collection>
</auxiliary>