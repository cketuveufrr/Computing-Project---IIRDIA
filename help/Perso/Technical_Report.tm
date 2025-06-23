<TeXmacs|2.1.4>

<style|<tuple|generic|french>>

<\body>
  <maketitle>

  <surround||<new-page>|<\table-of-contents|toc>
    \;
  </table-of-contents>>

  <section|Goal of the Original Thesis>

  The thesis proposes a decentralized SLAM framework for swarm robotics,
  enabling a group of autonomous robots to collectively explore and map an
  environment without a central controller or global localization system. It
  leverages local sensing, Wi-Fi ad-hoc communication, and lightweight map
  sharing techniques (image-based map exchange and merging) to generate
  coherent maps across the swarm. The system is designed for resilience,
  scalability, and efficient multi-robot collaboration in unknown or
  GPS-denied environments.

  <section|Required Hardware and Tools>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Component>|<cell|Purpose>>|<row|<cell|<strong|Raspberry
  Pi (x2)>>|<cell|One for main ROS master, one for OAK-D camera & perception
  node>>|<row|<cell|<strong|OAK-D camera>>|<cell|Robot detection through RGB
  image analysis (red LED detection logic)>>|<row|<cell|<strong|YD LiDAR
  X4>>|<cell|Environment sensing for obstacle detection and SLAM (Gmapping
  input)>>|<row|<cell|<strong|Sphero RVR>>|<cell|Robot base with wheel
  encoders for odometry>>|<row|<cell|<strong|DC-DC Step Down
  Module>>|<cell|Powers the second Raspberry Pi or additional
  sensors>>|<row|<cell|<strong|IMU (internal to RVR)>>|<cell|For orientation
  correction and pose estimation>>|<row|<cell|<strong|Wi-Fi
  modules>>|<cell|Required for inter-robot communication via ROS
  Multi-Master>>>>>

  <section|Installation Guide>

  <with|font-shape|italic|(To be completed)>

  <\enumerate>
    <item>Install Ubuntu 20.04.

    <item>Install ROS Noetic: <slink|http://wiki.ros.org/noetic/Installation/Ubuntu>

    <item>Clone the repository and install dependencies:

    <\verbatim>
      \;

      \ \ \ \ git clone https://github.com/your_repo/ros_swarm_slam.git

      \ \ \ \ cd ros_swarm_slam

      \ \ \ \ rosdep update

      \ \ \ \ rosdep install --from-paths src --ignore-src -r -y

      \ \ \ \ catkin_make
    </verbatim>

    <item>Install Python libraries: <with|font-family|tt|pip3 install
    opencv-python scikit-image sewar>

    <item>Configure ROS Multi-Master (set
    <with|font-family|tt|ROS_MASTER_URI>, <with|font-family|tt|ROS_HOSTNAME>)

    <item>Prepare launch files under <with|font-family|tt|launch/>.
  </enumerate>

  <subsection|Launching Scripts>

  <todo|Add my own launching scripts to help>

  <subsection|Bashrc>

  For the master :\ 

  <verbatim|sudo ip link set eth0 up && sudo ip addr flush dev eth0 && sudo
  ip addr add 10.66.66.1/24 dev eth0 && export
  ROS_MASTER_URI=http://10.66.66.1:11311 && export ROS_IP=10.66.66.1>

  For the slave :\ 

  <verbatim|sudo ip link set eth0 up && sudo ip addr flush dev eth0 && sudo
  ip addr add 10.66.66.2/24 dev eth0 && export
  ROS_MASTER_URI=http://10.66.66.1:11311 && export ROS_IP=10.66.66.2>

  ! Do not forget to flush because it caused me trouble not to

  <section|Sensor and Module Configuration>

  <subsection|Launch>

  To remap the names for each RPI <todo|on doit mettre dans un seul node tout
  ça ou alors ça ne servirait à rien ?>

  <subsection|Drivers>

  Sync vs Async and problems corresponding

  <subsection|LiDAR>

  <\itemize>
    <item>Use <with|font-family|tt|ydlidar_ros_driver>, set
    <with|font-family|tt|frame_id> for Gmapping.

    <item>Limit max range to 0.4<space|0.17em>m
    (<with|font-family|tt|maxUrange>).

    <item>Common fixes: rotate by 90\<degree\> if front readings drop to
    zero.

    <item>Test: verify 360\<degree\> scan in RViz; check obstacle detection.
  </itemize>

  <subsection|Odometry>

  <\itemize>
    <item>Node: <with|font-family|tt|rvr_async_driver>.

    <item>Issues: node crash if launched too early; UART instability on old
    batteries.

    <item>Test: run <with|font-family|tt|rosrun tf view_frames>; drive in a
    square.
  </itemize>

  <subsection|Gmapping>

  <\itemize>
    <item>Requires accurate odometry; set
    <with|font-family|tt|maxUrange=0.4>.

    <item>Watch for map drift on startup.

    <item>Test: map a small room; verify loop closures.
  </itemize>

  <subsection|Ethernet Interface>

  <\itemize>
    <item>Direct Ethernet or static IP Wi-Fi.

    <item>Set <with|font-family|tt|ROS_MASTER_URI> and
    <with|font-family|tt|ROS_IP> on both Pis.

    <item>Synchronize clocks with NTP/Chrony.
  </itemize>

  <subsection|OAK-D Camera>

  <\itemize>
    <item>Use DepthAI pipeline; node: <with|font-family|tt|oak_camera_publisher>.

    <item>Detect red LEDs by pixel threshold (tune per light).

    <item>Test: view image topic; confirm
    <with|font-family|tt|red_led_detected>.
  </itemize>

  <subsection|Robot Detection Code>

  <\itemize>
    <item>Crop frame <math|\<to\>> count red pixels <math|\<to\>> threshold.

    <item>Tune threshold; beware false positives.

    <item>Test at varying distances/angles.
  </itemize>

  <subsection|Inter-Robot Communication>

  <\itemize>
    <item>ROS Multi-Master over Wi-Fi mesh.

    <item>Exchange: map images, exploration flags.

    <item>Issues: message loss if <math|\<gtr\>>1<space|0.17em>m; Wi-Fi
    instability.
  </itemize>

  <subsection|Map Fusion>

  <\itemize>
    <item>Python (<with|font-family|tt|cv2>, <with|font-family|tt|skimage>):
    reshape, blend, median filter.

    <item>Use RMSD threshold to reject noisy merges.

    <item>Test: compare merged map to ground truth.
  </itemize>

  <section|ROS Node Architecture>

  <tabular|<tformat|<twith|table-hmode|min>|<twith|table-width|1par>|<cwith|1|-1|1|-1|cell-hyphen|t>|<table|<row|<cell|Node
  name>|<cell|Role>|<cell|Topics used>>|<row|<cell|<code*|random_walk_X>>|<cell|Controls
  basic motion>|<cell|Publishes <code*|/odom_X>,
  <code*|/wheels_speed_X>>>|<row|<cell|<code*|slam_gmapping_X>>|<cell|Generates
  map>|<cell|Subscribes to <code*|/scan_X>,
  <code*|/odom_X>>>|<row|<cell|<code*|map_saver_X>>|<cell|Saves map
  data>|<cell|Uses Gmapping output>>|<row|<cell|<code*|image_publisher_X>>|<cell|Publishes
  map image>|<cell|Publishes <code*|/map_global_X>>>|<row|<cell|<code*|image_merger_X>>|<cell|Merges
  maps with others>|<cell|Subscribes to <code*|/common_map>>>|<row|<cell|<code*|oak_camera_publisher_X>>|<cell|Publishes
  image for red LED detection>|<cell|Publishes
  <code*|/red_led_detected_X>>>|<row|<cell|<code*|rvr_async_driver_X>>|<cell|Interface
  with base and IMU>|<cell|Publishes <code*|/odom_X>, <code*|/imu_X>>>>>>

  <section|TF and Frame Structure>

  Each robot publishes:

  <\itemize>
    <item><with|font-family|tt|odom_X <math|\<to\>> base_link_X>

    <item><with|font-family|tt|laser_X <math|\<to\>> base_link_X>

    <item><with|font-family|tt|camera_link_X <math|\<to\>> base_link_X>

    <item>Global map under <with|font-family|tt|/map>.
  </itemize>

  Use <with|font-family|tt|tf2_static_transform_publisher> for fixed offsets.

  <section|Multi-Robot Coordination and Map Fusion>

  <\itemize>
    <item>Image-based map exchange reduces bandwidth.

    <item>Publish only after red LED detection to enforce proximity.

    <item>RMSD filtering prevents overconfidence in bad data.

    <item>Advantages: low bandwidth, decentralized.

    <item>Limitations: needs overlap and orientation alignment.
  </itemize>

  \;
</body>

<\initial>
  <\collection>
    <associate|page-medium|paper>
  </collection>
</initial>

<\references>
  <\collection>
    <associate|auto-1|<tuple|1|2>>
    <associate|auto-10|<tuple|4.4|3>>
    <associate|auto-11|<tuple|4.5|3>>
    <associate|auto-12|<tuple|4.6|4>>
    <associate|auto-13|<tuple|4.7|4>>
    <associate|auto-14|<tuple|4.8|4>>
    <associate|auto-15|<tuple|4.9|4>>
    <associate|auto-16|<tuple|4.10|?>>
    <associate|auto-17|<tuple|5|?>>
    <associate|auto-18|<tuple|6|?>>
    <associate|auto-19|<tuple|7|?>>
    <associate|auto-2|<tuple|2|2>>
    <associate|auto-3|<tuple|3|2>>
    <associate|auto-4|<tuple|3.1|2>>
    <associate|auto-5|<tuple|3.2|2>>
    <associate|auto-6|<tuple|4|3>>
    <associate|auto-7|<tuple|4.1|3>>
    <associate|auto-8|<tuple|4.2|3>>
    <associate|auto-9|<tuple|4.3|3>>
    <associate|menur4r|<tuple|3.2|?>>
  </collection>
</references>

<\auxiliary>
  <\collection>
    <\associate|toc>
      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|1<space|2spc>Goal
      of the Original Thesis> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-1><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|2<space|2spc>Required
      Hardware and Tools> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-2><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|3<space|2spc>Installation
      Guide> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-3><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|4<space|2spc>Sensor
      and Module Configuration> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-4><vspace|0.5fn>

      <with|par-left|<quote|1tab>|4.1<space|2spc>LiDAR
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-5>>

      <with|par-left|<quote|1tab>|4.2<space|2spc>Odometry
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-6>>

      <with|par-left|<quote|1tab>|4.3<space|2spc>Gmapping
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-7>>

      <with|par-left|<quote|1tab>|4.4<space|2spc>Ethernet Interface
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-8>>

      <with|par-left|<quote|1tab>|4.5<space|2spc>OAK-D Camera
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-9>>

      <with|par-left|<quote|1tab>|4.6<space|2spc>Robot Detection Code
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-10>>

      <with|par-left|<quote|1tab>|4.7<space|2spc>Inter-Robot Communication
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-11>>

      <with|par-left|<quote|1tab>|4.8<space|2spc>Map Fusion
      <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-12>>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|5<space|2spc>ROS
      Node Architecture> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-13><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|6<space|2spc>TF
      and Frame Structure> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-14><vspace|0.5fn>

      <vspace*|1fn><with|font-series|<quote|bold>|math-font-series|<quote|bold>|7<space|2spc>Multi-Robot
      Coordination and Map Fusion> <datoms|<macro|x|<repeat|<arg|x>|<with|font-series|medium|<with|font-size|1|<space|0.2fn>.<space|0.2fn>>>>>|<htab|5mm>>
      <no-break><pageref|auto-15><vspace|0.5fn>
    </associate>
  </collection>
</auxiliary>