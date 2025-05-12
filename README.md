# Computing Project - IIRDIA
 Computing Project. Goal is to implement E. Loems Master's Thesis for ulterior work. 

## Hardware Setup

This project is designed to work with:
- Sphero RVR robots
- IRIDIA's custom configuration:
    - 2x Raspberry Pi 4
    - TeraRanger distance sensors
    - YDLidar X4 for mapping
    - Oak-D camera

The project is currently in early development. Files are being organized and the build system is not yet functional. Compilation is not possible at this stage.

## Prerequisites

Hardware requirements will be detailed once the implementation is complete.



## Connecting to the Sphero RVR and Launching Ros Components

1. **SSH into the RVR**:

   Replace `<robot_ip>` with the actual IP address of your RVR and run:

   ```bash
   ssh spherorvr@<robot_ip>
   ```

2. **Start the Sphero RVR driver**:

   Depending on the mode you want to use (synchronous or asynchronous), run one of the following commands:

   * For the synchronous driver:

     ```bash
     rosrun rvr_ros rvr_driver.py
     ```

   * For the asynchronous driver:

     ```bash
     rosrun rvr_ros rvr_async_driver.py
     ```

3. **Start the YDLidar X4 module**:

   To launch the LiDAR for mapping, execute:

   ```bash
   roslaunch ydlidar X4.launch
   ```