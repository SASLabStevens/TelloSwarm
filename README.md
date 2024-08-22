# TelloSwarm


### Updates 

- **August 2024** | *TelloSwarm+*:

  A multi-threaded API for formation control and video streaming from multiple [Tello EDU](https://www.ryzerobotics.com/tello-edu) drones over a wireless network (WLAN).

- **June 2022** | *TelloSwarm*:

  An API for formation control of multiple *standard* [Tello](https://www.ryzerobotics.com/tello) drones over a wireless network (WLAN).
  
  See the [ICUAS_2022](https://github.com/SASLabStevens/TelloSwarm/tree/ICUAS_2022) branch for the code used in [Bahrami and Jafarnejadsani, IEEE ICUAS 2022](https://ieeexplore.ieee.org/abstract/document/9836208).
---

If you find this code useful in your research, please consider citing our paper:

```
@INPROCEEDINGS{9836208,
  author={Bahrami, Mohammad and Jafarnejadsani, Hamidreza},
  booktitle={2022 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Detection of Stealthy Adversaries for Networked Unmanned Aerial Vehicles}, 
  year={2022},
  volume={},
  number={},
  pages={1111-1120},
  doi={10.1109/ICUAS54217.2022.9836208}}
```
For details, check out the [PDF](https://arxiv.org/abs/2202.09661), the [video](https://youtu.be/lVT_muezKLU), and the following [instructions](#instructions).

[![stealthy-Intrusion-Detection](https://img.youtube.com/vi/lVT_muezKLU/0.jpg)](https://youtu.be/lVT_muezKLU)


# Instructions

The code has been tested on Ubuntu 20.04 and ROS Noetic with Python 3.8+.

### Installation

- **Install TelloSwarm+ with pip**
```
git clone https://github.com/SASLabStevens/TelloSwarm
pip install -r requirements.txt
  ```

- **Install with ROS** (optional for motion capture integration)

  [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and catkin tools are required.

```
# Create a catkin workspace named telloswarm_ws and set the current directory to the source (src) folder: 
mkdir -p ~/telloswarm_ws/src && cd ~/telloswarm_ws/src

# Clone the vicon_bridge and TelloSwarm_plus repositories:
git clone https://github.com/r-bahrami/vicon_bridge && git clone https://github.com/SASLabStevens/TelloSwarm

# Build and source the setup file:
cd ~/telloswarm_ws/ && catkin_make
#
source devel/setup.bash

# install packages
pip install -r requirements.txt
```

  In the `vicon.launch` file located in `cd ~/telloswarm_ws/src/vicon_bridge/launch`, you may need to set the "datastream_hostport" parameter to the IP/hostname of your vicon's host PC and the "stream_mode" parameter to either of "ServerPush", "ClientPull", or "ClientPullPreFetch" modes. For details, consult [Vicon DataStream SDK Developer's Guide](https://docs.vicon.com/display/DSSDK111/DataStream+SDK+Documentation).

  Run the `vicon.launch` file and `mocap.py` to check if ROS and Python packages work:

  In the already-opened terminal run

```
roslaunch vicon_bridge vicon.launch
```

  which will start streaming motion capture data using ROS. 

  In another terminal, run `mocap.py` to access the ground truth pose of a drone defined in the VICON Tracker. When asked to enter the "OBJECT_Name", it is the name of a drone created in the VICON Tracker.
```
python3 ~/telloswarm_ws/src/TelloSwarm/telloswarm_plus/mocap.py 
```      

### How to use

For network configuration and and instructions on how to use this code, please refer to the [README](https://github.com/SASLabStevens/TelloSwarm/tree/main/telloswarm_plus) file and the sample code `test_api.py` inside the `telloswarm_plus` directory.
