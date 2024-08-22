# TelloSwarm
  
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

# Instructions

The code has been tested on Ubuntu 20.04 and ROS Noetic with Python 3.8+.

### Installation

  [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and catkin tools are required.

```
# Create a catkin workspace named telloswarm_ws and set the current directory to the source (src) folder: 
mkdir -p ~/telloswarm_ws/src && cd ~/telloswarm_ws/src

# Clone the vicon_bridge and TelloSwarm_plus repositories:
git clone https://github.com/r-bahrami/vicon_bridge
git clone --recurse-submodules -b 2024 https://github.com/SASLabStevens/TelloSwarm

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

### How to use

Use `kf_consensus_wrt_jackal.py` to replicate the experiments.
