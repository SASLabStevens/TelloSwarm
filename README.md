# TelloSwarm+
  
---

The code used in [Bahrami and Jafarnejadsani, IEEE ICUAS 2025](https://ieeexplore.ieee.org/abstract/document/11007832).

If you find this code useful in your research, please consider citing our paper:

```
@inproceedings{bahrami2025multi,
  title={Multi-Robot Coordination with Adversarial Perception},
  author={Bahrami, Rayan and Jafarnejadsani, Hamidreza},
  booktitle={2025 International Conference on Unmanned Aircraft Systems (ICUAS)},
  pages={370--377},
  year={2025},
  organization={IEEE}
}
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
git clone --recurse-submodules -b ICUAS_2025 https://github.com/SASLabStevens/TelloSwarm

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
