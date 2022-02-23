# TelloSwarm

This repository contains the code for the following publication:

M. Bahrami, and H. Jafarnejadsani, "Detection of Stealthy Adversaries for Networked Unmanned Aerial Vehicles".

- It provides Python scripts to interact with multiple [Ryze Tello drones](https://www.ryzerobotics.com/tello) in swarming and formation control settings.
- It provides Python scripts for observer-based intrusion detection in multi-UAV control settings.
- It uses the ground truth pose of each drone obtained from a ([VICON](https://www.vicon.com/)) motion capture system through the [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) ROS package.

If you find this code useful in your research, please consider citing our paper:

```
@misc{bahrami2022detection,
      title={Detection of Stealthy Adversaries for Networked Unmanned Aerial Vehicles}, 
      author={Mohammad Bahrami and Hamidreza Jafarnejadsani},
      year={2022},
      eprint={2202.09661},
      archivePrefix={arXiv},
      primaryClass={eess.SY}
}
```
For details, check out the [PDF](), the [video](https://youtu.be/lVT_muezKLU), and the following [instructions](#instructions).

[![stealthy-Intrusion-Detection](https://img.youtube.com/vi/lVT_muezKLU/0.jpg)](https://youtu.be/lVT_muezKLU)

# Instructions

The code has been tested on Ubuntu 20.04 and ROS Noetic with Python 3.6+.

### Prerequisites

[ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and catkin tools. The Desktop-Full Installation is recommended.

### Installation

1. Create a catkin workspace named telloswarm_ws: 

`mkdir -p ~/telloswarm_ws/src && cd ~/telloswarm_ws/src`

2. Clone the vicon_bridge and TelloSwarm repositories: 

`git clone https://github.com/m-bahrami/vicon_bridge && git clone https://github.com/SASLabStevens/TelloSwarm`

3. Build and source the setup file:

`cd ~/telloswarm_ws/ && catkin_make`

`source devel/setup.bash`


### Configuratuion


