# TelloSwarm

This repository contains the code for the following publication:

[1] M. Bahrami, and H. Jafarnejadsani, "Detection of Stealthy Adversaries for Networked Unmanned Aerial Vehicles" ([PDF](https://arxiv.org/abs/2202.09661)).

- It provides Python scripts to interact with multiple [Ryze Tello drones](https://www.ryzerobotics.com/tello) in swarming and formation control settings.
- It provides Python scripts for observer-based intrusion detection in multi-UAV control settings.
- It uses the ground truth pose of each drone obtained from a ([VICON](https://www.vicon.com/)) motion capture system through the [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) ROS package.

If you find this code useful in your research, please consider citing our paper:

```
@article{bahrami2022detection,
  title={Detection of Stealthy Adversaries for Networked Unmanned Aerial Vehicles},
  author={Bahrami, Mohammad and Jafarnejadsani, Hamidreza},
  journal={arXiv preprint arXiv:2202.09661},
  year={2022}
}
```
For details, check out the [PDF](https://arxiv.org/abs/2202.09661), the [video](https://youtu.be/lVT_muezKLU), and the following [instructions](#instructions).

[![stealthy-Intrusion-Detection](https://img.youtube.com/vi/lVT_muezKLU/0.jpg)](https://youtu.be/lVT_muezKLU)

# Instructions

The code has been tested on Ubuntu 20.04 and ROS Noetic with Python 3.6+.

### Prerequisites

[ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and catkin tools. The Desktop-Full Installation is recommended.

### Installation

1. Create a catkin workspace named telloswarm_ws and move to its srouce (src) folder: 

```
mkdir -p ~/telloswarm_ws/src && cd ~/telloswarm_ws/src
```

2. Clone the vicon_bridge and TelloSwarm repositories: 

```
git clone https://github.com/m-bahrami/vicon_bridge && git clone https://github.com/SASLabStevens/TelloSwarm
```

3. Build and source the setup file:

```
cd ~/telloswarm_ws/ && catkin_make
```

```
source devel/setup.bash
```

A successfully installed workstation has the following structure in your computer:

<pre>
~/telloswarm_ws/                                -- telloswarm WORKSPACE
├── src/                                        -- SOURCE SPACE
|   |
│   ├── TelloSwarm/                             -- flight control, monitoring, and comm. network code stack
│   │   └── TelloSwarm_files
│   ├── vicon_bridge/                           -- motion capture system's PKG, providing the ground truth
|   |   ├── launch/vicon.launch
|   |   ├── package.xml
│   │   └── Other_vicon_bridge_files_&_folders
│   └── CMakeLists.txt                          -- 'Toplevel' CMake file, provided by catkin
├── devel/
└── build/
</pre>

4. In the `vicon.launch` file located in `cd ~/telloswarm_ws/src/vicon_bridge/launch`, you may need to set the "datastream_hostport" parameter to the IP/hostname of your vicon's host PC and the "stream_mode" paramter to either of "ServerPush", "ClientPull", or "ClientPullPreFetch" modes. For details, consult [Vicon DataStream SDK Developer's Guide](https://docs.vicon.com/display/DSSDK111/DataStream+SDK+Documentation).

5. Run the `vicon.launch` file and `TEST_mocap.py` to check if the installed ROS and Python packages work:

- In the already-opened terminal run

```
roslaunch vicon_bridge vicon.launch
```

which will start streaming motion captue data using ROS. 

- In a new terminal, run `TEST_mocap.py` to access the ground truth pose of a drone defined in the VICON Tracker. Inside this python file, you need to set the "OBJECT_Name" parameter to the name of a drone created in the VICON Tracker.
```
python3 ~/catkin_ws/src/vicon_bridge/TelloSwarm/scripts/test_mocap.py 
```      
   or
```   
cd ~/telloswarm_ws/src/TelloSwarm/scripts/
python3 test_mocap.py
```
The `test_mocap.py` is the test file of the python module `mocap.py` providing the positions and rotations of the objects(drones) using [tf transform](http://wiki.ros.org/tf/Tutorials). 

### Configuration

This is a one-time setup with two parts: 1) creating/naming objects associated with (Tello) drones in the VICON Tracker, and 2) configuring Wi-Fi connections with (Tello) drones. After the first time, the names and addresses assigned in this section will be used when running flight tests or developing control and monitoing algorithms.

#### Name Drones (in [VICON Tracker](https://www.vicon.com/software/tracker/)).
We will be using **a list of names**, assigned to the drones in the VICON Tracker software, as the **identifiers** of drones in Python programming. Particularly, the python module `mocap.py` uses this list in obtaining the ground truth pose of the drones tracked by the motion capture system.

#### Wi-Fi Network
We use the following communication network architecture:
<pre> 
    PC (in STA mode) 	                           Drones (in AP mode) 
     
  =======================                    ============================== 
 
Wi-Fi adapter 1 (e.g. 192.168.50.2) ------> Tello Drone 1 (192.168.10.1:8889)
  
Wi-Fi adapter 2 (e.g. 192.168.50.3) ------> Tello Drone 2 (192.168.10.1:8889)
  
Wi-Fi adapter 3 (e.g. 192.168.50.4) ------> Tello Drone 3 (192.168.10.1:8889)
              :                                         :   
              :                                         :     				         	         
</pre>

Each Wi-Fi adapter has a unique [Wi-Fi interface](https://askubuntu.com/questions/405508/how-to-find-name-of-currently-active-network-interface) as its identifier which later will be used by the python module `TelloServer.py` to enable the Wi-Fi communication and send control commands. 

The communication network is set up using NetworkManager GUI or TUI

1. Connect to a Tello drone using a Wi-Fi adapter.
2. Run `ifconfig` in a terminal and find the name of the [active Wi-Fi interface](https://askubuntu.com/questions/405508/how-to-find-name-of-currently-active-network-interface) to which the drone is connected.
3. Configure Wi-Fi connections by [assigning](https://help.ubuntu.com/stable/ubuntu-help/net-manual.html.en) a unique IP address from the range `192.168.10.2-255` with Netmask `255.255.255.0` to the drone's connection. Reconnect the drone and check if the connection is now established using the assigend IP to the Wi-Fi adpater.\
Follow steps 1-3 for each drone, separately, and reconnect them at the end. Also, the terminal commands `ifconfig` with details in [(here)
](https://manpages.ubuntu.com/manpages/focal/man8/ifconfig.8.html) and `route` with details in [(here)](https://manpages.ubuntu.com/manpages/focal/man8/route.8.html) might be useful. If you have configured the IP addresses as described and all Tello drones are connected, running `route` or `ifconfig` in a termnial will show a list of currently-active connections with their IP addresses and Wi-Fi interface names. 
4. The module `TelloServer.py` takes the Wi-Fi interface names, as identifiers, to establish connections.


### Running flight test experiments
Here we show how to use TelloSwarm in multi-UAV cooperative control settings as well as to reproduce the results in [[1]](https://arxiv.org/abs/2202.09661). 

At this point, we assume the VICON Tracker is running on the streaming mode with the drone's objects created and Wi-Fi connections established.

1. In a terminal run the vicon_bridge roslaunch file to enable motion capture data streaming through ROS
```
source ~/telloswarm_ws/devel/setup.bash
roslaunch vicon_bridge vicon.launch
```
2. In a new terminal run the example/test file `test_TelloSwarm.py`. Inside the `test_TelloSwarm.py` file, you may need to change the "wifi_interfaces" and "droneslist" lists according to your own set-up.
```
cd ~/telloswarm_ws/src/TelloSwarm/scripts/
python3 test_TelloSwarm.py
```
This test file serves as a starting point for any other applications by demonstrating how to use the modules `TelloServer.py` and `mocap.py` to simultaneously control multiple Tello drones in a simple flight test that is take off, go to the prespecified hovering setpoints during five seconds, and then land. 

To reproduce the results in [[1]](https://arxiv.org/abs/2202.09661), in step 2 run the files with the names `formation_5drones_*.py` where * is `adv_free`, `ZDA`, or `covert`.
