#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""
Created on Mon Jul 26 11:30:08 2021 - modified on May 2024

@author: Rayan Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

from telloserver_EDU import TelloSwarm
import time
import cv2

DronesDict = {
    # "Name_in_the_VICON_Tracker": ("IP_on_router", "Port_for_Video_stream"),
    # "EDU": ("192.168.10.1", 11111), factory default
    "EDU_7": ("172.16.0.17", 1037),  # drone IP on router, port for Video stream
    "EDU_8": ("172.16.0.18", 1038),
    # "EDU_9": ("172.16.0.19", 1039),
}

Ts, max_itr = 0.02, 100

# fmt: off
swarm = TelloSwarm(DronesDict, mocap=False) # mocap=True if you have vicon_bridge running running
swarm.connect()

swarm.set_video_bitrate(3)  # 0: auto, 1: 1Mbps, 2: 2Mbps, 3: 3Mbps, 4: 4Mbps, 5: 5Mbps
swarm.set_video_fps(30)
swarm.set_video_resolution("480p")  # 480p, 720p,

swarm.streamon(bufsize=10, FPS=30, resolution=(480, 640), resize=True) # (480, 640), (None, None) = (720,960) = (H, w)

if swarm.STREAM:
    swarm.takeoff()

time.sleep(2)

i = 0
while i <= max_itr and swarm.STREAM:
    i += 1
    time.sleep(Ts)
    for id, name in enumerate(DronesDict):
        IP, _ = DronesDict[name]
        print(f"drone {name}")
        swarm.cmdAngleZ(IP, 0, 0, 0, 50)
        if i % 5 == 0:
            states = swarm.get_state(IP)
            print(f"drone {name} state: {states}")
            # pos, rot = swarm.get_mocap_groundtruth_pose(IP)
            # print(f"mocap {name} pos: {pos}, rot: {rot}")
            try:
                frame = swarm.get_frame(IP)
                cv2.imwrite(f"{name}_{time.time()}.png", frame)
                cv2.imshow(f"{name}", frame)  # swarm.cameras[IP].frame
                cv2.waitKey(1)
            except:
                print(f"Error: {name} cv2.imshow() failed")
                continue


swarm.land()

swarm.disconnect()
