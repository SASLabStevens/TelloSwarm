#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 26 11:30:08 2021 - modified on May 2024

@author: Rayan Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

import socket
import time
import threading

import cv2
from telloserver_video import VideoStream

from dataclasses import dataclass
from typing import Tuple


@dataclass
class State:
    rotation: Tuple[float, float, float] = (0, 0, 0)
    velocity: Tuple[float, float, float] = (0, 0, 0)
    acceleration: Tuple[float, float, float] = (0, 0, 0)
    altitude: float = 0
    barometer: float = 0
    time: int = 0  # encapsulated time of the motors running


class TelloServer:
    """Python API to interact with multiple Tello EDU drones.
    It is based on the official SDK 3.0 for Tello EDU drones.
    It works with a wifi network (WLAN) to establish unique
    Wi-Fi connections with drones, allowing for performing swarming and formation of drones
    as well as receiving the drones' onboard data and video stream.

    Tello API's official documentation:
    [2](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
    [3](https://dl.djicdn.com/downloads/RoboMaster+TT/Tello_SDK_3.0_User_Guide_en.pdf)

    It creates 1 client and 2 servers on 2 threads.
    The client sends control commands to the drones.
    The two servers on two threads receive the drone's state and the drone's response to the sent commands.
    """

    ## Global Parameters

    # client socket to send commands to the drone
    BINDING_IP = ""  # server IP - local host
    CONTROL_UDP_PORT = 8889  # Tello UDP port

    # server socket to receive the drone's state
    STATE_UDP_PORT = 8890
    VIDEO_PORT = 11111

    FPS = {5: "low", 15: "middle", 30: "high"}
    VIDEO_RES = {"480p": "low", "720p": "high"}

    # mimic TCP
    MAX_RETRY = 3  # max number to repeat important commands e.g. land, emergency

    def __init__(
        self,
        drone_IPs=["192.168.10.1"],
        LOCAL_PORT=9000,
    ):
        """__init__

        Args:
            drone_IPs (list, optional): _description_. Defaults to ["192.168.10.1"].
            The list will be used to create a dictionary of drones states, named self.states with the IPs as keys.
            LOCAL_PORT (int, optional): _description_. Defaults to 9000.

        """

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, self.bytes_interface)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("", LOCAL_PORT))  # server

        # self.address = (self.TELLO_IP, TelloServer.CONTROL_UDP_PORT)

        # thread for receiving cmd ack
        self.msg = b"None"  # latest message received from the drone (Bytes)
        self.client_addr = ["None", 8889]  # Tello IP and port 8889
        self.recv_msg_thread = threading.Thread(target=self._recv_msg)
        self.recv_msg_active = True
        self.recv_msg_thread.start()

        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.state_socket.bind(("", TelloServer.STATE_UDP_PORT))
        self.state_socket.bind(("0.0.0.0", TelloServer.STATE_UDP_PORT))

        # latest state received from the drones
        self.states = {ip: State() for ip in drone_IPs}
        self.recv_state_thread = threading.Thread(target=self._recv_state)
        self.recv_state_active = True
        self.recv_state_thread.start()

    # =====================  drone's response commands  ====================

    def _recv_msg(self):
        """_recv_msg _summary_"""
        while self.recv_msg_active:
            try:
                self.msg, self.client_addr = self.socket.recvfrom(3000)
                # print(f"client_addr: {client_addr}")  # Tello IP and port 8889
                # print(f"Received message: {self.msg.decode('utf-8')}")
            except socket.error as e:
                print(f"Exception socket.error : {e}")

    def _recv_state(self):
        """_recv_state callback"""
        while self.recv_state_active:
            try:
                data, addr = self.state_socket.recvfrom(3000)
                (
                    self.states[addr[0]].rotation,
                    self.states[addr[0]].velocity,
                    self.states[addr[0]].acceleration,
                    self.states[addr[0]].altitude,
                    self.states[addr[0]].barometer,
                    self.states[addr[0]].time,
                ) = self._decode_state(data)
                # tello_IP, tello_PORT = tello_addr[0], tello_addr[1]
                # print(f"state of tello {tello_addr}")
                # print(f"Received state: {self.state}")
            except socket.error as e:
                print(f"Exception socket.error : {e}")

    def _decode_state(self, data: bytes):

        # test # data = "mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:0;roll:1;yaw:28;vgx:5;vgy:0;vgz:0;templ:69;temph:72;tof:6553;h:80;bat:83;baro:-64.11;time:41;agx:-5.00;agy:-26.00;agz:-1010.00"

        data = data.decode("utf-8")
        key_value_pairs = data.split(";")

        rot, vel, acc, altitude, barometer, time = {}, {}, {}, 0, 0, 0
        for pair in key_value_pairs:
            if pair and ":" in pair:
                key, value = pair.split(":")
                if key in ["roll", "pitch", "yaw"]:
                    rot[key] = int(value)
                elif key.startswith("vg"):
                    vel[key] = float(value) / 10  # dm/s to m/s
                elif key.startswith("ag"):
                    acc[key] = float(value) / 100  # cm/s^2 to m/s^2
                elif key == "h":
                    altitude = float(value) / 100  # cm to m
                elif key == "baro":
                    barometer = float(value)  # ? see their doc
                elif key == "time":
                    time = int(value)  # seconds

        rotation = (
            rot.get("roll", 0),
            -rot.get("pitch", 0),
            -rot.get("yaw", 0),
        )  # degrees in FLU = xyz body-fixed frame

        velocity = (
            vel.get("vgx", 0.0),
            -vel.get("vgy", 0.0),
            -vel.get("vgz", 0.0),
        )  #  m/s in FLU = xyz body-fixed frame

        acceleration = (
            acc.get("agx", 0.0),
            acc.get("agy", 0.0),
            acc.get("agz", 0.0),
        )  # m/s^2

        return rotation, velocity, acceleration, altitude, barometer, time

    # =====================  drone's control commands  ====================

    def connect(self, addr):
        """
        Enables sending control commands to a Tello drone
        """
        self.socket.sendto("command".encode("utf-8"), 0, addr)

    def disconnect(self):
        """
        closes the UDP channels used for sending commands to a Tello drone
        """
        print(f"Stopping state thread ...")
        self.recv_state_active = False
        self.recv_state_thread.join()

        print(f"Stopping msg thread ...")
        self.recv_msg_active = False
        self.recv_msg_thread.join()

        print(f"Closing sockets connections ...")
        self.state_socket.close()
        self.socket.close()

    def emergency(self, addr):
        """stop all motors immediately."""
        self.socket.sendto("emergency".encode("utf-8"), 0, addr)
        for _ in range(TelloServer.MAX_RETRY):
            print(f"{addr} emergency: {self.msg.decode('utf-8')}")
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                break
            else:
                time.sleep(0.1)
                self.socket.sendto("emergency".encode("utf-8"), 0, addr)

    def takeoff(self, addr):
        self.socket.sendto("takeoff".encode("utf-8"), 0, addr)
        for _ in range(TelloServer.MAX_RETRY):
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                print(f"{addr} takeoff: {self.msg.decode('utf-8')}")
                break
            else:
                time.sleep(0.1)
                self.socket.sendto("takeoff".encode("utf-8"), 0, addr)

    def land(self, addr):
        self.socket.sendto("land".encode("utf-8"), 0, addr)
        for _ in range(TelloServer.MAX_RETRY):
            print(f"{addr} land: {self.msg.decode('utf-8')}")
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                break
            else:
                time.sleep(0.1)
                self.socket.sendto("land".encode("utf-8"), 0, addr)

    def cmdAngleZ(self, roll, pitch, throttle, yawRate, addr):
        """

        - Sends remote control commands in four channels.
        - The body frame follows a right-handed Front(x) Left(y) Up(z) (FLU) convention.

        - Roll angle and pitch angle are the setpoints given in the body-frame.
        - throttle controls the altitude in the Z axis of the drone's FLU body-frame.
        - yaw rate is the rotation rate about the z axis.

        - Frame Convention:
            -- Roll angle: cw(-) and ccw(+) rotation about the X axis to move, respectively, in the left and right direction, w.r.t. the FLU body-frame.
            -- pitch angle: cw(-) and ccw(+) rotation about the Y axis to move, respectively, in the backward and forward direction, w.r.t. the FLU body-frame.
            -- yaw angle: cw(+) and ccw(-) rotation about the Z axis; yes it's negative in the SDK!

        Arguments:
            roll (int): percentage of desired roll angle (between -100 to +100 corresponding to -10 to 10 degrees).
            pitch (int): percentage of desired pitch angle (between -100 to +100 corresponding to -10 to 10 degrees).
            throttle (int): throttle input of the drone (between -100 to +100 corresponding to ~ -100 [cm/sec] to ~ +100 [cm/sec])
            yawRate: -100~100  (ToDo)

        """

        def clamp(x: int, min_value: int, max_value: int) -> int:
            return max(min_value, min(max_value, x))

        cmd = "rc {} {} {} {}".format(
            clamp(round(roll), -100, 100),
            clamp(round(pitch), -100, 100),
            clamp(round(throttle), -100, 100),
            clamp(round(yawRate), -100, 100),
        )

        self.socket.sendto(cmd.encode("utf-8"), 0, addr)

    # =============== SDK 3.0 receive the drone's onboard video  =================

    def send_streamon(self, addr):
        """Enable video stream"""
        self.socket.sendto("streamon".encode("utf-8"), 0, addr)
        time.sleep(0.1)
        STREAM = False
        for _ in range(TelloServer.MAX_RETRY * 2):
            print(f"{addr} streamon: {self.msg.decode('utf-8')}")
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                STREAM = True
                break
            else:
                time.sleep(0.1)
                self.socket.sendto("streamon".encode("utf-8"), 0, addr)
        return STREAM

    def set_video_port(self, vid_port, status_port, addr):
        cmd = f"port {status_port} {vid_port}"
        self.socket.sendto(cmd.encode("utf-8"), 0, addr)
        VIDEO_PORT = False
        for _ in range(TelloServer.MAX_RETRY):
            print(f"{addr} setting video port: {self.msg.decode('utf-8')}")
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                VIDEO_PORT = True
                break
            else:
                time.sleep(0.1)
                self.socket.sendto(cmd.encode("utf-8"), 0, addr)
        return VIDEO_PORT

    def streamoff(self, addr):
        """Disable video stream"""
        self.socket.sendto("streamoff".encode("utf-8"), 0, addr)
        for _ in range(TelloServer.MAX_RETRY * 2):
            print(f"{addr} streamoff: {self.msg.decode('utf-8')}")
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                break
            else:
                time.sleep(0.1)
                self.socket.sendto("streamoff".encode("utf-8"), 0, addr)

    def set_video_fps(self, fps: int, addr):
        """set video stream FPS to 5, 15, or 30"""
        cmd = f"setfps {TelloServer.FPS[fps]}"

        self.socket.sendto(cmd.encode("utf-8"), 0, addr)
        for _ in range(TelloServer.MAX_RETRY):
            print(f"{addr} setting FPS: {self.msg.decode('utf-8')}")
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                break
            else:
                time.sleep(0.1)
                self.socket.sendto(cmd.encode("utf-8"), 0, addr)

    def set_video_resolution(self, res: str, addr):
        """set video res. to 480p or 720"""
        cmd = f"setresolution {TelloServer.VIDEO_RES[res]}"
        self.socket.sendto(cmd.encode("utf-8"), 0, addr)
        for _ in range(TelloServer.MAX_RETRY):
            print(f"{addr} setting video resolution: {self.msg.decode('utf-8')}")
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                break
            else:
                time.sleep(0.1)
                self.socket.sendto(cmd.encode("utf-8"), 0, addr)

    def set_video_bitrate(self, rate, addr):
        """set video bitrate to
        0: auto
        1: 1Mbps
        2: 2Mbps
        3: 2Mbps
        4: 3Mbps
        5: 4Mbps
        """
        cmd = f"setbitrate {rate}"
        self.socket.sendto(cmd.encode("utf-8"), 0, addr)
        for _ in range(TelloServer.MAX_RETRY):
            print(f"{addr} setting video bitrate: {self.msg.decode('utf-8')}")
            if "ok" in self.msg.decode("utf-8") and self.client_addr[0] == addr[0]:
                break
            else:
                time.sleep(0.1)
                self.socket.sendto(cmd.encode("utf-8"), 0, addr)


# ===============   Multi-agent API for swarm and formation  ================

#
from concurrent.futures import ThreadPoolExecutor


class TelloSwarm:
    """
    Multi-agent API for swarm and formation

    """

    def __init__(self, dronesDict, mocap=False):
        """

        Parameters
        ----------
        wifi_interfaces : list
            DESCRIPTION.
            GT
        droneslist : str = optional
            DESCRIPTION.

        defaultName : str, optional
            DESCRIPTION. The default is 'Drone'.

        Returns
        -------
        None.

        """
        # self.droneslist = []
        # for drone in dronesDict:
        #     self.TELLO_IP = dronesDict[drone] # Tello IP address
        #     self.Drone_Name = drone # Tello name
        #     self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #     self.socket.bind((TelloServer.BINDING_IP, TelloServer.CONTROL_UDP_PORT))
        #     self.address = (self.TELLO_IP, TelloServer.CONTROL_UDP_PORT)

        self.droneslist = []  # just the list of names
        self.dronesDict = dronesDict

        self.IPs = [addr[0] for addr in list(dronesDict.values())]
        self.video_ports = [addr[1] for addr in list(dronesDict.values())]

        # 1 client and 2 servers on 2 threads
        self.tellos = TelloServer(drone_IPs=self.IPs)

        # N video streams over UDP on N threads
        self.cameras = {ip: None for ip in self.IPs}
        self.STREAM = False

        self.mocap = mocap
        if self.mocap:
            from mocap import MotionCapture  # , rosClock

            self.GroundTruth = {
                addr[0]: MotionCapture(name) for name, addr in self.dronesDict.items()
            }

    # ============= control commands for the swarm =================
    # TCP-like commands to start and stop the drones in the swarm
    def connect(self):
        for ip in self.IPs:
            addr = (ip, 8889)
            self.tellos.connect(addr)

    def disconnect(self, streamoff=True):
        if self.STREAM and streamoff:
            for ip in self.IPs:
                addr = (ip, 8889)
                # if self.cameras[ip] is not None:
                self.tellos.streamoff(addr)

        cv2.destroyAllWindows()
        self.tellos.disconnect()

    def takeoff(self):
        for ip in self.IPs:
            addr = (ip, 8889)
            self.tellos.takeoff(addr)

    def land(self, delay=3):
        for ip in self.IPs:
            addr = (ip, 8889)
            self.tellos.land(addr)
        time.sleep(delay)

    def emergency(self):
        for ip in self.IPs:
            addr = (ip, 8889)
            self.tellos.emergency(addr)

    # drone control commands to be called for each drone
    def cmdAngleZ(self, drone_IP: str, roll, pitch, throttle, yawRate):
        addr = (drone_IP, 8889)
        # for whatever reason, yawRate is negative FLU (xyz) coord. frame in the SDK
        self.tellos.cmdAngleZ(roll, pitch, throttle, -yawRate, addr)

    # ============= receive drones' onboard data =================
    def get_state(self, drone_IP):
        """get_state

        Args:
            drone_IP (str): drone's IP address on the network

        Returns: a dataclass object with the following attributes:
            rotation = (roll, pitch, yaw) in degrees and FLU body-fixed frame
            velocity = (vx, vy, vz),
            acceleration = (ax, ay, az),
            altitude, barometer, time_of_flight
        """
        return self.tellos.states[drone_IP]

    def streamon(self, bufsize=5, FPS=None, resolution=(None, None), resize=False):
        """activate video stream from Tello drones

        Args:
            bufsize : int,
            FPS : int, frame per second (5 - 30), optional
            resolution (H=720,W=960) : tuple, optional
            resize : True or False, optional - make it True if you get distorted images for lower resolution
        """
        for id, ip in enumerate(self.IPs):
            addr = (ip, 8889)
            PORT = self.tellos.set_video_port(self.video_ports[id], 8890, addr)
            STREAM = self.tellos.send_streamon(addr)

            self.STREAM = True
            if STREAM and PORT:
                self.cameras[ip] = VideoStream(
                    ip,
                    self.video_ports[id],
                    bufsize,
                    FPS,
                    resolution,
                    resize,
                )
                if not self.cameras[ip].video.isOpened():
                    self.STREAM = False  # streamon failed
                    break
            else:
                print(f"streamon failed.")
                self.STREAM = False  # streamon failed
                break

    def get_frame(self, drone_IP):
        """get_frame assuming the video stream is on"""
        return self.cameras[drone_IP].frame

    def streamoff(self):
        """disable video stream asuming the video stream is on"""
        for ip in self.IPs:
            addr = (ip, 8889)
            if self.cameras[ip].video.isOpened():
                self.cameras[ip].stop()  # stops telloserver_video.py
            self.tellos.streamoff(addr)

    def set_video_fps(self, fps: int):
        """set video stream FPS to 5, 15, or 30"""
        for ip in self.IPs:
            addr = (ip, 8889)
            self.tellos.set_video_fps(fps, addr)

    def set_video_resolution(self, res: str):
        """set video res. to 480p or 720"""
        for ip in self.IPs:
            addr = (ip, 8889)
            self.tellos.set_video_resolution(res, addr)

    def set_video_bitrate(self, rate=0):
        """set video bitrate"""
        for ip in self.IPs:
            addr = (ip, 8889)
            self.tellos.set_video_bitrate(rate, addr)

    def get_mocap_groundtruth_pose(self, drone_IP, mode: str = None):
        """get_mocap_groundtruth_pose assuming self.mocap is True
        Args:
            drone_IP (str):
            mode (str, optional): "quaternion", "euler" = "None". Defaults to None.

        Returns:
            position, rotation: np.array of pos=(px, py, pz) in world FLU, and (roll, pitch, yaw) in radians
        """
        return self.GroundTruth[drone_IP].getPose(mode)

    # the landing function was written by Shalemuraju Katari @ https://github.com/SASLabStevens/Swarm-Drones-Using-Shape-Vectors
    # The landing function was not meant to be optimum; modification and rectification may be required!
    # @staticmethod
    def Landing(GroundTruth, allDrones, rosClock):
        """
        a landing function that uses motion capture data
        this is used to land the standard Tello drones safely on the ground.
        for Tello EDU drones, use land() method in TelloSwarm class.

        Arguments:
        ----------
        GT : GroundTruth object ( the output of MotionCaptureGroundTruth method in SWARM class)
        allDrones: the list of drones generated by __init__ method in  SWARM class
        """
        GT = GroundTruth
        # print('Received Landing Command')
        safe_height = False

        while safe_height == False:
            try:

                altitude = []
                for drone in GT:
                    altitude.append(drone.getPose()[0][2])
                count = 0

                for drone in allDrones:
                    idx = allDrones.index(drone)
                    if altitude[idx] > 0.07:

                        landing_velocity = (
                            altitude[idx] * 10 if altitude[idx] < 0.035 else 60
                        )
                        drone.cmdAngleZ(0, 0, -landing_velocity, 0)
                    else:
                        drone.emergency()
                        rosClock.sleepForRate(10)
                        drone.emergency()
                        count += 1

                        if count == len(altitude):
                            safe_height = True

            except KeyboardInterrupt:

                print("emergency interruption!; aborting all flights ...")
                for drone in allDrones:
                    drone.emergency()

                rosClock.sleepForRate(10)
