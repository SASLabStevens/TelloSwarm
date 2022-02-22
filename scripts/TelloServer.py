#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 26 11:30:08 2021

@author: Mohamad Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""


import socket
import time


class Telloserver:
    """
    Python wrapper to interact with multiple Ryze Tello drones. 
    It is based on the official Tello API for standard Tello drones.
    It uses hardware redundacy (multiple Wi-Fi adapters) to establish multiple 
    Wi-Fi channels allowing for connecting to multiple Tellos that do not 
    officially support operations in Station Mode (Client Mode).

    
    Tello API's official documentation:
    [1.3](https://terra-1-g.djicdn.com/2d4dce68897a46b19fc717f3576b7c6a/Tello%20%E7%BC%96%E7%A8%8B%E7%9B%B8%E5%85%B3/For%20Tello/Tello%20SDK%20Documentation%20EN_1.3_1122.pdf)
    
    """
    ## Global Parameters
    
    # server socket
    TELLO_IP = '192.168.10.1'  # Tello IP address
    CONTROL_UDP_PORT = 8889    # Tello UDP port
    
    # mimic TCP
    MAX_RETRY = 1  # max number to repeat high-level control commands such as land and emergency
    
    def __init__(self, wifi_interface):
        """
    
        Parameters
        ----------
        wifi_interface : string
             Wi-Fi interface's name is used to establish wifi connections
             between multiple standard Tello drones (with IP:192.168.10.1  
             and Port 8889) and multiple Wi-Fi adapters (with an IP range
                                                         of your choice).

        Returns
        -------
        None.

        """
        self.wifi_interface = wifi_interface
        self.bytes_interface = self.wifi_interface.encode("utf-8")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, self.bytes_interface)
        
        self.address = (Telloserver.TELLO_IP, Telloserver.CONTROL_UDP_PORT)
    

    
    # =====================  drone's control commands  ====================

    def connect(self):
        """
        Enables sending control commands to a Tello drone
        """
        self.server_socket.sendto('command'.encode('utf-8'), 0, self.address)
        
    def disconnect(self):
        """
        closes the UDP channels used for sending commands to a Tello drone
        """
        self.server_socket.close()    
    
    def emergency(self):
        """
        Stops all motors immediately.
        """
        # self.server_socket.sendto('emergency'.encode(), 0, self.address)
        for i in range(Telloserver.MAX_RETRY):
            self.server_socket.sendto('emergency'.encode('utf-8'), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered   
            
    
    def takeoff(self):
        """
    
        """
        self.server_socket.sendto('takeoff'.encode('utf-8'), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered   
    
    def land(self): 
        return self.server_socket.sendto('land'.encode('utf-8'), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered       
            
        
       
    def cmdAngleVelocityZ(self, roll, pitch, Vz, yawRate): 
        """
            
        - Sends remote control commands in four channels.
        - The body frame follows a right-handed Front(x) Left(y) Up(z) (FLU) convention. 
        
        - Roll angle and pitch angle are the setpoints given in the body-frame.
        - Vz is the velcoity in the Z axis of the drone's FLU body-frame.
        - yaw rate is the rotation rate about the z axis. 
        
        - Frame Convention:
            -- Roll angle: cw(-) and ccw(+) rotation about the X axis to move, respectively, in the left and right direction, w.r.t. the FLU body-frame.
            -- pitch angle: cw(-) and ccw(+) rotation about the Y axis to move, respectively, in the backward and forward direction, w.r.t. the FLU body-frame.
            -- yaw angle: cw(-) and ccw(+) rotation about the Z axis 
            
        Arguments:
            roll (int): percentage of desired roll angle (between -100 to +100 corresponding to -10 to 10 degrees).
            pitch (int): percentage of desired pitch angle (between -100 to +100 corresponding to -10 to 10 degrees).
            Vz (int): desired velocity in the Z axis of the drone's FLU body-frame (between -100 to +100 corresponding to -100 [cm/sec] to +100 [cm/sec])
            yawRate: -100~100  (ToDo)

        """
        def clamp(x: int, min_value: int, max_value: int) -> int:
            return max(min_value, min(max_value, x))
        
        cmd = 'rc {} {} {} {}'.format(
                                    clamp(round(roll),-100,100),
                                    clamp(round(pitch),-100,100),
                                    clamp(round(Vz),-100,100),
                                    clamp(round(yawRate),-100,100))

        self.server_socket.sendto(cmd.encode('utf-8'), 0, self.address)
    
    
        # =============== receive the drone's onboard data  =================
        
        # TODO 
        
# ===============   Multi-agent API for swarm and formation  ================

from mocap import MotionCapture, rosClock

class SWARM:
    """
    - Multi-UAV API for swarm and formation 
    - It uses a motion capture system to obtain the ground truth pose
    
    """
    
    def __init__(self, wifi_interfaces: list,
                 droneslist:list = None,
                 defaultName = 'Drone'):
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
        if droneslist is None:
            self.droneslist = []
            for i in range(len(wifi_interfaces)):
                self.droneslist.append(defaultName + str(i+1))
        else:
            self.droneslist = droneslist
            
        self.allDrones = []
        for drone in self.droneslist:
            idx = self.droneslist.index(drone)
            drone = Telloserver(wifi_interface=wifi_interfaces[idx])
            self.allDrones.append(drone)
        
        
        self.rosClock = rosClock
 

    def MotionCaptureGroundTruth(self):
        self.GroundTruth = []
        for drone in self.droneslist:
            self.GroundTruth.append(MotionCapture(drone))  
        return self.GroundTruth
    
    
 

    
    
