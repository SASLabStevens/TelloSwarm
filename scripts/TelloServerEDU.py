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
    It uses hardware redundacy (multiple Wi-Fi adapters) to establish unique 
    Wi-Fi connections, allowing for performing swarming and formation of drones
    that is not officially supported for standard Tello drones.
    
    Tello API's official documentation:
    [1.3](https://terra-1-g.djicdn.com/2d4dce68897a46b19fc717f3576b7c6a/Tello%20%E7%BC%96%E7%A8%8B%E7%9B%B8%E5%85%B3/For%20Tello/Tello%20SDK%20Documentation%20EN_1.3_1122.pdf)
    
    """
    ## Global Parameters
    
    #  socket
    BINDING_IP = ''
    CONTROL_UDP_PORT = 8889    # Tello UDP port
    
    # mimic TCP
    MAX_RETRY = 1  # max number to repeat important commands e.g. land, emergency
    
    def __init__(self, drone_IP):
        """
    
        Parameters
        ----------
        wifi_interface : string
             wifi interface's name is used to establish unique wifi connections
             between multiple standard Tello drones (as servers with IP:192.168.10.1  
             and Port 8889) and multiple wifi adapters (as clients with IP ranges
             192.168.10.X).

        Returns
        -------
        None.

        """

    
        # self.TELLO_IP = '192.173.67.194'  # Tello IP address

         
        self.TELLO_IP = drone_IP # Tello IP address

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
        # self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, self.bytes_interface)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('', 8889))
                        
        self.address = (self.TELLO_IP, Telloserver.CONTROL_UDP_PORT)
    
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
            self.server_socket.sendto('emergency'.encode(), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered   
            
    
    def takeoff(self):
        """
    
        """
        self.server_socket.sendto('takeoff'.encode('utf-8'), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered   
    
    def land(self): 
        return self.server_socket.sendto('land'.encode('utf-8'), 0, self.address)
        # TODO: add the receiving feature to check if the message has been delivered       
            
        
       
    def cmdAngleZ(self, roll, pitch, throttle, yawRate): 
            """
                
            - Sends remote control commands in four channels.
            - The body frame follows a right-handed Front(x) Left(y) Up(z) (FLU) convention. 
            
            - Roll angle and pitch angle are the setpoints given in the body-frame.
            - throttle controls the altitude in the Z axis of the drone's FLU body-frame.
            - yaw rate is the rotation rate about the z axis. 
            
            - Frame Convention:
                -- Roll angle: cw(-) and ccw(+) rotation about the X axis to move, respectively, in the left and right direction, w.r.t. the FLU body-frame.
                -- pitch angle: cw(-) and ccw(+) rotation about the Y axis to move, respectively, in the backward and forward direction, w.r.t. the FLU body-frame.
                -- yaw angle: cw(-) and ccw(+) rotation about the Z axis 
                
            Arguments:
                roll (int): percentage of desired roll angle (between -100 to +100 corresponding to -10 to 10 degrees).
                pitch (int): percentage of desired pitch angle (between -100 to +100 corresponding to -10 to 10 degrees).
                throttle (int): throttle input of the drone (between -100 to +100 corresponding to ~ -100 [cm/sec] to ~ +100 [cm/sec])
                yawRate: -100~100  (ToDo)

            """
            def clamp(x: int, min_value: int, max_value: int) -> int:
                return max(min_value, min(max_value, x))
            
            cmd = 'rc {} {} {} {}'.format(
                                        clamp(round(roll),-100,100),
                                        clamp(round(pitch),-100,100),
                                        clamp(round(throttle),-100,100),
                                        clamp(round(yawRate),-100,100))

            self.server_socket.sendto(cmd.encode('utf-8'), 0, self.address)
    
    
        # =============== receive the drone's onboard data  =================
        
        # TODO 
        
# ===============   Multi-agent API for swarm and formation  ================

from mocap import MotionCapture, rosClock

class SWARM:
    """
    Multi-agent API for swarm and formation 
    
    """
    
    def __init__(self, dronesDict ,
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
        # self.droneslist = []
        # for drone in dronesDict:
        #     self.TELLO_IP = dronesDict[drone] # Tello IP address
        #     self.Drone_Name = drone # Tello name
        #     self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #     self.server_socket.bind((Telloserver.BINDING_IP, Telloserver.CONTROL_UDP_PORT))
        #     self.address = (self.TELLO_IP, Telloserver.CONTROL_UDP_PORT)
            

            
        self.allDrones = []
        self.droneslist = [] # just the list of names
        for drone in dronesDict:
            self.droneslist.append(drone)
            # self.Drone_Name = drone # Tello name
            ip = dronesDict[drone] # Tello IP address
            drone_obj = Telloserver(ip)
            self.allDrones.append(drone_obj)
            
        
        
        self.rosClock = rosClock
 

    def MotionCaptureGroundTruth(self):
        self.GroundTruth = []
        for drone in self.droneslist:
            self.GroundTruth.append(MotionCapture(drone))  
        return self.GroundTruth
    
    # the landing function was written by Shalemuraju Katari @ https://github.com/SASLabStevens/Swarm-Drones-Using-Shape-Vectors
    # The landing function was not meant to be optimum; modification and rectification may be required!   
    # @staticmethod
    def Landing(GroundTruth,allDrones,rosClock):
        """
        a landing function that uses motion capture data  

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
                                  
                        landing_velocity = altitude[idx] * 10 if altitude[idx] < .035 else 50
                        drone.cmdAngleZ(0,0,-landing_velocity,0)
                    else:
                        drone.emergency()
                        rosClock.sleepForRate(10)
                        drone.emergency()
                        count += 1

                        if count == len(altitude): 
                            safe_height = True                             

            except KeyboardInterrupt:
        
                print('emergency interruption!; aborting all flights ...')
                for drone in allDrones:
                    drone.emergency()
                
                rosClock.sleepForRate(10) 

 

    
    