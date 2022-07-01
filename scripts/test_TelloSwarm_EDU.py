#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 16 15:47:30 2021

@author: Mohamad Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology


"""

from TelloServerEDU import SWARM

from PID import PID
from Observers import LuenbergerObserver
from scipy.io import loadmat, savemat

import numpy as np

import math, time 

"""
WI-FI interfaces  |        network 
-----------------   ---------------------------------- 
wlx9cefd5fae98b   :  VICON network 2G
wlxa09f10beb890   :  VICON network 5G
______________________________________________________

Tello WiFi name   | client's name and IP in the router
----------------   ----------------------------------

Tello-CBF2C8      :    EDU_1     192.173.67.194
Tello-            :    EDU_2     192.173.67.195
Tello-CBF267      :    EDU_3     192.173.67.196
Tello-CBF2CB      :    EDU_4     192.173.67.197 
Tello-CBF1B9      :    EDU_5     192.173.67.198
Tello-CBF2C1      :    EDU_6     192.173.67.199

"""

DronesDict = {'EDU_1' : '192.173.67.194',
              'EDU_4' : '192.173.67.197'
              }


swarm = SWARM(DronesDict)
allDrones = swarm.allDrones
rosClock = swarm.rosClock()

GroundTruth = swarm.MotionCaptureGroundTruth()

# =======================================================
# Initialize the PID controllers

# =======================================================
# Initialize the PID controllers

MAX_PID = 100 #  saturation on PID outputs

Dterm_filter = 10

K_px = 400
K_dx = 400
K_ix = 130

K_py = 400
K_dy = 400
K_iy = 130

K_pz = 330
K_dz = 330
K_iz = 10

drone1PIDx = PID(Kp= K_px, Kd= K_dx, Ki= K_ix,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time())   
    

drone1PIDy = PID(Kp= K_py, Kd= K_dy, Ki= K_iy,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time()) 

drone1PIDz = PID(Kp= K_pz, Kd= K_dz, Ki= K_iz,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time()) 

# =======================================================

drone2PIDx = PID(Kp= K_px, Kd= K_dx, Ki= K_ix,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time())   
    

drone2PIDy = PID(Kp= K_py, Kd= K_dy, Ki= K_iy,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time()) 

drone2PIDz = PID(Kp= K_pz, Kd= K_dz, Ki= K_iz,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time()) 

# =======================================================
#  main loop's configuration

itr = 0
max_itr = 500

Ts = 0.02 # sample time for the main loop


# hovering setpoints in 3D: [x, y, z]
Ref_1 = [-0.8, 1.5, 0.6] 
Ref_2 = [-0.8,-0.9, 0.6]
# Ref_3 = [0.0, +0.7, 0.6]
# Ref_4 = [+0.8, -1, 0.6]
# Ref_5 = [+0.8, 1.5, 0.6]

# =======================================================

# enable wireless communication with drones
for drone in allDrones:   
    drone.connect()
  
# start taking off
for drone in allDrones:   
    drone.takeoff()

# rosClock.sleep(3)
# time.sleep(3)

# for drone in allDrones:   
#     drone.land()

# =======================================================
# main loop in try-except blocks for emergency interruption
try:
    # while not rosClock.isShutdown() and itr < max_itr:
    while itr < max_itr:    
        
        # get ground truth pose from mocap
        pos_curr = []
        ori_curr = []
        for drone in GroundTruth:   
            pos, rot = drone.getPose()
            pos_curr.append(pos)
            ori_curr.append(rot*180/np.pi)


        # hovering at the specified setpoints 
        cmd_pitch_x1 =  drone1PIDx.update(pos_curr[0][0], Ref_1[0], tracking_error=None, current_time=rosClock.time())  
        cmd_roll_y1  = -drone1PIDy.update(pos_curr[0][1], Ref_1[1], tracking_error=None, current_time=rosClock.time())  
        cmd_z1       =  drone1PIDz.update(pos_curr[0][2], Ref_1[2], tracking_error=None, current_time=rosClock.time())
  
        cmd_pitch_x2 =  drone2PIDx.update(pos_curr[1][0], Ref_2[0], tracking_error=None, current_time=rosClock.time())  
        cmd_roll_y2  = -drone2PIDy.update(pos_curr[1][1], Ref_2[1], tracking_error=None, current_time=rosClock.time())  
        cmd_z2       =  drone2PIDz.update(pos_curr[1][2], Ref_2[2], tracking_error=None, current_time=rosClock.time())

        # ============= send control commands ============
        
        allDrones[0].cmdAngleZ(cmd_roll_y1, cmd_pitch_x1, cmd_z1, 0)
        allDrones[1].cmdAngleZ(cmd_roll_y2, cmd_pitch_x2, cmd_z2, 0)

        # ================================================ 
        
        rosClock.sleepForRate(1/Ts)
        itr = itr+1
        
except KeyboardInterrupt:
    
    print('emergency interruption!; aborting all flights ...')
    
    for i in range(2):
        for drone in allDrones:
            drone.emergency()
    
        rosClock.sleepForRate(10)
        
    pass

else:
    print('End of experiment!; all drones are landing ...')
    SWARM.Landing(GroundTruth,allDrones,rosClock)
    # for drone in allDrones:
    #     drone.land()
        
    rosClock.sleep(1)
    for drone in allDrones:   
        drone.disconnect()

