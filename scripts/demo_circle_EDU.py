#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 16 15:47:30 2021

@author: Rayan Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology


"""

from TelloServerEDU import SWARM

from PID import PID
from Observers import LuenbergerObserver
from scipy.io import loadmat, savemat

import numpy as np

# import math, time 

"""
WI-FI interfaces  |        network 
-----------------   ---------------------------------- 
wlx9cefd5fae98b   :  VICON network 2G
wlxa09f10beb890   :  VICON network 5G
______________________________________________________

Tello WiFi name   | client's name and IP in the router
----------------   ----------------------------------

Tello-CBF2C8      :    EDU_1     172.16.0.11
Tello-CBF2C9      :    EDU_2     172.16.0.12
Tello-CBF267      :    EDU_3     172.16.0.13
Tello-CBF2CB      :    EDU_4     172.16.0.xx 
Tello-CBF1B9      :    EDU_5     172.16.0.xx
Tello-CBF2C1      :    EDU_6     172.16.0.xx

"""

DronesDict = {'EDU_1' : '172.16.0.11',
              'EDU_2' : '172.16.0.12',
              'EDU_3' : '172.16.0.13'
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

K_px = 330  # 400
K_dx = 330  # 400
K_ix = 10  # 130

K_py = 330  # 400
K_dy = 330  # 400
K_iy = 10  # 130

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

drone3PIDx = PID(Kp= K_px, Kd= K_dx, Ki= K_ix,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time())   
    

drone3PIDy = PID(Kp= K_py, Kd= K_dy, Ki= K_iy,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time()) 

drone3PIDz = PID(Kp= K_pz, Kd= K_dz, Ki= K_iz,
                 derivativeFilterFreq = Dterm_filter,
                 minOutput = -MAX_PID, 
                 maxOutput = MAX_PID,
                 current_time = rosClock.time()) 

# =======================================================
#  main loop's configuration

itr = 0
max_itr = 1850
max_itr_ = 1650


Ts = 0.02 # sample time for the main loop


# hovering setpoints in 3D: [x, y, z]
Ref_1 = [-0.8, 1.5, 0.75] 
Ref_2 = [-0.8,-1.0, 0.75]
Ref_3 = [+0.2,+0.0, 0.75]
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
frq = 0.1 
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
        if itr<=max_itr_:
            Ref_1x = np.cos(2*np.pi *frq * itr*Ts)
            Ref_1y = np.sin(2*np.pi *frq * itr*Ts)
            Ref_2x = np.cos(2*np.pi *frq * itr*Ts + 2*np.pi/3) 
            Ref_2y = np.sin(2*np.pi *frq * itr*Ts + 2*np.pi/3) 
            Ref_3x = np.cos(2*np.pi *frq * itr*Ts + 4*np.pi/3) 
            Ref_3y = np.sin(2*np.pi *frq * itr*Ts + 4*np.pi/3) 

        elif itr==max_itr_+1:
            Ref_1x_ = pos_curr[0][0] 
            Ref_1y_ = pos_curr[0][1] 
            Ref_2x_ = pos_curr[1][0] 
            Ref_2y_ = pos_curr[1][1]
            Ref_3x_ = pos_curr[2][0]
            Ref_3y_ = pos_curr[2][1] 
        else:
            Ref_1x = Ref_1x_ 
            Ref_1y = Ref_1y_
            Ref_2x = Ref_2x_ 
            Ref_2y = Ref_2y_
            Ref_3x = Ref_3x_
            Ref_3y = Ref_3y_



        cmd_pitch_x1 =  drone1PIDx.update(pos_curr[0][0], Ref_1x, tracking_error=None, current_time=rosClock.time())  
        cmd_roll_y1  = -drone1PIDy.update(pos_curr[0][1], Ref_1y, tracking_error=None, current_time=rosClock.time())  
        cmd_z1       =  drone1PIDz.update(pos_curr[0][2], Ref_1[2], tracking_error=None, current_time=rosClock.time())
  
        cmd_pitch_x2 =  drone2PIDx.update(pos_curr[1][0], Ref_2x, tracking_error=None, current_time=rosClock.time())  
        cmd_roll_y2  = -drone2PIDy.update(pos_curr[1][1], Ref_2y, tracking_error=None, current_time=rosClock.time())  
        cmd_z2       =  drone2PIDz.update(pos_curr[1][2], Ref_2[2], tracking_error=None, current_time=rosClock.time())

        cmd_pitch_x3 =  drone3PIDx.update(pos_curr[2][0], Ref_3x, tracking_error=None, current_time=rosClock.time())  
        cmd_roll_y3  = -drone3PIDy.update(pos_curr[2][1], Ref_3y, tracking_error=None, current_time=rosClock.time())  
        cmd_z3       =  drone3PIDz.update(pos_curr[2][2], Ref_3[2], tracking_error=None, current_time=rosClock.time())
        # ============= send control commands ============
        
        allDrones[0].cmdAngleZ(cmd_roll_y1, cmd_pitch_x1, cmd_z1, 0)
        allDrones[1].cmdAngleZ(cmd_roll_y2, cmd_pitch_x2, cmd_z2, 0)
        allDrones[2].cmdAngleZ(cmd_roll_y3, cmd_pitch_x3, cmd_z3, 0)

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
        
    # rosClock.sleep(1)
    # for drone in allDrones:   
    #     drone.disconnect()

