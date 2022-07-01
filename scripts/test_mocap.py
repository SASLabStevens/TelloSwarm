#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 28 17:15:47 2021

@author: Mohamad Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

#import sys
#from os import path
#sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )
#from scripts.mocap import  MotionCapture as GroundTruth

from mocap import  MotionCapture as GroundTruth


import time

import math
    
OBJECT_Name = 'Tello4' # the object name defined in VICON Tracker

# first method:
    
# pos, rot = GroundTruth(OBJECT_Name).getPose()
# print(pos)


# second method:

Drone_GroundTruthPose = GroundTruth(OBJECT_Name)

i = 0
for i in range(3):
    
    pos, rot = Drone_GroundTruthPose.getPose(mode='euler')
    print('pos:', pos)
    print('rot:', rot)
    
    time.sleep(1) 
    i = i+1



    
    
    
