#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Wed Jul 28 14:56:48 2021

@author: Mohamad Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology


"""


import rospy
import tf

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

import time


class rosClock:
    """
    Description ...
    
    This object (rosClock) was adopted from TimeHelper object in
    https://github.com/USC-ACTLab/crazyswarm
    

    """
    def __init__(self):
        self.rosRate = None
        self.rateHz = None

    def time(self):
        """Returns the current time in seconds."""
        return rospy.Time.now().to_sec()

    def sleep(self, duration):
        """Sleeps for the provided duration in seconds."""
        rospy.sleep(duration)

    def sleepForRate(self, rateHz):
        """Sleeps so that, if called in a loop, executes at specified rate."""
        if self.rosRate is None or self.rateHz != rateHz:
            self.rosRate = rospy.Rate(rateHz)
            self.rateHz = rateHz
        self.rosRate.sleep()

    def isShutdown(self):
        """Returns true if the script should abort, e.g. from Ctrl-C."""
        return rospy.is_shutdown()
    

class MotionCapture:
    """
    
    TODO:
        make the world and child(object) frames configurable.
        
        currently the default structure is consistent only with the 
        vicon_bridge pkg @ https://github.com/ethz-asl/vicon_bridge
        
    """
    
    WORLD_FRAME ='/vicon/world'
    
    
    def __init__(self, OBJECT_FRAME):
        rospy.init_node("GroundTruthPose", anonymous=False, disable_signals=True)
        self.tflistener = tf.TransformListener()
        self.OBJECT_FRAME = '/vicon'+str('/')+str(OBJECT_FRAME)+str('/')+str(OBJECT_FRAME)
        


    def getPose(self, mode:str = None):
        """
        # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html

        Parameters
        ----------
        mode : str, optional
            DESCRIPTION. The default is None.
            mode == "quaternion", "euler", "None"

        Returns
        -------
        TYPE
            DESCRIPTION.
            euler angles are in radians
            

        """
        self.tflistener.waitForTransform( MotionCapture.WORLD_FRAME, self.OBJECT_FRAME, rospy.Time(0), rospy.Duration(.03))
        position, quaternion = self.tflistener.lookupTransform(MotionCapture.WORLD_FRAME, self.OBJECT_FRAME, rospy.Time(0))
        
        if mode == 'quaternion':
            rotation = quaternion
        else:        
            (Rx, Ry, Rz) = euler_from_quaternion(quaternion)
            rotation = (Rx, Ry, Rz)
            
            # not necessarily:
            # (Rx, Ry, Rz) != (roll, pitch, yaw) 
        
        # in list format
        # TODO:
        
        # specify the units of measurement
        
        # print(position) 
        # np.array(position)       
        return np.array(position), np.array(rotation)
    
        

if __name__ == '__main__':
    # rospy.init_node("GroundTruthPose", anonymous=False)
    try:
        OBJECT_FRAME = input("Enter the object name defined in VICON Tracker: ") 
        print("Hello", OBJECT_FRAME + "!")
        Drone_GroundTruthPose = MotionCapture(str(OBJECT_FRAME))
        
        rate = rospy.Rate(100) # in Hz
        while not rospy.is_shutdown():
                    pos, rot = Drone_GroundTruthPose.getPose()
                    print("The Position and Rotation are:")
                    print('position:', pos)
                    print('orientation:', rot)
                    print('Press ctrl+C to stop...')
                    rate.sleep() 
                    
        # OBJECT_FRAME = input("Enter the object name defined in VICON Tracker: ") 
        # print("Hello", OBJECT_FRAME + "!")
        # print("The Position and Rotation are:")
        # Drone_GroundTruthPose = MotionCapture(str(OBJECT_FRAME))
        # pos, rot = Drone_GroundTruthPose.getPose()
        # print(pos)
        # print(rot)
    # except rospy.ROSInterruptException:  pass
    except KeyboardInterrupt:  pass

