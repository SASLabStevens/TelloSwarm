#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug  5 17:51:22 2021

@author: Mohamad Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

import time


class PID:
    """
    PID controller with a lowpass filter on D term.
    Backward Euler method are used in approximations.
      
    """
    def __init__(self, Kp: float, Kd: float, Ki: float,
                 derivativeFilterFreq: int,
                 # previousTime: None,
                 minOutput: float, maxOutput: float,
                 current_time = None):
        """
        TODO:
        

        Parameters
        ----------
        Kp : float
            DESCRIPTION.
        Kd : float
            DESCRIPTION.
        Ki : float
            DESCRIPTION.
        derivativeFilterFreq : int
            DESCRIPTION.
        previousTime : float
            DESCRIPTION.
        minOutput : TYPE, optional
            DESCRIPTION. The default is None.
        maxOutput : TYPE, optional
            DESCRIPTION. The default is None.
        current_time : TYPE, optional
            DESCRIPTION. The default is None.

        Returns
        -------
        None.

        """
        
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.F = derivativeFilterFreq
        # self.sample_time = sampleTime
        self.Ki = Ki
        self.minOutput = minOutput
        self.maxOutput = maxOutput
        self.previousError = 0.0
        self.previousDterm = 0.0
        self.previousIterm = 0.0
        # self.currentTime = current_time if current_time is not None else time.time()
        # self.previousTime = ros::Time::now()
        # self.previousTime = self.currentTime
        self.previousTime = current_time if current_time is not None else time.time()
        
        
        # self.reset(self)
        
    def reset(self, current_time = None):
        self.previousError = 0.0
        self.previousDterm = 0.0
        self.previousIterm = 0.0
        # self.previousTime = ros::Time::now()
        self.previousTime = current_time if current_time is not None else time.time()


    def update(self, feedback_value=None, target_value=None, tracking_error=None, current_time=None):
        """
        

        Parameters
        ----------
        feedback_value : TYPE
            DESCRIPTION.
        target_value : TYPE
            DESCRIPTION.
        tracking_error : TYPE, optional
            DESCRIPTION. The default is None.
        current_time : TYPE, optional
            DESCRIPTION. The default is None.

        Returns
        -------
        TYPE
            DESCRIPTION.

        """
        
        if tracking_error is None and feedback_value is None:
            raise Exception("ERROR! PID requires either tracking_error OR both feedback_value and target_value as input.")

        if tracking_error is None and target_value is None:
            raise Exception("ERROR! PID requires either tracking_error OR both feedback_value and target_value as input.")

        
        
        # TOO: add if {dt>0 OR Ts} for updating the D term
        # what if feedback_value, target_value are not given
            
            
        currentTime = current_time if current_time is not None else time.time()
        error = (target_value - feedback_value) if tracking_error is None else tracking_error
        dt = currentTime - self.previousTime
        
        Pterm = self.Kp * error
        Dterm = self.previousDterm/(1+dt*self.F) + (
                  self.Kd*self.F*(error - self.previousError))/(1+self.F*dt)
        Iterm = self.previousIterm + self.Ki * dt * error
        Iterm = max(self.minOutput*.4, min(self.maxOutput*.4, Iterm))
        
        # self.currentTime = current_time if current_time is not None else time.time()
        self.previousError = error
        self.previousDterm = Dterm
        self.previousIterm = Iterm
        self.previousTime  = currentTime
        output = Pterm + Dterm + Iterm
        return max(self.minOutput, min(self.maxOutput, output))
        
        

