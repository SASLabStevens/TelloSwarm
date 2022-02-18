#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 16 15:47:30 2021

@author: Mohamad Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology


"""

from TelloServer import SWARM

from PID import PID
from Observers import LuenbergerObserver
from scipy.io import loadmat, savemat

import numpy as np

import math 

"""
wlx9cefd5fae98b  VICON network 2G; 
wlxa09f10beb890  VICON network 5G; 

"""
"""
wlx9cefd5fb4bdd 192.168.10.51 Tello 1 (Tello-632C18) 5
wlx9cefd5faea28 192.168.100.52 Tello 2 (Tello-6272DB) 2
wlx9cefd5fae83e 192.168.10.63 Tello 3 (Tello-632C69)  wlxa09f10bef866
wlx9cefd5fae837 192.168.10.54 Tello 4 (Tello-632C68) 
wlx9cefd5fb6f40 192.168.10.55 Tello 5 (Tello-632C38)  
wlx9cefd5faeXXX 192.168.10.66 Tello 6 (Tello-XXXXXX) x wlxa09f10b9f7fc
wlx9cefd5faec05 192.168.10.57 Tello 7 (Tello-ED72E6) usb 1b
wlx9cefd5fb6d70 192.168.10.58 Tello 8 (Tello-9F8FA2) usb 2b 

"""

wifi_interfaces = ["wlx9cefd5fae837",
                   "wlx9cefd5faea28",
                   "wlx9cefd5fb4bdd",
                   "wlx9cefd5fb6d70",
                   "wlx9cefd5fb6f40"
                   ]

droneslist = ['Tello4',
              'Tello7',
              'Tello10',
              'Tello8',
              'Tello6'
             ]



swarm = SWARM(wifi_interfaces, droneslist)
allDrones = swarm.allDrones
rosClock = swarm.rosClock()

GroundTruth = swarm.MotionCaptureGroundTruth()

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
                  derivativeFilterFreq=Dterm_filter,
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
                  derivativeFilterFreq=Dterm_filter,
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

drone4PIDx = PID(Kp= K_px, Kd= K_dx, Ki= K_ix,
                  derivativeFilterFreq=Dterm_filter,
                  minOutput = -MAX_PID, 
                  maxOutput = MAX_PID,
                  current_time = rosClock.time())   
    

drone4PIDy = PID(Kp= K_py, Kd= K_dy, Ki= K_iy,
                  derivativeFilterFreq = Dterm_filter,
                  minOutput = -MAX_PID, 
                  maxOutput = MAX_PID,
                  current_time = rosClock.time()) 

drone4PIDz = PID(Kp= K_pz, Kd= K_dz, Ki= K_iz,
                  derivativeFilterFreq = Dterm_filter,
                  minOutput = -MAX_PID, 
                  maxOutput = MAX_PID,
                  current_time = rosClock.time()) 

# =======================================================

drone5PIDx = PID(Kp= K_px, Kd= K_dx, Ki= K_ix,
                  derivativeFilterFreq=Dterm_filter,
                  minOutput = -MAX_PID, 
                  maxOutput = MAX_PID,
                  current_time = rosClock.time())   
    

drone5PIDy = PID(Kp= K_py, Kd= K_dy, Ki= K_iy,
                  derivativeFilterFreq = Dterm_filter,
                  minOutput = -MAX_PID, 
                  maxOutput = MAX_PID,
                  current_time = rosClock.time()) 

drone5PIDz = PID(Kp= K_pz, Kd= K_dz, Ki= K_iz,
                  derivativeFilterFreq = Dterm_filter,
                  minOutput = -MAX_PID, 
                  maxOutput = MAX_PID,
                  current_time = rosClock.time()) 



# =======================================================
#  main loop's configuration

itr = 0

itr_hover = 500+2
duration_itr = 700
itr_landing = itr_hover+duration_itr
max_itr = itr_hover + duration_itr + 250

Ts = 0.02 # sample time for the main loop

tstNum = 'ZDA_sw13'

# preallocate the system states, inputs, and outputs/measurements

numAgents = len(droneslist)

# X = [x1,y1,z1, ... ,xN,yN,zN,  vx1,vy1,vz1, ... ,vxN,vyN,vzN]  # system states
X = np.zeros((6*numAgents,max_itr))


# ctrl_cmd = [roll_1, pitch_1, Vz_1, ... , roll_N, pitch_N, Vz_N]
ctrl_cmd = np.zeros((3*numAgents,max_itr))

# orien = [Rotx_1, Roty_1, Rotz_1, ... , Rotx_N, Roty_N, Rotz_N]
orien = np.zeros((3*numAgents,max_itr))

# Ref = np.zeros((3*numAgents,max_itr))

# ==================================
# Laplacian matrix


Lap1 = np.array([[3,  0, -1, -1, -1],
                 [0,  2, -1,  0,  -1],
                 [-1,-1,  3,  0, -1],
                 [-1, 0,  0,  2,  -1],
                 [-1,-1, -1, -1,  4]]);

# remove link (1,5)
Lap2 = np.array([[2,  0, -1, -1,  0],
                 [0,  2, -1,  0,  -1],
                 [-1,-1,  3,  0, -1],
                 [-1, 0,  0,  2,  -1],
                 [ 0,-1, -1, -1,  3]]);

# remove link (4,5)
Lap3 = np.array([[3,  0, -1, -1, -1],
                 [0,  2, -1,  0,  -1],
                 [-1,-1,  3,  0, -1],
                 [-1, 0,  0,  1,  0],
                 [-1,-1, -1,  0,  3]]);

# add link (1,2) 
Lap4 = np.array([[ 4, -1, -1, -1, -1],
                 [-1,  3, -1,  0,  -1],
                 [-1,-1,  3,  0, -1],
                 [-1, 0,  0,  2,  -1],
                 [-1,-1, -1, -1,  4]]);


Xstar = np.array([[-1, -0.5, 0, 0.5, 1]]).T
Ystar = np.array([[1.4, 0.7, 0, 0.7, 1.4]]).T


W = 0*np.diag([0,0,0,0,0]) # leaders are drones={3}

g = 9.81 # gravity


gamma = 7.2828 
alpha = 1.63863
    
gammaY = gamma
alphaY = alpha


# filter frequency; used in calculating the linear velocity from mocap

Dfrq = 5  
pos_dim = numAgents*3
        

# hovering setpoints in 3D: [x, y, z]
Ref_1 = [-0.95, 1.5, 0.6] 
Ref_2 = [-0.95, 0.1, 0.6]
Ref_3 = [-0.15, +0.7, 0.6]
Ref_4 = [+0.75, 0.1, 0.6]
Ref_5 = [+0.75, 1.5, 0.6]


# ===============  Cent. Obs.  ===================

Ax_1 = np.bmat([ [np.zeros((len(Lap1),len(Lap1))) , np.eye(len(Lap1))] , [-alpha*Lap1, -gamma* np.eye(len(Lap1))]])
Ax_2 = np.bmat([ [np.zeros((len(Lap2),len(Lap2))) , np.eye(len(Lap2))] , [-alpha*Lap2, -gamma* np.eye(len(Lap2))]])
Ax_3 = np.bmat([ [np.zeros((len(Lap3),len(Lap3))) , np.eye(len(Lap3))] , [-alpha*Lap3, -gamma* np.eye(len(Lap3))]])
Ax_4 = np.bmat([ [np.zeros((len(Lap4),len(Lap4))) , np.eye(len(Lap4))] , [-alpha*Lap4, -gamma* np.eye(len(Lap4))]])

Ay_1 = np.bmat([ [np.zeros((len(Lap1),len(Lap1))) , np.eye(len(Lap1))] , [-alphaY*Lap1, -gammaY* np.eye(len(Lap1))]])
Ay_2 = np.bmat([ [np.zeros((len(Lap2),len(Lap2))) , np.eye(len(Lap2))] , [-alphaY*Lap2, -gammaY* np.eye(len(Lap2))]])
Ay_3 = np.bmat([ [np.zeros((len(Lap3),len(Lap3))) , np.eye(len(Lap3))] , [-alphaY*Lap3, -gammaY* np.eye(len(Lap3))]])
Ay_4 = np.bmat([ [np.zeros((len(Lap4),len(Lap4))) , np.eye(len(Lap4))] , [-alphaY*Lap4, -gammaY* np.eye(len(Lap4))]])

A_CentObs  = Ax_1
A_CentObs_y = Ay_1
C_CentObs  = np.zeros((2,len(A_CentObs)))
C_CentObs[0,2] = 1 # agent 3's position
C_CentObs[1,4] = 1 # agent 5's position

# C_CentObs[0,len(Lap1)+5-1] = 1 # agent 5's velocity

CentObsGain = loadmat('CentObs_Hmatrix', squeeze_me=False)
H_CentObs = (CentObsGain['H'])

localObsGain = loadmat('localObs_Hmatrix', squeeze_me=False)
H_locObs_1 = (localObsGain['H_l1'])
H_locObs_3 = (localObsGain['H_l3'])

# preallocate the observer states and outputs

# x-direction
Xhat_CentObs_x = np.zeros((len(A_CentObs),max_itr))
Yhat_CentObs_x = np.zeros((np.size(C_CentObs,0),max_itr))
res_CentObs_x =  np.zeros((np.size(C_CentObs,0),max_itr))

# y-direction
Xhat_CentObs_y = np.zeros((len(A_CentObs),max_itr))
Yhat_CentObs_y = np.zeros((np.size(C_CentObs,0),max_itr))
res_CentObs_y =  np.zeros((np.size(C_CentObs,0),max_itr))
   
# set the observer's IC
        
IC_Centobs = np.zeros((len(A_CentObs),1))


Pxstar = np.bmat([[Xstar],[np.zeros(Xstar.shape)]])
Pystar = np.bmat([[Ystar],[np.zeros(Ystar.shape)]])


# initialize the observers
        
GlobalObs_x = LuenbergerObserver(A_CentObs,-A_CentObs,C_CentObs,H_CentObs,IC_Centobs,Ts)
GlobalObs_y = LuenbergerObserver(A_CentObs_y,-A_CentObs_y,C_CentObs,H_CentObs,IC_Centobs,Ts)

# ===============  Local. Obs.  ===================

A_locObs = Ax_1
A_locObs_y = Ay_1

C_locObs_1 = np.zeros((5,len(A_locObs)))
# agents 1,3,4,5's position
C_locObs_1[0,0] = 1
C_locObs_1[1,2] = 1
C_locObs_1[2,3] = 1
C_locObs_1[3,4] = 1
C_locObs_1[4,len(Lap1)+1-1] = 1 # agent 1's velocity

C_locObs_3 = np.zeros((5,len(A_locObs)))
# agents 1,2,3,5's position
C_locObs_3[0,0] = 1
C_locObs_3[1,1] = 1
C_locObs_3[2,2] = 1
C_locObs_3[3,4] = 1
C_locObs_3[4,len(Lap1)+3-1] = 1 # agent 3's velocity

# preallocate the local observers' states and outputs

# x-direction
Xhat_locObs_1_x = np.zeros((len(A_locObs),max_itr))
Yhat_locObs_1_x = np.zeros((np.size(C_locObs_1,0),max_itr))
res_locObs_1_x =  np.zeros((np.size(C_locObs_1,0),max_itr))

Xhat_locObs_3_x = np.zeros((len(A_locObs),max_itr))
Yhat_locObs_3_x = np.zeros((np.size(C_locObs_3,0),max_itr))
res_locObs_3_x =  np.zeros((np.size(C_locObs_3,0),max_itr))

# y-direction
Xhat_locObs_1_y = np.zeros((len(A_locObs_y),max_itr))
Yhat_locObs_1_y = np.zeros((np.size(C_locObs_1,0),max_itr))
res_locObs_1_y =  np.zeros((np.size(C_locObs_1,0),max_itr))

Xhat_locObs_3_y = np.zeros((len(A_locObs_y),max_itr))
Yhat_locObs_3_y = np.zeros((np.size(C_locObs_3,0),max_itr))
res_locObs_3_y =  np.zeros((np.size(C_locObs_3,0),max_itr))

# set the observer's IC
        
IC_localobs = np.zeros((len(A_locObs),1))
       
# initialize the observers
        
localObs_1_x = LuenbergerObserver(A_locObs,-A_locObs,C_locObs_1,H_locObs_1,IC_localobs,Ts)
localObs_1_y = LuenbergerObserver(A_locObs_y,-A_locObs_y,C_locObs_1,H_locObs_1,IC_localobs,Ts)

localObs_3_x = LuenbergerObserver(A_locObs,-A_locObs,C_locObs_3,H_locObs_3,IC_localobs,Ts)
localObs_3_y = LuenbergerObserver(A_locObs_y,-A_locObs_y,C_locObs_3,H_locObs_3,IC_localobs,Ts)


# =======================================================
rosClock.sleep(10)

for drone in allDrones:   
    drone.connect()
  
for drone in allDrones:   
    drone.takeoff()
       

rosClock.sleep(.1)

# %% main code

# try:
#     while not rosClock.isShutdown():
#         # break (the main body of code)
# except KeyboardInter:  pass

NullHypothesis = True
log_file_ZDA = open("log_file_ZDA.txt","w")

try:
    # while not rosClock.isShutdown() and itr < max_itr:
    while itr < max_itr:    
        
        # =============== update pose and velocity data ====================
    
        
        # initializaton
        if itr == 0 or itr == 1:

            pos_curr = []
            for drone in GroundTruth:   
                pos = drone.getPose()[0]
                pos_curr.append(pos)
            
            # ====  store data:   
            # position data   
            X[0:3,itr] = pos_curr[0]
            X[3:6,itr] = pos_curr[1]
            X[6:9,itr] = pos_curr[2]
            X[9:12,itr] = pos_curr[3]
            X[12:15,itr] = pos_curr[4]
           

            if itr == 0:
                x_filter_previous_2 = X[0:pos_dim,itr]
            if itr == 1:    
                x_filter_previous = X[0:pos_dim,itr]    
            
            # velocity data
            X[pos_dim:2*pos_dim,itr] = np.zeros((1,pos_dim))
            
         
        # other iterations        
        else:
                
            pos_curr = []
            ori_curr = []
            for drone in GroundTruth:   
                pos, rot = drone.getPose()
                pos_curr.append(pos)
                ori_curr.append(rot*180/math.pi)
                       

            # ====  store data:   
            # pose data
            
            X[0:3,itr] = pos_curr[0]
            X[3:6,itr] = pos_curr[1]
            X[6:9,itr] = pos_curr[2]
            X[9:12,itr] = pos_curr[3]
            X[12:15,itr] = pos_curr[4]
            
            orien[0:3,itr] = ori_curr[0]
            orien[3:6,itr] = ori_curr[1]
            orien[6:9,itr] = ori_curr[2]
            orien[9:12,itr] = ori_curr[3]
            orien[12:15,itr] = ori_curr[4]
        
            # velocity data
            
            # calculate velocity using filtered pose data
            
            """
              velocity = (x_filter - x_filter_previous) / Ts
              x_filter =  x_filter_previous/(1+Ts*filter_freq) + x_current*Ts*filter_freq/(1+Ts*filter_freq) 
            
            """
            
            X[pos_dim:2*pos_dim,itr] =  (x_filter_previous/(Ts+Ts*Ts*Dfrq) + X[0:pos_dim,itr]*Dfrq/(1+Ts*Dfrq) 
                                         - x_filter_previous_2/(Ts+Ts*Ts*Dfrq) - X[0:pos_dim,itr-1]*Dfrq/(1+Ts*Dfrq) )
            
            x_filter_previous_2 = x_filter_previous_2/(1+Ts*Dfrq) + X[0:pos_dim,itr-1]*Ts*Dfrq/(1+Ts*Dfrq) 
            x_filter_previous = x_filter_previous/(1+Ts*Dfrq) + X[0:pos_dim,itr]*Ts*Dfrq/(1+Ts*Dfrq) 
            
            
            # calculate velocity using unfiltered pose data
            # X[pos_dim:2*pos_dim,itr] = (X[0:pos_dim,itr] - X[0:pos_dim,itr-1])/Ts        
            
           
            pos_state_x = np.array([[pos_curr[0][0], 
                                     pos_curr[1][0],
                                     pos_curr[2][0],
                                     pos_curr[3][0],
                                     pos_curr[4][0]]]).T

             
            vel_state_x = np.array([[X[pos_dim,itr],
                                     X[pos_dim+3,itr],
                                     X[pos_dim+6,itr],
                                     X[pos_dim+9,itr],
                                     X[pos_dim+12,itr]]]).T

            pos_state_y = np.array([[pos_curr[0][1], 
                                     pos_curr[1][1],
                                     pos_curr[2][1],
                                     pos_curr[3][1],
                                     pos_curr[4][1]]]).T

             
            vel_state_y = np.array([[X[pos_dim+1,itr],
                                     X[pos_dim+3+1,itr],
                                     X[pos_dim+6+1,itr],
                                     X[pos_dim+9+1,itr],
                                     X[pos_dim+12+1,itr]]]).T
            
                   

        # =========== update controllers and observers =================
        
        if itr >= itr_hover and itr < itr_landing:
            
              # ===============  update observers' states ======================= 

              Y_localObs_current_1_x = C_locObs_1 @ np.bmat([[pos_state_x],[vel_state_x]])
              Xhat_locObs_1_x[:,itr+1:itr+2], Yhat_locObs_1_x[:,itr:itr+1], res_locObs_1_x[:,itr:itr+1] = localObs_1_x.Update(Pxstar,Y_localObs_current_1_x)               
              
              Y_localObs_current_1_y = C_locObs_1 @ np.bmat([[pos_state_y],[vel_state_y]])
              Xhat_locObs_1_y[:,itr+1:itr+2], Yhat_locObs_1_y[:,itr:itr+1], res_locObs_1_y[:,itr:itr+1] = localObs_1_y.Update(Pystar,Y_localObs_current_1_y) 

              Y_localObs_current_3_x = C_locObs_3 @ np.bmat([[pos_state_x],[vel_state_x]])
              Xhat_locObs_3_x[:,itr+1:itr+2], Yhat_locObs_3_x[:,itr:itr+1], res_locObs_3_x[:,itr:itr+1] = localObs_3_x.Update(Pxstar,Y_localObs_current_3_x)               
              
              Y_localObs_current_3_y = C_locObs_3 @ np.bmat([[pos_state_y],[vel_state_y]])
              Xhat_locObs_3_y[:,itr+1:itr+2], Yhat_locObs_3_y[:,itr:itr+1], res_locObs_3_y[:,itr:itr+1] = localObs_3_y.Update(Pystar,Y_localObs_current_3_y) 



              Y_CentObs_current_x = C_CentObs @ np.bmat([[pos_state_x],[vel_state_x]])
              Xhat_CentObs_x[:,itr+1:itr+2], Yhat_CentObs_x[:,itr:itr+1], res_CentObs_x[:,itr:itr+1] = GlobalObs_x.Update(Pxstar,Y_CentObs_current_x)               

              Y_CentObs_current_y = C_CentObs @ np.bmat([[pos_state_y],[vel_state_y]])
              Xhat_CentObs_y[:,itr+1:itr+2], Yhat_CentObs_y[:,itr:itr+1], res_CentObs_y[:,itr:itr+1] = GlobalObs_y.Update(Pystar,Y_CentObs_current_y)               


              # ==================== update controllers' output =============== 
              
              # thresholds
              
              threshold_0 =  3*np.exp(-2.2*(itr-itr_hover)*Ts) + 0.015
              threshold_local = 3*np.exp(-2.5*(itr-itr_hover)*Ts) + 0.02
              threshold_local_y = 3*np.exp(-2*(itr-itr_hover)*Ts) + 0.02
              
              # stack the local residuals of position estimation
              
              res_locObs_x = np.bmat([[res_locObs_1_x[0:4,itr:itr+1]],[res_locObs_3_x[0:4,itr:itr+1]]])
              res_locObs_y = np.bmat([[res_locObs_1_y[0:4,itr:itr+1]],[res_locObs_3_y[0:4,itr:itr+1]]])
              
              if NullHypothesis is True and all(res_locObs_x <= threshold_local) and all(res_locObs_y <= threshold_local_y):                 
                  
                  # null Hypothesis is valid  
                  Lap = Lap1
                  
              elif NullHypothesis is True and (any(res_locObs_x > threshold_local) or any(res_locObs_y > threshold_local_y)):  
                      
                  # reject the Null Hypothesis <==> accept the Alternative Hypothesis
                  NullHypothesis = False
                  # update the communication topology
                  Lap = Lap3
                  # local observers' cooperation:
                  #    update local observers' matrices     

                  localObs_1_x.A = Ax_3
                  localObs_3_x.A = Ax_3
                  
                  localObs_1_x.B = -Ax_3
                  localObs_3_x.B = -Ax_3
                  
                  localObs_1_y.A = Ay_3
                  localObs_3_y.A = Ay_3
                  
                  localObs_1_y.B = -Ay_3
                  localObs_3_y.B = -Ay_3
                  

                  print("local detection at itr = {} \n".format(itr))
                  log_file_ZDA.write("local detection at itr = {} \n".format(itr))
                                    
                  # back-up controllers for hovering
                  # itr = itr_landing
                  
              else:  
                 # the Alternative Hypothesis has been accepted
                 Lap = Lap3
                 # print("safe mode operation at itr = {} \n".format(itr))
                 log_file_ZDA.write("safe mode operation at itr = {} \n".format(itr))
                  
              if  NullHypothesis is False and (any(res_CentObs_x[:,itr:itr+1] > threshold_0) or any(res_CentObs_y[:,itr:itr+1] > threshold_0)):  
                   # back-up controllers for hovering      
                   print("back-up plan was activated by the control center at itr = {} \n".format(itr))
                   log_file_ZDA.write("back-up plan was activated by the control center at itr = {} \n".format(itr))
                   itr = itr_landing
                  
              Ux = - (alpha*Lap+W)@(pos_state_x-Xstar) - gamma*vel_state_x 
              Uy = - (alphaY*Lap+W)@(pos_state_y-Ystar) - gammaY*vel_state_y
              
              # ZDA 
              
              epsi = 0.7
              lambda_o = 0.5
              Ua0 = np.array([[-2.3409*epsi, 10.24094286*epsi, -2.3409*epsi]]).T
               
              
              # epsi = 0.1
              # lambda_o = 1
              # Ua0 = np.array([[-2.3409*epsi, 16.5144*epsi, -2.3409*epsi]]).T
              
              cmd_pitch_x1 = Ux[0,0]/g*180/math.pi*10 + Ua0[0,0]*np.exp(lambda_o*(itr-itr_hover)*Ts)
              cmd_pitch_x2 = Ux[1,0]/g*180/math.pi*10 
              cmd_pitch_x3 = Ux[2,0]/g*180/math.pi*10
              cmd_pitch_x4 = Ux[3,0]/g*180/math.pi*10 + Ua0[1,0]*np.exp(lambda_o*(itr-itr_hover)*Ts)
              cmd_pitch_x5 = Ux[4,0]/g*180/math.pi*10 + Ua0[2,0]*np.exp(lambda_o*(itr-itr_hover)*Ts)
              
              cmd_roll_y1 = -Uy[0,0]/g*180/math.pi*10 + .7*Ua0[0,0]*np.exp(lambda_o*(itr-itr_hover)*Ts)
              cmd_roll_y2 = -Uy[1,0]/g*180/math.pi*10 
              cmd_roll_y3 = -Uy[2,0]/g*180/math.pi*10 
              cmd_roll_y4 = -Uy[3,0]/g*180/math.pi*10 + .7*Ua0[1,0]*np.exp(lambda_o*(itr-itr_hover)*Ts)
              cmd_roll_y5 = -Uy[4,0]/g*180/math.pi*10 + .7*Ua0[2,0]*np.exp(lambda_o*(itr-itr_hover)*Ts)
              
                            
        else:
            cmd_pitch_x1 = drone1PIDx.update(pos_curr[0][0], Ref_1[0], tracking_error=None, current_time=rosClock.time())  
            cmd_pitch_x2 = drone2PIDx.update(pos_curr[1][0], Ref_2[0], tracking_error=None, current_time=rosClock.time())
            cmd_pitch_x3 = drone3PIDx.update(pos_curr[2][0], Ref_3[0], tracking_error=None, current_time=rosClock.time())
            cmd_pitch_x4 = drone4PIDx.update(pos_curr[3][0], Ref_4[0], tracking_error=None, current_time=rosClock.time())
            cmd_pitch_x5 = drone5PIDx.update(pos_curr[4][0], Ref_5[0], tracking_error=None, current_time=rosClock.time())
            
            cmd_roll_y1 = -drone1PIDy.update(pos_curr[0][1], Ref_1[1], tracking_error=None, current_time=rosClock.time())  
            cmd_roll_y2 = -drone2PIDy.update(pos_curr[1][1], Ref_2[1], tracking_error=None, current_time=rosClock.time())
            cmd_roll_y3 = -drone3PIDy.update(pos_curr[2][1], Ref_3[1], tracking_error=None, current_time=rosClock.time())
            cmd_roll_y4 = -drone4PIDy.update(pos_curr[3][1], Ref_4[1], tracking_error=None, current_time=rosClock.time())
            cmd_roll_y5 = -drone5PIDy.update(pos_curr[4][1], Ref_5[1], tracking_error=None, current_time=rosClock.time())

                 
        
        cmd_z1 = drone1PIDz.update(pos_curr[0][2], Ref_1[2], tracking_error=None, current_time=rosClock.time())
        cmd_z2 = drone2PIDz.update(pos_curr[1][2], Ref_2[2], tracking_error=None, current_time=rosClock.time())
        cmd_z3 = drone3PIDz.update(pos_curr[2][2], Ref_3[2], tracking_error=None, current_time=rosClock.time())
        cmd_z4 = drone4PIDz.update(pos_curr[3][2], Ref_4[2], tracking_error=None, current_time=rosClock.time())
        cmd_z5 = drone5PIDz.update(pos_curr[4][2], Ref_5[2], tracking_error=None, current_time=rosClock.time())

     
        
        # store data
       
        ctrl_cmd[0:3,itr] = [cmd_pitch_x1, cmd_roll_y1, cmd_z1]
        ctrl_cmd[3:6,itr] = [cmd_pitch_x2, cmd_roll_y2, cmd_z2]
        ctrl_cmd[6:9,itr] = [cmd_pitch_x3, cmd_roll_y3, cmd_z3]
        ctrl_cmd[9:12,itr] = [cmd_pitch_x4, cmd_roll_y4, cmd_z4]
        ctrl_cmd[12:15,itr] = [cmd_pitch_x5, cmd_roll_y5, cmd_z5]


        
        # ============= send control commands ============
        
        allDrones[0].cmdAngleVelocityZ(cmd_roll_y1, cmd_pitch_x1, cmd_z1, 0)
        allDrones[1].cmdAngleVelocityZ(cmd_roll_y2, cmd_pitch_x2, cmd_z2, 0)
        allDrones[2].cmdAngleVelocityZ(cmd_roll_y3, cmd_pitch_x3, cmd_z3, 0)
        allDrones[3].cmdAngleVelocityZ(cmd_roll_y4, cmd_pitch_x4, cmd_z4, 0)
        allDrones[4].cmdAngleVelocityZ(cmd_roll_y5, cmd_pitch_x5, cmd_z5, 0)

         
        # ================================================ 
        
        # rosClock.sleep(Ts)
        rosClock.sleepForRate(1/Ts)
        itr = itr+1
        
                
except KeyboardInterrupt:
    
    print('emergency interruption!; aborting all flights ...')
    
    for i in range(2):
        for drone in allDrones:
            drone.emergency()

        rosClock.sleepForRate(10)
        
        
    with open("X_{}.npy".format(tstNum), 'wb') as savedata:    
        np.save(savedata, X)
    
    # np.save("Ref_"+str(tstNum), Ref) 
    np.save("orien_"+str(tstNum), orien) 
    np.save("ctrl_cmd_"+str(tstNum), ctrl_cmd)

    with open("CentObs_{}.npy".format(tstNum), 'wb') as savedata:    
        np.save(savedata, Xhat_CentObs_x)
        np.save(savedata, Yhat_CentObs_x)
        np.save(savedata, res_CentObs_x)
        np.save(savedata, Xhat_CentObs_y)
        np.save(savedata, Yhat_CentObs_y)
        np.save(savedata, res_CentObs_y)
        
    with open("localObs_{}.npy".format(tstNum), 'wb') as savedata: 
        np.save(savedata, Xhat_locObs_1_x)
        np.save(savedata, Yhat_locObs_1_x)
        np.save(savedata, res_locObs_1_x)
        np.save(savedata, Xhat_locObs_1_y)
        np.save(savedata, Yhat_locObs_1_y)
        np.save(savedata, res_locObs_1_y)
        np.save(savedata, Xhat_locObs_3_x)
        np.save(savedata, Yhat_locObs_3_x)
        np.save(savedata, res_locObs_3_x)
        np.save(savedata, Xhat_locObs_3_y)
        np.save(savedata, Yhat_locObs_3_y)
        np.save(savedata, res_locObs_3_y)
    
    log_file_ZDA.close()
    
    pass

else:
    print('End of experiment!; all drones are landing ...')
    # Landing(GroundTruth, allDrones)
    for drone in allDrones:
        drone.land()

   
    rosClock.sleep(1)
    for drone in allDrones:   
        drone.Disconnect()

    
    with open("X_{}.npy".format(tstNum), 'wb') as savedata:    
        np.save(savedata, X)

    # np.save("Ref_"+str(tstNum), Ref)
    np.save("orien_"+str(tstNum), orien) 
    np.save("ctrl_cmd_"+str(tstNum), ctrl_cmd)
    
    with open("CentObs_{}.npy".format(tstNum), 'wb') as savedata:    
        np.save(savedata, Xhat_CentObs_x)
        np.save(savedata, Yhat_CentObs_x)
        np.save(savedata, res_CentObs_x)
        np.save(savedata, Xhat_CentObs_y)
        np.save(savedata, Yhat_CentObs_y)
        np.save(savedata, res_CentObs_y)
        
    with open("localObs_{}.npy".format(tstNum), 'wb') as savedata: 
        np.save(savedata, Xhat_locObs_1_x)
        np.save(savedata, Yhat_locObs_1_x)
        np.save(savedata, res_locObs_1_x)
        np.save(savedata, Xhat_locObs_1_y)
        np.save(savedata, Yhat_locObs_1_y)
        np.save(savedata, res_locObs_1_y)
        np.save(savedata, Xhat_locObs_3_x)
        np.save(savedata, Yhat_locObs_3_x)
        np.save(savedata, res_locObs_3_x)
        np.save(savedata, Xhat_locObs_3_y)
        np.save(savedata, Yhat_locObs_3_y)
        np.save(savedata, res_locObs_3_y)   
    
    log_file_ZDA.close()



