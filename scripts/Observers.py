# -*- coding: utf-8 -*-
"""
Created on Wed May 19 13:55:13 2021

@author: Mohamad Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

import numpy as np
import matplotlib.pyplot as plt

"""
# version 1.0: 

    does not instantiate the observer states Xhat and output Yhat
    
    
""" 
# class LuenbergerObserver():
#   def __init__(self, A, C, H, Ts):
#     # pass
#     self.A = A
#     self.C = C
#     self.H = H
#     self.Ts = Ts
#     self.residual = None
#     self.dim = len(A)
  
#   def Update(self, Xhat, Y):
#       Yhat = np.dot(self.C, Xhat)
#       self.residual = Y - Yhat
#       Xhat_next = (np.eye(self.dim) + self.Ts * self.A).dot(Xhat) + self.Ts * np.dot(self.H,(Y - Yhat))
#       return Xhat_next, Yhat, self.residual

"""
    Example:
        
        
        # initialize the system parameters: 
        
        A  = np.array([[-1,0,0],[0,-2,0],[0,0,-3]])
        C  = np.array([[1, 0, 0],[0,1,0]])
        H  = np.array([[8,0],[0,3],[0,3]])
        IC = np.array([5,4,3])
        Ts = .1
        
        # simulation duration: [0 - 10] sec with Ts = .1 ==> 101 sample 
        
        # real system:
            # Preallocate the system states and outputs
        x = np.zeros((len(A),101))
        y = np.zeros((np.size(C,0),101))

        x[:,0] = IC
        
        for i in range(100):
            x[:,i+1] = (np.eye(len(A))+Ts*A).dot(x[:,i])
            y[:,i] = C.dot(x[:,i])
        
        # ==========================  Observer  =============================
        # 
            # Preallocate the observer states and outputs 
             
        Xhat = np.zeros((len(A),101))
        yhat = np.zeros((np.size(C,0),101))
        res =  np.zeros((np.size(C,0),101))
        
        # (optional: set a non-zero initial condition for the observer:)
        # Xhat[:,:1] = .2*IC.reshape(len(A),1)  
        
        # initialize the observer
        
        obs = LuenbergerObserver(A,C,H,Ts)
    
        # run and update the observer
        
        for i in range(100):
            Xhat[:,i+1], yhat[:,i], res[:,i] = obs.Update(Xhat[:,i], y[:,i]) 
        
        # ==================================================================
        
        t = np.arange(0, 10.1, Ts)
        plt.plot(t, y.T, 'b', t, yhat.T, '--r')
        plt.show()    

"""

#%%      

"""
# version 2.0: 

    instantiates the observer states Xhat, output Yhat, and residual r; 
    also stores their current value
    
    
""" 

class LuenbergerObserver():
  def __init__(self, A, B, C, H, IC, Ts):
      """
      

      Parameters
      ----------
      A : TYPE
          DESCRIPTION.
      C : TYPE
          DESCRIPTION.
      H : TYPE
          DESCRIPTION.
      IC : TYPE
          DESCRIPTION.
      Ts : TYPE
          DESCRIPTION.

      Returns
      -------
      None.

      """
    # pass
      self.A = A
      self.B = B
      self.C = C
      self.H = H
      self.IC = IC
      self.Ts = Ts
      self.residual = None
      self.dim = len(A)
      self.Xhat = IC
      # self.Yhat = np.dot(self.C, self.Xhat)
      self.Yhat = self.C @ self.Xhat
    
  def Update(self, U, Y):
      """
      

      Parameters
      ----------
      U : TYPE
          DESCRIPTION.
          
      Y : TYPE
          DESCRIPTION.

      Returns
      -------
      TYPE
          DESCRIPTION.
      TYPE
          DESCRIPTION.
      TYPE
          DESCRIPTION.

      """
 
      self.Yhat = self.C @ self.Xhat
      self.residual = Y - self.Yhat
      self.Xhat = (np.eye(self.dim) + self.Ts * self.A)@(self.Xhat) + self.Ts * self.B @ U + self.Ts * self.H @ (Y - self.Yhat)
      
      return self.Xhat, self.Yhat, self.residual
      
"""
    Example:
        
        
        # initialize the system parameters: 
            
          # continuous-time model:
              
            # xDot = A * X + B * u 
            #    y = C * X
        
        A  = np.array([[-1,0,0],[0,-2,0],[0,0,-3]])
        C  = np.array([[1, 0, 0],[0,1,0]])
        H  = np.array([[8,0],[0,3],[0,3]]) # observer (matrix) gain
        IC = np.array([5,4,3])             # Initial Conditions
        Ts = .1                            # Sample time
        
        # simulation duration: [0 - 10] sec with Ts = .1 ==> 101 sample 
        
        # real system:
            
            # Preallocate the system states and outputs
            
        x = np.zeros((len(A),101))
        y = np.zeros((np.size(C,0),101))

        x[:,0] = IC
        
        for i in range(100):
            #  X[k+1] = (I + Ts * A) * X[k] + Ts * B * u[k]
            x[:,i+1] = (np.eye(len(A))+Ts*A).dot(x[:,i])
            y[:,i] = C.dot(x[:,i])
        
        # =========================== Observer ============================
        # 
        #   continuous-time model:
            
            # xhatDot = A * Xhat + B * u + H (y - yhat)
            #    yhat = C * Xhat
        
        # Preallocate the observer states and outputs 
             
        Xhat = np.zeros((len(A),101))
        yhat = np.zeros((np.size(C,0),101))
        res =  np.zeros((np.size(C,0),101))
        
        # set the observer's IC
        
        IC_obs = np.array([1,.5,.3]).reshape(3,1)
        Xhat[:,:1] = IC_obs 
        
        # initialize the observer
        
        obs = LuenbergerObserver(A,C,H,IC_obs,Ts)
    
        # run and update the observer
        
        for i in range(100):
            Xhat[:,i+1:i+2], yhat[:,i:i+1], res[:,i:i+1] = obs.Update(y[:,i:i+1])
        
        # ==================================================================
        
        t = np.arange(0, 10.1, Ts)
        plt.plot(t, y.T, 'b', t, yhat.T, '--r')
        plt.show()    

"""

# %%


class UnknownInputObserver():
  def __init__(self, F, B, C, h, K, IC, Ts):
      """
      Initilization of the Unknown input Observer

      Parameters
      ----------
      F : numpy array (N-by-N Matrix) 
          Observer dynamics matrix.
      C : numpy array (p-by-N matrix)
          Output matrix.
      h : numpy array (matrix/vector)
          Decoupling matrix to be designed.
      K : numpy array (matrix/vector)
          Observer gain matrix to be designed..
      IC : numpay array (N-by-1 vector) 
          Initial Condition.
      Ts : Scalar
          Time sampling of the discretized UIO as well as discrete-time simulation

      Returns
      -------
      None.

      """
      self.F = F
      self.B = B
      self.C = C
      self.h = h
      self.K = K
      self.IC = IC
      self.Ts = Ts
      self.residual = None
      self.dim = len(F)
      self.Xhat = IC
      self.Z = None
      self.yhat = np.dot(self.C, self.Xhat) 
    
  def Update(self, U, Y):
      """
      

      Parameters
      ----------
      Y : np.array 
          real system's measurements.
      U : np.array 
          real system's known input.    

      Returns
      -------
      TYPE
          DESCRIPTION.
      TYPE
          DESCRIPTION.
      TYPE
          DESCRIPTION.
      TYPE
          DESCRIPTION.

      """
      if np.any(self.Z) == None:
          self.Yhat = self.C @ self.Xhat
          self.residual = Y - self.Yhat
          # the first iteration (initialization at t = 0)
          self.Z = self.Xhat - self.h @ Y
          self.Z = (np.eye(self.dim) + self.Ts * self.F) @ (self.Z) + self.Ts * self.T @ self.B @ U + self.Ts * self.K @ Y
      else:
          self.Xhat = self.Z + self.h @ Y
          self.Yhat = self.C @ self.Xhat
          self.residual = Y - self.Yhat
          self.Z = (np.eye(self.dim) + self.Ts * self.F) @ (self.Z) + self.Ts * self.T @ self.B @ U + self.Ts * self.K @ Y
          
      # Hint: this implemnetation does not return Z[k=0], but Z[k];k>=1. To 
      # store the values of Z[k] ; k>=0, store self.Xhat and recover Z using
      # their relationship Z = self.Xhat - np.dot(h, Y)
      return self.Xhat, self.Yhat, self.Z, self.residual



"""
    Example:
        
        
        # initialize the system parameters: 
        
          # continuous-time model:
              
            # xDot = A * X + B * u + E * d
            #    y = C * X
            # where u is the known input and d is the unknown exogenous input
            # E msut be full column rank
        
        A  = np.array([[-1,0],[0,-2]])
        B  = np.array([[0],[1]])
        C  = np.array([[1, 0],[0,1]])
        E  = np.array([[1],[0]])
        
        
        IC_sys = np.array([[5],[4]])
        Ts = .1
        
        # simulation duration: [0 - 10] sec with Ts = .1 ==> 101 sample 
        
        # real system:
            # Preallocate the system states and outputs
        x = np.zeros((len(A),101))
        y = np.zeros((np.size(C,0),101))
        d = np.zeros((1,101))

        x[:,0:1] = IC_sys
        
        import math 
        
        for i in range(100):
            #  X[k+1] = (I + Ts * A) * X[k] + Ts * B * u[k] 
            d[:,i] = .2 * math.sin(2*math.pi*1*Ts*i)
            x[:,i+1:i+2] = (np.eye(len(A))+Ts*A).dot(x[:,i:i+1]) + E * d[:,i]
            y[:,i] = C.dot(x[:,i])
        
        # ========================== UIO Observer ===========================
        #
        #   continuous-time model:
            
            #  zDot = F * Z + (Kbar + KK) * y =  F * Z + K * y
            #  yhat = C * Xhat
            #  Z = Xhat - h * y ;           where Z is the UIO state
        
        # initialize the UIO parameters:
            
        # Ref.  
        # Chen, Jie, Ron J. Patton, and Hong-Yue Zhang. "Design of unknown
        # input observers and robust fault detection filters." International
        # Journal of control 63.1 (1996): 85-105.
        
        # C : output matrix corresponding to the locally available measurements y
        F  = np.array([[-2.5,0],[0,-3.5]]) 
        h  =  np.array([[1,0],[0,0]])
        Kbar = np.array([[2.5,0],[0,1.5]])
        KK = np.array([[-2.5,0],[0,0]])
        K = Kbar + KK 
        
        # Preallocate the observer states and outputs 
        
        
        Xhat = np.zeros((len(F),101))
        yhat = np.zeros((np.size(C,0),101))
        res =  np.zeros((np.size(C,0),101))
        
        # set the observer's IC
        
        IC_obs = np.array([1,.5]).reshape(2,1)
        Xhat[:,:1] = IC_obs 
        
        
        # initialize the observer
        
        UIObs = UnknownInputObserver(F, C, h, K, IC_obs, Ts)
    
        # run and update the observer
        
        for i in range(100):
            Xhat[:,i:i+1], yhat[:,i:i+1], _, res[:,i:i+1] = UIObs.Update(y[:,i:i+1])
        
        # ==================================================================
        
        t = np.arange(0, 10.1, Ts)
        plt.plot(t, y.T, 'b', t, yhat.T, '--r')
        plt.show()    

"""
   
    