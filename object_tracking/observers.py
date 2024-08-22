# -*- coding: utf-8 -*-
"""
@author: Rayan Bahrami
"""

import numpy as np

# import matplotlib.pyplot as plt
from scipy.linalg import inv

# from .similarity import crop_image, calculate_ssim, calculate_cosine_similarity
from collections import deque


class KalmanFilter:
    def __init__(
        self, A, Ts, q_pos, q_vel, IC=None, R_std=None, P=None, dtype=None, bufsize=10
    ):
        """
        if C is (1, p) ndarray, do not use vector form (p,) ndarray (e.g. np.array([[1, 0, 0]]) instead of np.array([1, 0, 0]))
        also if C is (1, p) ndarray, R should be scalar
        states (xhat) and IC are 1D vectors
        """
        # System dynamics
        # xDot = A * X + B * u + w
        #    y = C * X + e
        self.dtype = np.float32 if dtype is None else dtype
        self.A = self.dtype(A)
        self.Cp = np.block([np.eye(3), np.zeros((3, 3))])
        self.Cv = np.block([np.zeros((3, 3)), np.eye(3)])
        self.C = np.eye(6)
        self.Ts = self.dtype(Ts)
        self.dimA = len(self.A)
        # Estimation: Kalman Filter
        # set the initial condition
        self.IC = (
            np.zeros(self.dimA, dtype=self.dtype) if IC is None else self.dtype(IC)
        )
        self.xhat_prior = self.IC
        self.xhat_posterior = self.xhat_prior
        # process noise covariance - cov_w
        self.Q = np.block(
            [
                [np.eye(3) * q_pos**2, np.zeros((3, 3))],
                [np.zeros((3, 3)), np.eye(3) * q_vel**2],
            ]
        )
        # measurement noise covariance - cov_e
        self.R_cov = R_std**2 if R_std is not None else 0.08
        # self.R_inv = self._inv(self.R)
        # self.S_inv = self.R_inv
        # state covariance
        self.P_prior = (
            np.eye(self.dimA, dtype=self.dtype) * 20
            # if IC is None or P is None
            if P is None
            else self.dtype(P)
        )
        self.P_posterior = self.P_prior
        self.KalmanGain = np.full(
            (self.dimA, 3),
            np.nan,
            dtype=self.dtype,
        )
        self.innovation = np.full(3, np.nan, dtype=self.dtype)
        self.residual = np.full(3, np.nan, dtype=self.dtype)
        # self.yhat = np.dot(self.C, self.xhat) np.eye(3, dtype=np.float64)
        self.missed_measurements = 0
        self.t_prev = 0
        self.dt_history = []
        #
        self.bufsize = bufsize  # the length of the tracklet
        # store the past bufsize iterations of the track states and covariances
        self.tracklet_x = deque(maxlen=bufsize)
        self.tracklet_P = deque(maxlen=bufsize)
        self.tracklet_S = deque(maxlen=bufsize)
        # initialize
        self.tracklet_x.extend([np.full(self.dimA, np.nan)] * self.tracklet_x.maxlen)
        self.tracklet_x.append(self.IC)
        self.tracklet_P.extend(
            [np.full((self.dimA, self.dimA), np.nan)] * self.tracklet_P.maxlen
        )
        self.tracklet_P.append(self.P_posterior)

    #   self.yhat = self.C @ self.xhat

    def predict(self, t=None):
        """predict a priori - should be run before update() method

        Args:
            u (ndarray, optional): 1D vector control input. Defaults to 0.
        """
        # predict a priori
        if t is not None:
            dt = t - self.t_prev
            self.dt_history += [dt]
            self.t_prev = t
            self.A = np.block(
                [[np.eye(3), dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]]
            )
            if dt > 1:
                print(
                    f"Warning: dt: {dt} is large than 1 ! make EKF.t_prev has been initialized properly."
                )

        self.xhat_prior = self.A @ self.xhat_posterior
        self.P_prior = self.A @ self.P_posterior @ self.A.transpose() + self.Q
        # print(f"self.P_prior:\n{self.P_prior}")
        # return self.xhat_prior, self.P_prior

    def update(
        self,
        yp=None,
        yv=None,
        R_pos_cov=None,
        itr=None,
        association=False,
        mahalanobis_threshold=100.0,
    ):
        """update a posteriori estimation - shoul be run after predict() method
        Args:
            y (ndarray, optional): (p,) 1D vector of measurments. Defaults to None.
            itr (int, optional): _description_. Defaults to None.
        """
        # update a priori
        # self.xhat_prior = self.A @ self.xhat_posterior + self.B.dot(u)
        # self.P_prior = self.A @ self.P_posterior @ self.A.transpose() + self.Q

        # update a posteriori
        if association:
            yp, R_pos_cov = self._measurement_to_track(itr, yp, mahalanobis_threshold)

            # print(f"yp after assoc itr {itr}: {yp}, yv: {yv}")

        if yp is None:
            # print(f"here at itr {itr}")
            self.missed_measurements += 1  # count missed pos. measurements
            H = self.Cv
            R = np.eye(3) * self.R_cov
            Y = yv
        else:
            H = self.C
            R = np.block(
                [
                    [np.eye(3) * R_pos_cov, np.zeros((3, 3))],
                    [np.zeros((3, 3)), np.eye(3) * self.R_cov],
                ]
            )
            Y = np.block([yp, yv])
            # print(f"Y itr {itr}: {Y}")

        S = H @ self.P_prior @ H.transpose() + R
        self.S_inv = inv(S)
        # print(f"S.shape outer itr {itr}: {S.shape}")
        self.KalmanGain = self.P_prior @ H.transpose().dot(self.S_inv)
        # self.KalmanGain = self.P_posterior @ self.C.transpose() @ inv(self.R)

        self.xhat_posterior = self.xhat_prior + self.KalmanGain @ (
            Y - H @ self.xhat_prior
        )
        # print(f"after xhat_posterior itr {itr}")
        self.P_posterior = (
            np.eye(self.dimA, dtype=self.dtype) - self.KalmanGain @ H
        ) @ self.P_prior
        # print(f" P_posterior itr {itr} {self.P_posterior.shape}")
        self.innovation = Y - H @ self.xhat_prior
        # print(f"self.innovation itr {itr} {self.innovation.shape}")
        self.residual = Y - H @ self.xhat_posterior
        # print(f"self.residual itr {itr} {self.residual.shape}")

        # store the tracklet
        self.tracklet_x.append(self.xhat_posterior)
        self.tracklet_P.append(self.P_posterior)
        self.tracklet_S.append(S)

        # return (
        #     self.xhat_prior,
        #     self.xhat_posterior,
        #     self.P_prior,
        #     self.P_posterior,
        #     self.KalmanGain,
        #     self.innovation,
        #     self.residual,
        # )

    def _measurement_to_track(self, itr, yp, mahalanobis_threshold=3.0):
        if yp is None:
            return None, None

        # print(f"yp is list itr {itr}: {isinstance(yp, list)}")
        # Mahalanobis distance
        mahalanobis_dists = []
        cov = []
        FOUND_ONE = False
        for y, conf in yp:
            # print(f"y itr {itr}: {y}, conf: {conf}")
            R = np.eye(3) * (1 - conf) * 0.4 + 0.01
            cov.append(R)
            S = self.Cp @ self.P_prior @ self.Cp.transpose() + R
            y_hat = self.xhat_prior[:3]
            # print(f"y_hat itr {itr}: {y_hat}, S: {S}")

            dist = np.sqrt((y - y_hat) @ np.linalg.inv(S) @ (y - y_hat).T)
            # if itr<=300:
            # print(f"Mahalanobis distance: {dist} itr {itr}")

            if dist <= mahalanobis_threshold:
                FOUND_ONE = True
            else:
                dist = np.inf

            mahalanobis_dists.append(dist)

        if FOUND_ONE:
            inx = mahalanobis_dists.index(min(mahalanobis_dists))
            return yp[inx][0], cov[inx]
        else:
            return None, None
