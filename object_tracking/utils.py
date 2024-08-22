""" Copyright 2023, Rayan Bahrami, 
    Safe Autonomous Systems Lab (SAS Lab)
    Stevens Institute of Technology
    See LICENSE file for the license information.
"""

# data association and filtering

import numpy as np
import cv2
from omegaconf import OmegaConf
from scipy.spatial.transform import Rotation
from scipy.optimize import linear_sum_assignment


# metrics for gating
def get_iou_bboxes(box1, box2, x1y1x2y2=False, eps=1e-9, numpy=False):
    # https://github.com/ultralytics/yolov5/blob/v5.0/utils/general.py#L343C11-L343C11
    # Returns the IoU of box1 to box2. box1 is 4, box2 is nx4
    box2 = box2.T

    # Get the coordinates of bounding boxes
    if x1y1x2y2:  # x1, y1, x2, y2 = box1
        b1_x1, b1_y1, b1_x2, b1_y2 = box1[0], box1[1], box1[2], box1[3]
        b2_x1, b2_y1, b2_x2, b2_y2 = box2[0], box2[1], box2[2], box2[3]
    else:  # transform from xywh to xyxy
        b1_x1, b1_x2 = box1[0] - box1[2] / 2, box1[0] + box1[2] / 2
        b1_y1, b1_y2 = box1[1] - box1[3] / 2, box1[1] + box1[3] / 2
        b2_x1, b2_x2 = box2[0] - box2[2] / 2, box2[0] + box2[2] / 2
        b2_y1, b2_y2 = box2[1] - box2[3] / 2, box2[1] + box2[3] / 2

    # Intersection area

    inter = (min(b1_x2, b2_x2) - max(b1_x1, b2_x1)).clip(0) * (
        min(b1_y2, b2_y2) - max(b1_y1, b2_y1)
    ).clip(0)

    # Union Area
    w1, h1 = b1_x2 - b1_x1, b1_y2 - b1_y1 + eps
    w2, h2 = b2_x2 - b2_x1, b2_y2 - b2_y1 + eps
    union = w1 * h1 + w2 * h2 - inter + eps

    iou = inter / union

    return iou  # IoU


def iou_batch(bboxes_A, bboxes_B):
    """From SORT: Computes IOU between two bboxes in the form [x1,y1,x2,y2]
    https://github.com/abewley/sort/blob/master/sort.py#L47C31-L47C31
    Modifications: Input, Outpot names were changed.
    """
    # bboxes_B serves as the ground truth
    bboxes_B = np.expand_dims(bboxes_B, 0)
    bboxes_A = np.expand_dims(bboxes_A, 1)

    xx1 = np.maximum(bboxes_A[..., 0], bboxes_B[..., 0])
    yy1 = np.maximum(bboxes_A[..., 1], bboxes_B[..., 1])
    xx2 = np.minimum(bboxes_A[..., 2], bboxes_B[..., 2])
    yy2 = np.minimum(bboxes_A[..., 3], bboxes_B[..., 3])
    w = np.maximum(0.0, xx2 - xx1)
    h = np.maximum(0.0, yy2 - yy1)
    wh = w * h
    iou_matrix = wh / (
        (bboxes_A[..., 2] - bboxes_A[..., 0]) * (bboxes_A[..., 3] - bboxes_A[..., 1])
        + (bboxes_B[..., 2] - bboxes_B[..., 0]) * (bboxes_B[..., 3] - bboxes_B[..., 1])
        - wh
    )
    return iou_matrix.T  # M_by_N,  M tracks and N detections


def xywh_to_xyxy(x):
    """Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right

    https://github.com/ultralytics/yolov5/blob/master/utils/general.py#L828
    """
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2  # top left x
    y[..., 1] = x[..., 1] - x[..., 3] / 2  # top left y
    y[..., 2] = x[..., 0] + x[..., 2] / 2  # bottom right x
    y[..., 3] = x[..., 1] + x[..., 3] / 2  # bottom right y
    return y


class Derivative:
    """fist-order derivative filter"""

    def __init__(self, F, Ts, method="backward", prev_u=0, prev_y=0):
        """__init__ _summary_
        method: str,  'forward' or 'backward' or 'trapezoidal'
        """
        self.F = F  # low-pass filter cut-off frequency
        self.Ts = Ts  # sampling period of digital filter
        self.prev_u = prev_u  # Previous input value
        self.prev_y = prev_y  # Previous output value
        self.method = method

    def reset(self):
        self.__init__(F=self.F, Ts=self.Ts)

    def __call__(self, u):
        """
        Process a single sample through the filter.

        Parameters:
        u: Current input sample.

        Returns:
        y: Current output sample.
        """
        # Calculate current output
        # fmt: off

        # u = (1 / (1 + self.F * self.Ts)) * (
        #     self.prev_u + self.F * self.Ts * (u)
        # ) 
        if self.method == "forward": 
            y = (1 / (1 - self.Ts * self.F)) * self.prev_y +\
                self.Ts * self.F * ( u - self.prev_u) 
        elif self.method == "backward":
            y = (1 / (1 + self.Ts * self.F)) * (self.prev_y + self.F * (u - self.prev_u))
        elif self.method == "trapezoidal":
            y = (1 / (1 + 0.5 * self.Ts * self.F)) * (
                (1 - 0.5 * self.Ts * self.F) * self.prev_y + self.F * (u - self.prev_u)
            )
        # Update previous values for the next iteration
        self.prev_u = u
        self.prev_y = y

        return y


class STWDifferentiator:
    """Robust Exact Differentiation via Sliding Mode Technique
    # https://www.sciencedirect.com/science/article/abs/pii/S0005109897002094

    """

    def __init__(self, a, b, Ts, F):
        self.a = a  #
        self.b = b
        self.Ts = Ts
        self.x = 0  # internal state with composite state s = x - f(t)
        self.prev_z = 0  # previous s2=z
        self.prev_zdot = 0  # previous zdot
        self.x = 0
        self.t0 = True
        self.fdot = 0
        self.F = F

    def _fdot(self, s, z):
        return -self.a * (np.sqrt(np.abs(s)) * np.sign(s)) + z

    def zdot(self, s):
        return -self.b * np.sign(s)

    def reset(self):
        self.__init__(a=self.a, b=self.b, Ts=self.Ts)

    def __call__(self, f):

        if self.t0:
            self.x = f
            self.t0 = False

        s = self.x - f

        # mu = -self.a * (np.sqrt(np.abs(s)) * np.sign(s))
        # zdot = -self.b * np.sign(s)
        # z = self.prev_z + (self.Ts / 2) * (zdot + self.prev_zdot)
        # self.x = self.x + (self.Ts / 2) * (mu + z + self.fdot)  # fdot = mu + z

        # self.prev_z, self.prev_zdot = z, zdot

        # self.fdot = (1 / (1 + self.F * self.Ts)) * (
        #     self.fdot + self.F * self.Ts * (mu + z)
        # )  # low-pass filter with cut-off frequency F - backward rule discretization

        # integration of zdot
        k1 = self.Ts * self.zdot(s)
        k2 = self.Ts * self.zdot(s + 0.5 * k1)
        k3 = self.Ts * self.zdot(s + 0.5 * k2)
        k4 = self.Ts * self.zdot(s + k3)
        z = self.prev_z + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0

        # integration of fdot
        k1 = self.Ts * self._fdot(s, self.prev_z)
        k2 = self.Ts * self._fdot(s + 0.5 * k1, self.prev_z + 0.5 * k1)
        k3 = self.Ts * self._fdot(s + 0.5 * k2, self.prev_z + 0.5 * k2)
        k4 = self.Ts * self._fdot(s + k3, self.prev_z + k3)

        self.x += (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        self.fdot = (1 / (1 + self.F * self.Ts)) * (
            self.fdot + self.F * self.Ts * (self._fdot(s, self.prev_z))
        )
        self.prev_z = z
        # self.x is the filtered signal f(t)
        return self.x, self.fdot  # f(t), f_dot(t)


""" example of usage

    np.random.seed(250)
    t = np.linspace(0, 10, 501)  # Ts = 0.02
    TT = 10
    y = np.sin(2 * np.pi * (t / TT)) + 0.1 * np.sin(2 * np.pi * t / 0.25)
    # 1 * np.random.normal(0, 0.1, t.shape)
    Y_DOT_a = (2 / TT) * np.pi * np.cos(2 * np.pi * (t / TT))

    # t = np.linspace(0, 10, 101)  # Ts = 0.02
    # y = 5 * t + np.sin(t)
    # # 1 * np.random.normal(0, 0.1, t.shape)
    # Y_DOT_a = 5 + np.cos(t)

    Y_DOT, fdot = [], []
    dev = Derivative(F=5, Ts=0.1, method="trapezoidal")
    stw = STWDifferentiator(a=2, b=2, Ts=0.01, F=5)
    # stw.reset() # Reset the filter
    # dev.reset() # Reset the filter
    for u in y:
        Y_DOT += [dev(u)]
        fdot += [stw(u)]

    plt.plot(t, y, "r", t, Y_DOT_a, "k", t, Y_DOT, "--c", t, fdot, "-.b")

"""
