""" Copyright 2023, Rayan Bahrami, 
    Safe Autonomous Systems Lab (SAS Lab)
    Stevens Institute of Technology
    See LICENSE file for the license information.
    
    credit: part of the code is adapted and modified from the following repository:
    https://github.com/lis-epfl/vswarm/tree/master/src/vswarm/relative_localization


    This module performs visual relative localization.
    It provides bounding box to relative position conversion and vice versa.
    It also provides a method to convert a 2D point to a 3D bearing vector.
"""

import numpy as np
import cv2


class RelativeLocalizer:
    """A relative localization wrapper that precomputes the inverse camera matrix"""

    def __init__(self, K, D=None, R_C2B=np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])):
        self.K = K  # camera matrix
        self.D = D  # fisheye distortion parameters
        self.K_inv = np.linalg.inv(K)  # precompute for efficiency!
        self.R_B2C = R_C2B.T
        self.KR_B2C = self.K @ R_C2B.T
        self.RK_inv = R_C2B @ self.K_inv  # precompute for efficiency!

    def relative_pos_from_bbox(
        self, bbox, R_B2W, object_size, bearning_angle=None, distance=None
    ):
        """recovers the rel. position from a 2D bounding box, rotation matrix, and the observed object size

        Args:
            bbox (ndarray or list): Bounding box in pixel coordinates (x, y, width, height)
            R_B2W (ndarray): Rotation matrix from the agents' body-fixed FLU frame to the world frame
            object_size (ndarray or list): Physical size of the object (width, height)
            bearning_angle (float, optional): Bearing angle to object. Defaults to None.
            distance (float, optional): Distance to object. Defaults to None.

        Returns:
            P_bt (ndarray): Relative position vector from body to object in the world frame
            distance (float): Distance to object in bode-fixed frame
        """
        x, y, w, h = bbox
        obj_w, obj_h = object_size

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        if bearning_angle is not None and distance is not None:
            # if the observing and observed object are at the same height, the bearing angle is azimuth
            depth = distance * np.cos(bearning_angle)
        else:  # assuming a planer object
            # Obj_x, Obj_y, Obj_z = R_B2W.inv().apply([0, obj_w, obj_h])
            # depth_W = (fx / w) * max(abs(Obj_x), abs(Obj_y))
            # depth_H = (fy / h) * Obj_z
            Obj_x, Obj_y, Obj_z = self.R_B2C @ R_B2W.inv().apply([0, obj_w, obj_h])
            depth_W = (fx / w) * abs(Obj_x)
            depth_H = (fy / h) * abs(Obj_y)

            # use max for Tello drones, use min for jackal-UGV
            depth = min(depth_W, depth_H)

        # P_bt = P_body - P_object
        # P_bt = -np.array(depth) * R_B2W.apply(
        #     [
        #         1,
        #         (cx - x) / fx,
        #         (cy - y) / fy,
        #     ]
        # )
        P_bt = -np.array(depth) * R_B2W.apply(self.RK_inv @ np.array([x, y, 1.0]))
        return P_bt, depth

    def detection_to_bearing(self, bbox, object_size, axis: str = None):
        # axis: "x" or "y" to specify the camera axis to use
        K, D, K_inv = self.K, self.D, self.K_inv
        return detection_to_bearing_and_distance(bbox, object_size, K, D, K_inv, axis)

    def point_to_bearing(self, bbox_center):
        K, D, K_inv = self.K, self.D, self.K_inv
        return point_to_bearing(bbox_center, K, D, K_inv)

    def reprojection_pos_to_bbox(self, P_b2o, R_B2W, object_size):
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        obj_w, obj_h = object_size

        depth = abs(P_b2o[0])  # approx. x-axis distance
        # print(f"depth: {depth}")
        Pb = R_B2W.inv().apply(P_b2o)
        # print(f"Pb: {Pb}")
        P_in_c = self.KR_B2C @ Pb
        # print(f"P_in_c: {P_in_c}")
        P_in_c[0] /= P_in_c[2]  # x in image plane
        P_in_c[1] /= P_in_c[2]  # y in image plane

        Obj_x, Obj_y, Obj_z = self.R_B2C @ R_B2W.inv().apply([0, obj_w, obj_h])
        # print(f"Obj_x: {Obj_x}, Obj_y: {Obj_y}, Obj_z: {Obj_z}")
        obj = max(abs(Obj_x), abs(Obj_y), abs(Obj_z))
        bbox_w = (fx / depth) * abs(Obj_x)
        bbox_h = (fy / depth) * abs(Obj_y)

        return np.array([P_in_c[0], P_in_c[1], bbox_w, bbox_h])


def detection_to_bearing_and_distance(bbox, object_size, K, D, K_inv, axis=None):
    """Convert a 2D bounding box to a 3D unit-norm bearing vector based on object size,
      as well as the distance and bearing angle

    Args:
        bbox_center (ndarray): Bounding box center in pixel coordinates (x, y, w, h)
        object_size (tuple): Physical size of the object (width, height)
        K (ndarray): Camera matrix with focal lengths and optical center (fx, fy, cx, cy)
        D (ndarray): Fisheye distortion parameters (k1, k2, k3, k4)
        K_inv (ndarray): Inverse camera matrix

    Returns:
        bearing (ndarray): Unit-norm 3D bearing vector to point (x, y, z)
        distance (float): Distance to object
        bearing_angle (float): Bearing angle to object
    """

    x, y, w, h = bbox

    if axis is None:
        axis = "x" if (abs(x) + w / 2) >= (abs(y) + h / 2) else "y"

    obj_len = object_size[0] if axis == "x" else object_size[1]

    # Create point to the bbox center and the furthest edge
    pnt_center = np.array([x, y])
    pnt_border = (
        np.array([x + w / 2.0, y]) if axis == "x" else np.array([x, y + h / 2.0])
    )

    # Calculate the bearing to the center of the object
    bearing_center = point_to_bearing(pnt_center, K, D, K_inv)

    # Calculate bearing to the furthest visible edge/side of the object
    bearing_border = point_to_bearing(pnt_border, K, D, K_inv)

    # Calculate the angle between the two bearing vectors (already normalized!)
    angle = np.arccos(bearing_center.dot(bearing_border))

    # Calculate distance to object with known object width (and optionally depth)
    distance = (obj_len / 2.0) / np.tan(angle)  # + (self.object_depth / 2)

    # Scale bearing vector by distance
    bearing = bearing_center * distance

    # =============== calculate bearing angle between bbox center and camera center  ===============
    # Calculate unit-norm bearing vector of the z-axis
    bearing_z_axis = np.array([0.0, 0.0, 1])  # camera z-axis forward-pointing

    # Calculate the angle between the two bearing vectors (already normalized!)
    bearing_angle = np.arccos(bearing_center.dot(bearing_z_axis))
    # if the observing and observed object are at the same height, the bearing angle is azimuth

    return bearing, distance, bearing_angle


def point_to_bearing(bbox_center, K, D, K_inv):
    """Convert a 2D point in pixel coordinates to a unit-norm 3D bearing vector.

    Args:
        bbox_center (ndarray): Bounding box center in pixel coordinates (x, y)
        K (ndarray): Camera matrix with focal lengths and optical center (fx, fy, cx, cy)
        D (ndarray): Fisheye distortion parameters (k1, k2, k3, k4)
        K_inv (ndarray): Inverse camera matrix

    Returns:
        bearing (ndarray): Unit-norm 3D bearing vector to point (x, y, z)
    """

    if D is None:
        undistorted_image_point_homogeneous = np.array(
            [bbox_center[0], bbox_center[1], 1.0]
        )
    else:
        # Undistort points from fisheye as if they were created by a pinhole camera
        image_point_cv = bbox_center.reshape(1, -1, 2)  # shape: (1, 1, 2)
        undistorted_image_point_cv = cv2.fisheye.undistortPoints(
            image_point_cv, K, D, P=K
        )
        undistorted_image_point = undistorted_image_point_cv.reshape(2)
        # back-project points using inverse camera matrix with homogeneous coordinates
        undistorted_image_point_homogeneous = np.array([*undistorted_image_point, 1.0])

    world_point = K_inv.dot(undistorted_image_point_homogeneous)
    bearing_norm = world_point / np.linalg.norm(world_point)
    return bearing_norm
