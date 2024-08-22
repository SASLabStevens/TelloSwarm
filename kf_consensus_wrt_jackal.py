#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""
@author: Rayan Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

import pathlib
import os, sys

FILE_ABS_DIR = pathlib.Path(__file__).absolute()
ROOT_DIR = FILE_ABS_DIR.parent  # src

if ROOT_DIR.as_posix() not in sys.path:
    sys.path.append(ROOT_DIR.as_posix())

telloswarm_ROOT = ROOT_DIR / "telloswarm_plus"
if telloswarm_ROOT.as_posix() not in sys.path:
    sys.path.append(telloswarm_ROOT.as_posix())

OBJ_Det_ROOT = ROOT_DIR / "object_detection"
if OBJ_Det_ROOT.as_posix() not in sys.path:
    sys.path.append(OBJ_Det_ROOT.as_posix())


from telloserver_EDU import TelloSwarm

import concurrent.futures
from concurrent.futures import ThreadPoolExecutor
import threading
import queue

import time
import cv2
import numpy as np

from omegaconf import OmegaConf


from collections import OrderedDict

save_data = "True"  # Set to 'True' to save I/O data, otherwise set to 'False'
cfg = OmegaConf.load("./configs/test-2tello.yaml")


from wrapper import YoloV7

# from perceptual_adversary.perceptual_adv import AdversarialGenerator
# from yolov7.utils.loss import ComputeLoss

from scipy.spatial.transform import Rotation
from scipy.linalg import block_diag

from object_tracking.observers import KalmanFilter

from localization.localization import RelativeLocalizer
from object_tracking.utils import Derivative

from scipy.interpolate import interp1d


def get_DoS(p, num_dos, t_s, t_end, seed=None):
    # independent Bernoulli
    # n=1, p are num of binary trials for DoS, success probability of each trial
    seed = 703093 if seed is None else seed
    np.random.seed(seed)

    dos_dur = t_end / num_dos  # 1/dos_freq must be 0.1, 0.01, 0.001

    # independent Bernoulli
    dos_ = np.random.binomial(n=1, p=p, size=num_dos + 1)

    # t = np.arange(0, t_end + dos_dur, dos_dur)
    t = np.linspace(0, t_end, num_dos + 1)

    # if dos_dur == t_s:
    num_itr = np.int64(t_end / t_s)
    if num_dos == num_itr:
        DoS = dos_
    else:
        t_span = np.linspace(0, t_end, num_itr + 1)
        f_dos = interp1d(t, dos_, kind="previous")
        DoS = f_dos(t_span)

    return DoS


# to simulate misclassification effect = intermittent missurements
DoS = get_DoS(p=0.2, num_dos=200, t_s=0.02, t_end=20, seed=93658)
# to simulate mislocalaization effect = spurious measurements
adv = get_DoS(p=0.3, num_dos=400, t_s=0.02, t_end=20, seed=10658)

ref_id = 81  # target class id # 81 for Jackal and 80 for Tello

calib_matrix = np.array(
    [
        [cfg.camera.fx, 0, cfg.camera.cx],
        [0, cfg.camera.fy, cfg.camera.cy],
        [0, 0, 1],
    ]
)
ang = (-90 - 11) * np.pi / 180  # tilted camera: 11 degree rotation in the camera x-axis
R_C2B = np.array(
    [[0, np.cos(ang), -np.sin(ang)], [-1, 0, 0], [0, np.sin(ang), np.cos(ang)]]
)

R_B2C = R_C2B.T

localizer = RelativeLocalizer(K=calib_matrix, R_C2B=R_C2B)
vel_estimator = [
    Derivative(
        F=15, Ts=0.02, method="trapezoidal", prev_u=np.zeros(3), prev_y=np.zeros(3)
    )
    for _ in range(cfg.numAgents)
]

W_estimator = [
    Derivative(
        F=15, Ts=0.02, method="trapezoidal", prev_u=np.zeros(3), prev_y=np.zeros(3)
    )
    for _ in range(cfg.numAgents)
]


def F(dt):
    return np.block([[np.eye(3), dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])


cov_0 = block_diag(np.eye(3), np.eye(3) * 0.05)
EKF = [
    KalmanFilter(
        A=F(0.02),
        Ts=0.02,
        q_pos=0.224,
        q_vel=0.2,
        IC=None,
        R_std=0.28,
        P=cov_0,
        dtype=np.float32,
        bufsize=cfg.max_itr + 1,
    )
    for _ in range(cfg.numAgents)
]

Adj = np.array(cfg.Adj)
Pstar = [
    np.array([cfg.Pstar.x[id], cfg.Pstar.y[id], cfg.Pstar.z[id]])
    for id in range(cfg.numAgents)
]

weights = "yolov7-tiny-best.pt"  # pre
weights_path = OBJ_Det_ROOT / weights

detector = [
    YoloV7(
        weights_path.as_posix(),
        conf_thresh=cfg.conf_thres,
        iou_thresh=cfg.iou_thres,
        img_size=640,  # 480
        device="cuda",
        bbox_plot_line_thickness=2,
    )
    for _ in range(cfg.numAgents)
]
# agent 2 subject to the adversary
# adv_generator = AdversarialGenerator(detector[1], ComputeLoss)

img_queue = queue.Queue()
stop_display = threading.Event()


def display_images(img_queue, stop_event, Ts=0.01):
    while not stop_event.is_set():
        if not img_queue.empty():
            name, img = img_queue.get()
            cv2.imshow(name, img)
            cv2.waitKey(1)
        time.sleep(Ts)


agents_pos_data = {
    i: {"P": np.zeros(3)} for i in range(cfg.numAgents)
}  # Initialize data structure for drones
lock = threading.Lock()  # Lock for synchronization

# store the timestamp of each iteration for each agent
timestamps = [[] for _ in range(cfg.numAgents)]

DronesDict = {
    "EDU_7": ("172.16.0.17", 1037),
    "EDU_8": ("172.16.0.18", 1038),
}

g, alpha, gamma = cfg.g, cfg.alpha, cfg.gamma
Ts, max_itr = cfg.sample_time, cfg.max_itr
numAgents = len(DronesDict)

from PID import PID

MAX_PID = 100  #  saturation on PID outputs
Dterm_filter = 10

K_px = 330  # 400
K_dx = 330  # 400
K_ix = 10  # 130

K_py = 330  # 400
K_dy = 330  # 400
K_iy = 10  # 130

K_pz = 200  # 300
K_dz = 200
K_iz = 5

frq = 0.1
WW = 2 * np.pi * frq

pid_z = [
    PID(
        Kp=K_pz,
        Kd=K_dz,
        Ki=K_iz,
        derivativeFilterFreq=Dterm_filter,
        minOutput=-MAX_PID,
        maxOutput=MAX_PID,
        current_time=time.time(),
    )
    for _ in range(numAgents)
]

# ground-truth position and velocity in xyz-directions
gt_x = np.zeros((2 * numAgents, max_itr))  # 2 for position and velocity
gt_y = np.zeros((2 * numAgents, max_itr))
gt_z = np.zeros((2 * numAgents, max_itr))
gt_roll = np.zeros((numAgents, max_itr))
gt_pitch = np.zeros((numAgents, max_itr))
gt_yaw = np.zeros((numAgents, max_itr))
gt_wx = np.zeros((numAgents, max_itr))
gt_wy = np.zeros((numAgents, max_itr))
gt_wz = np.zeros((numAgents, max_itr))
action_ux = np.zeros((numAgents, max_itr))  # control input in x-direction
action_uy = np.zeros((numAgents, max_itr))
action_uz = np.zeros((numAgents, max_itr))

imu_vx = np.zeros((numAgents, max_itr))
imu_vy = np.zeros((numAgents, max_itr))
imu_vz = np.zeros((numAgents, max_itr))

imu_roll = np.zeros((numAgents, max_itr))
imu_pitch = np.zeros((numAgents, max_itr))
imu_yaw = np.zeros((numAgents, max_itr))

agents_detections = [{} for _ in range(numAgents)]


swarm = TelloSwarm(DronesDict, mocap=True)
swarm.connect()

swarm.set_video_bitrate(3)  # 0: auto, 1: 1Mbps, 2: 2Mbps, 3: 3Mbps, 4: 4Mbps, 5: 5Mbps
swarm.set_video_fps(cfg.camera.fps)
swarm.set_video_resolution(cfg.camera.res)  # 480p, 720p,

swarm.streamon(
    bufsize=5, FPS=cfg.camera.fps, resolution=(cfg.camera.H, cfg.camera.W), resize=True
)  # (480, 640), (None, None) = (720,960) = (H, w)

if swarm.STREAM:
    swarm.takeoff()

time.sleep(2)

# Initialize velocity estimator for ground-truth velocity
for tt in range(350):
    for id, name in enumerate(DronesDict):
        IP, _ = DronesDict[name]
        pos, rot = swarm.get_mocap_groundtruth_pose(IP)
        _ = vel_estimator[id](pos)  # just to converge
    time.sleep(0.02)


def run_consensus(id, name, max_itr, Ts):
    IP, _ = DronesDict[name]
    EKF[id].t_prev = time.time()
    try:
        for itr in range(max_itr):
            t0_ = time.time()
            agents_detections[id][itr] = OrderedDict()  # to store data
            # ================== inference on image ================
            try:
                frame = swarm.get_frame(IP)  # bgr image HxWxC
                # cv2.imwrite(f"results/original_{name}_{itr}.jpg", frame)
                img = frame.copy()  # important!
                # if itr <= 5:
                dets, dets_xywh = detector[id].detect(
                    img,
                    classes=None,
                    multi_label=False,
                    perturb_boxes=False,
                    perturb_target_id=81,  # jackal-UGV
                    deviation_percentage=0.75,
                    num_points=0,
                )
                # else:
                # ======= adversarial mislocalization =======
                # dets, dets_xywh = detector[id].detect(
                #     img,
                #     classes=None,
                #     multi_label=False,
                #     perturb_boxes=bool(
                #         (id == 1) * adv[itr]
                #     ),  # adversarial mislocalization
                #     perturb_target_id=81,
                #     deviation_percentage=0.75,
                #     num_points=5, # num of spurious (fake) boxes
                # )
                # ======= adversarial misclassification =======
                # if bool(DoS[itr]) and id == 1:
                #     mask = dets_xywh[:, -1] == 81  # jackal id
                #     dets_xywh[mask, -1] = 4  # change the class id to 4
                #     dets[mask, -1] = 4  # change the class id to 4

                detector[id].draw_2d_bboxes(img, dets)
                # cv2.imwrite(f"results/{name}_{itr}.jpg", img)

                # img_queue.put((name, img))
                # cv2.imshow(f"{name}", img)  # swarm.cameras[IP].frame
                # cv2.waitKey(1)
            except Exception as e:
                print(f"Exception for {name} cv2.imshow() at {itr}: {e}")
                # continue
            # ================== get the states ==================
            states = swarm.get_state(IP)  # onborad states in local frame
            imu_vx[id, itr], imu_vy[id, itr], imu_vz[id, itr] = states.velocity
            imu_roll[id, itr], imu_pitch[id, itr], imu_yaw[id, itr] = states.rotation
            # ground truth pose in world frame
            pos, rot = swarm.get_mocap_groundtruth_pose(IP)
            gt_x[id, itr], gt_y[id, itr], gt_z[id, itr] = pos
            gt_roll[id, itr], gt_pitch[id, itr], gt_yaw[id, itr] = rot  # in radians

            (
                gt_x[id + numAgents, itr],
                gt_y[id + numAgents, itr],
                gt_z[id + numAgents, itr],
            ) = vel_estimator[id](pos)

            # gt_wx[id, itr], gt_wy[id, itr], gt_wz[id, itr] = W_estimator[id](rot)

            R_B2W_gt = Rotation.from_euler(
                "ZXY",
                [gt_yaw[id, itr], gt_roll[id, itr], gt_pitch[id, itr]],
                degrees=False,
            )  # zero yaw == aligning the world frame with the body frame
            # ========== Localization (IMU + VISION) ===============
            dt_r = time.time() - t0_
            p_ref = 1 * np.sin(WW * itr * Ts)
            v_ref = 1 * WW * np.cos(WW * itr * Ts)
            vdot_ref = -1 * (WW**2) * np.sin(WW * itr * Ts)

            yp = localizer_from_dets(dets_xywh, id, R_B2W_gt, itr)

            # _R_B2W_gt_ = Rotation.from_euler(
            #     "ZXY",
            #     [gt_yaw[id, itr], gt_roll[id, itr], gt_pitch[id, itr]],
            #     degrees=False,
            # )
            vel_in_body = R_B2W_gt.inv().apply(
                np.array(
                    [
                        gt_x[id + numAgents, itr],
                        gt_y[id + numAgents, itr],
                        gt_z[id + numAgents, itr],
                    ]
                )
            )
            vel_in_body[1] += -v_ref  # y-direction v in KF

            try:  # EKF update under adversarial measurements
                EKF[id].predict(t=time.time())
                EKF[id].update(
                    yp=yp,
                    yv=vel_in_body,
                    R_pos_cov=None,
                    itr=itr,
                    association=True,
                    mahalanobis_threshold=2.4476,  #
                )

                # ========= project Pos to bbox =============
                # Update agent's position date in agents_pos_data
                with lock:
                    agents_pos_data[id]["P"] = EKF[id].xhat_posterior[:3]

                est_box = localizer.reprojection_pos_to_bbox(
                    EKF[id].xhat_posterior[:3],
                    R_B2W_gt,
                    cfg.targets.object_size[1],
                )

                p = np.trace(EKF[id].P_posterior)  # cov_trace
                trk = np.block(
                    [[est_box, p, 0, cfg.targets.class_index[1]]]
                )  # shape must be (n, 7), here n=1
                detector[id].draw_2d_bboxes_trk(img, trk, "bottem")
                # ========= End of projection to bbox =============

                # img_queue.put((name, img))
                cv2.imwrite(f"results/{name}_{itr}.jpg", img)
            except Exception as e:
                print(f"Exception for {name} EKF at {itr}: {e}")
                with lock:
                    agents_pos_data[id]["P"] = np.full(3, np.nan)
                # cv2.imwrite(f"results/{name}_{itr}.jpg", img)
                # continue

            # ======== send the control commads to the drone =======
            try:
                U = np.zeros(3)
                if not np.isnan(agents_pos_data[id]["P"][0]):
                    # U += agents_pos_data[id]["P"] - Pstar[id]
                    for j in range(cfg.numAgents):
                        if Adj[id, j] == 1:  # if there is a comm. link
                            if not np.isnan(agents_pos_data[j]["P"][0]):
                                U += (agents_pos_data[id]["P"] - Pstar[id]) - (
                                    agents_pos_data[j]["P"] - Pstar[j]
                                )

            except Exception as e:
                print(f"Control error for agent {id+1} at iteration {itr}: {e}")
                U = np.zeros(3)
                print(f"agents_detections[id][itr]:\n{agents_detections[id][itr]}")

            # fmt: off
            ux = (1 / g) * (
                # -alpha * Lap[id, :].dot(gt_x[0:numAgents, itr])
                -alpha * U[0]
                # - gamma * gt_x[numAgents + id : numAgents + (id + 1), itr]
                - gamma * vel_in_body[0]
            ) * 180/np.pi*10 #  to perentage

            uy = (1 / g) * (
                # -alpha * Lap[id, :].dot(gt_y[0:numAgents, itr])
                -alpha * U[1]
                # - gamma * gt_y[numAgents + id : numAgents + (id + 1), itr]
                - gamma * vel_in_body[1] + vdot_ref
            ) * 180/np.pi*10 #  to perentage

            action_ux[id, itr] = ux
            action_uy[id, itr] = uy

            cmd_z1 = pid_z[id].update(
                gt_z[id, itr], cfg.Pstar.z[id], tracking_error=None, current_time=time.time()
            )

            swarm.cmdAngleZ(IP, roll=-uy, pitch=ux, throttle=cmd_z1, yawRate=0)

            T_sl = Ts - (time.time() - t0_)
            time.sleep(T_sl if T_sl > 0 else 0)
            # print(f"Agent {id+1}: Time taken for iteration {itr} is {time.time() - t0_}")
            timestamps[id] += [time.time() - t0_]  # store the timestamp of each iteration

    except Exception as e:
        print(f"Exception occurred in agent {name} at iteration {itr}: {e}")


def localizer_from_dets(dets_xywh, id, R_B2W_gt, itr):

    if len(dets_xywh) == 0:
        print(f"No detection by agent {id+1}! at itr {itr}")
        yp = None
    else:
        for i, tg in enumerate(cfg.targets.class_index):
            _dets_xywh_conf = dets_xywh[dets_xywh[..., -1] == tg][..., :-1]

            if len(_dets_xywh_conf) == 0:
                if tg == ref_id:
                    yp = None
            else:
                yp = []
                for xywh_conf in _dets_xywh_conf:
                    P_b2o, depth = localizer.relative_pos_from_bbox(
                        xywh_conf[..., :-1], R_B2W_gt, cfg.targets.object_size[i]
                    )
                    if tg == ref_id:
                        yp.append((P_b2o, xywh_conf[-1]))

                    data = {
                        "P_b2o": P_b2o.tolist(),
                        "depth": depth,
                        "bbox_conf": xywh_conf.tolist(),
                    }

                    if tg not in agents_detections[id][itr]:
                        agents_detections[id][itr][tg] = {0: data}
                    else:
                        j = len(agents_detections[id][itr][tg]) + 1
                        agents_detections[id][itr][tg][j] = data

    return yp


if swarm.STREAM:

    # display_thread = threading.Thread(
    #     target=display_images, args=(img_queue, stop_display)
    # )
    # display_thread.daemon = True
    # display_thread.start()

    numCores = 8
    # Create a ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=numCores) as executor:
        # Submit tasks to the executor
        t0_thread = time.time()
        futures = [
            executor.submit(run_consensus, i, name, max_itr, Ts)
            for i, name in enumerate(DronesDict)
        ]
        # print(f"Time taken to run futures is {time.time() - t0_thread}")
        # wait for all futures to complete
        concurrent.futures.wait(futures)
        print(
            f"Time taken to complete the coordination task is {time.time() - t0_thread}"
        )
stop_display.set()
# display_thread.join()
cv2.destroyAllWindows()

for id in range(numAgents):
    print(
        f"Agent {id+1} - Number of missed measurements: {EKF[id].missed_measurements}"
    )

import pickle

for id in range(numAgents):
    EKF_x = np.zeros((max_itr + 1, 6))
    EKF_P = np.zeros((max_itr + 1, 6, 6))
    EKF_S = list(EKF[id].tracklet_S)
    EKF_x[:, :] = EKF[id].tracklet_x
    EKF_P[:, :] = EKF[id].tracklet_P

    with open(f"results/experiment_{cfg.exptNum}_EKF_{id}.npz", "wb") as savedata:
        np.savez(
            savedata,
            EKF_x=EKF_x,
            EKF_P=EKF_P,
        )
    with open(f"results/experiment_{cfg.exptNum}_EKF_{id}.pkl", "wb") as savedata:
        pickle.dump(EKF_S, savedata)


if save_data:
    import json

    with open(f"results/experiment_{cfg.exptNum}.json", "w") as f:
        json.dump(agents_detections, f, indent=4)

    with open(f"results/experiment_{cfg.exptNum}_timestamps.json", "w") as f:
        json.dump(timestamps, f, indent=4)

    with open(f"results/experiment_{cfg.exptNum}.npz", "wb") as savedata:
        np.savez(
            savedata,
            gt_x=gt_x,
            gt_y=gt_y,
            gt_z=gt_z,
            gt_roll=gt_roll,
            gt_pitch=gt_pitch,
            gt_yaw=gt_yaw,
            gt_wx=gt_wx,
            gt_wy=gt_wy,
            gt_wz=gt_wz,
            action_ux=action_ux,
            action_uy=action_uy,
            action_uz=action_uz,
            imu_vx=imu_vx,
            imu_vy=imu_vy,
            imu_vz=imu_vz,
            imu_roll=imu_roll,
            imu_pitch=imu_pitch,
            imu_yaw=imu_yaw,
        )


swarm.land()

swarm.disconnect(streamoff=False)
