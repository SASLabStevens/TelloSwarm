#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 26 11:30:08 2021 - modified on May 2024

@author: Rayan Bahrami

Safe Autonomous Systems Lab (SAS Lab)
Stevens Institute of Technology

"""

import threading
import cv2


class VideoStream:
    """
    Python class to stream video from multiple Ryze Tello drones.
    It is writen to be used along with the TelloServerEDU class.
    It requires the Ryze Tello SDK 3.0 to be installed on Tello.
    [2](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
    [3](https://dl.djicdn.com/downloads/RoboMaster+TT/Tello_SDK_3.0_User_Guide_en.pdf)
    It uses OpenCV to display the video stream.
    """

    def __init__(
        self,
        TELLO_IP: str,  # default is "0.0.0.0" for a single drone
        VIDEO_PORT: int,  # 11111
        bufsize=5,
        FPS=None,  # max is 30
        resolution=(None, None),
        resize=False,  # make it True if you get distorted images for lowe resolution
    ):
        """

        Parameters
        ----------
        TELLO_IP : str, IP address of the Tello drone on the wifi network
        VIDEO_PORT : int, port number on the PC for video stream (default is 11111). This is configurable with TelloServer.set_video_port()
        FPS : int, frame per second (5 - 30), optional
        resolution (H=720,W=960) : tuple, optional

        Returns
        -------
        None.

        """
        self.TELLO_IP = TELLO_IP
        self.VIDEO_PORT = VIDEO_PORT
        self.bufsize = bufsize
        self.FPS = 30 if FPS is None else FPS
        self.resolution = resolution
        self.resize = resize

        self.frame = b"None"  # latest frame received from the drone camera
        # self.lock = threading.Lock()

        # Initialize video stream from Tello

        print(f"Connecting to cv2.VideoCapture(), it may take ~30 seconds ...")
        # https://docs.opencv.org/4.8.0/dd/d43/tutorial_py_video_display.html
        self.video = cv2.VideoCapture(
            f"udp://{self.TELLO_IP}:{self.VIDEO_PORT}", cv2.CAP_ANY
        )
        if not self.video.isOpened():
            # self.video_isOpened = False
            print("Error: VideoCapture not opened!, exit ...")

        else:
            # self.video_isOpened = True
            print(f"Connected! streaming on cv2.VideoCapture() ... ")

            self.video.set(cv2.CAP_PROP_BUFFERSIZE, self.bufsize)
            if FPS is not None:
                self.video.set(cv2.CAP_PROP_FPS, self.FPS)
            if resolution[0] is not None and resize is False:
                self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[0])
            if resolution[1] is not None and resize is False:
                self.video.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[1])

            self.recv_video_thread = threading.Thread(target=self._recv_video)
            self.recv_video_active = True
            self.recv_video_thread.start()

    def _recv_video(self):
        """get video stream from Tello"""
        while self.recv_video_active:
            try:
                ret, frame = self.video.read()  # capture frame-by-frame
                if ret:  # if frame is read correctly
                    if self.resize and self.resolution[0] is not None:
                        frame = cv2.resize(frame, self.resolution[::-1])
                        # self.frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
                    # with self.lock:
                    self.frame = frame
                    # cv2.imshow(f"{self.TELLO_IP}", self.frame)  # bad idea here!
                    # cv2.imwrite(f"{self.TELLO_IP}_at_{time.time()}.png", self.frame)
                    # cv2.waitKey(1)

            except Exception as e:
                print(f"Exception cv2.VideoCapture() error : {e}")

    # =====================  drone's control commands  ====================

    def stop(self):
        """
        closes the UDP channels used for sending commands to a Tello drone
        """
        self.recv_video_active = False
        self.video.release()
        cv2.destroyAllWindows()
        self.recv_video_thread.join()
