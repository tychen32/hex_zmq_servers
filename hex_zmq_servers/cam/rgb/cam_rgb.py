#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-14
################################################################

import cv2
import threading
import numpy as np

from ..cam_base import HexCamBase
from ...zmq_base import (
    hex_zmq_ts_now,
    HexRate,
    HexSafeValue,
)
from ...hex_launch import hex_log, HEX_LOG_LEVEL

CAMERA_CONFIG = {
    "cam_path": "/dev/video0",
    "resolution": [640, 480],
    "frame_rate": 30,
    "sens_ts": True,
}


class HexCamRGB(HexCamBase):

    def __init__(
        self,
        camera_config: dict = CAMERA_CONFIG,
    ):
        HexCamBase.__init__(self)

        try:
            self.__cam_path = camera_config["cam_path"]
            self.__resolution = camera_config["resolution"]
            self.__frame_rate = camera_config["frame_rate"]
            self.__sens_ts = camera_config["sens_ts"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"camera_config is not valid, missing key: {missing_key}")

        # variables
        # camera variables
        self.__cap = cv2.VideoCapture(self.__cam_path)
        # camera variables
        self.__intri = np.zeros(4)

        # open device
        self.__cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.__cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.__resolution[0])
        self.__cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.__resolution[1])
        self.__cap.set(cv2.CAP_PROP_FPS, self.__frame_rate)

        print("#############################")
        print(
            f"# Resolution: ({self.__cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, {self.__cap.get(cv2.CAP_PROP_FRAME_HEIGHT)})"
        )
        print(f"# FPS: {self.__cap.get(cv2.CAP_PROP_FPS)}")
        four_cc_int = int(self.__cap.get(cv2.CAP_PROP_FOURCC))
        four_cc_str = chr(four_cc_int & 0xff) + chr(
            (four_cc_int >> 8) & 0xff) + chr((four_cc_int >> 16) & 0xff) + chr(
                (four_cc_int >> 24) & 0xff)
        print(f"# FourCC: {four_cc_str}")
        print("#############################")

        # start work loop
        self._working.set()

    def get_intri(self) -> np.ndarray:
        self._wait_for_working()
        return self.__intri

    def work_loop(self, hex_values: list[HexSafeValue | threading.Event]):
        rgb_value = hex_values[0]
        depth_value = hex_values[1]
        stop_event = hex_values[2]

        rgb_count = 0
        depth_value.set((hex_zmq_ts_now(), 0,
                         np.zeros((self.__resolution[1], self.__resolution[0]),
                                  dtype=np.uint16)))
        rate = HexRate(self.__frame_rate * 5)
        while self._working.is_set() and not stop_event.is_set():
            # read frame
            ret, frame = self.__cap.read()

            # collect rgb frame
            if ret:
                rgb_value.set((hex_zmq_ts_now(), rgb_count, frame))
                rgb_count = (rgb_count + 1) % self._max_seq_num

            rate.sleep()

        # close
        self.close()

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        self.__cap.release()
        hex_log(HEX_LOG_LEVEL["info"], "HexCamRGB closed")
