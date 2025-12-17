#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# RealSense Camera Client
################################################################

from ..cam_base import HexCamClientBase

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12346,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}


class HexCamRealsenseClient(HexCamClientBase):
    """RealSense camera ZMQ client."""

    def __init__(self, net_config: dict = NET_CONFIG):
        HexCamClientBase.__init__(self, net_config)

    def get_rgb(self):
        """
        Get RGB image from camera.

        Returns:
            tuple: (header, image) where image is numpy array (H, W, 3) uint8,
                   or (None, None) on failure
        """
        return self._process_frame(depth_flag=False)

    def get_depth(self):
        """
        Get depth image from camera.

        Returns:
            tuple: (header, image) where image is numpy array (H, W) uint16 (mm),
                   or (None, None) on failure
        """
        return self._process_frame(depth_flag=True)

    def get_intri(self):
        """
        Get camera intrinsic parameters.

        Returns:
            tuple: (header, intrinsics) where intrinsics is numpy array [fx, fy, cx, cy],
                   or (None, None) on failure
        """
        hdr, intri = self.request({"cmd": "get_intri"})

        try:
            cmd = hdr["cmd"]
            if cmd == "get_intri_ok":
                return hdr, intri
            else:
                print(f"\033[91mget_intri failed: {cmd}\033[0m")
                return None, None
        except KeyError:
            print(f"\033[91mget_intri response missing 'cmd'\033[0m")
            return None, None
        except Exception as e:
            print(f"\033[91mget_intri error: {e}\033[0m")
            return None, None
