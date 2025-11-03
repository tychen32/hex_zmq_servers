#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"

from hex_zmq_servers import HexLaunch, HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT

# device config
GELLO_DEVICE = "/dev/ttyUSB0"

# node configs
NODE_CFGS = [
    {
        "name":
        "robot_gello_cli",
        "venv":
        f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/robot_gello/cli.py",
        "cfg_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/robot_gello/cli.json",
    },
    {
        "name": "robot_gello_srv",
        "venv": f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_gello"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_gello"],
        "cfg": {
            "params": {
                "idxs": [0, 1, 2, 3, 4, 5, 6],
                "invs": [1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -3.5],
                "limits": [
                    [-2.7, 2.7],
                    [-1.57, 2.09],
                    [0, 3.14],
                    [-1.57, 1.57],
                    [-1.57, 1.57],
                    [-1.57, 1.57],
                    [0.0, 1.33],
                ],
                "device":
                GELLO_DEVICE,
                "baudrate":
                115200,
                "max_retries":
                3,
                "sens_ts":
                True,
            },
        },
    },
]


def main():
    launch = HexLaunch(NODE_CFGS)
    launch.run()


if __name__ == '__main__':
    main()
