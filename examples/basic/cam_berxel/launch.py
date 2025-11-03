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
# # cam 0
# SERIAL_NUMBER = "P050HYX5410E1A001"
# EXPOSURE = 16000
# # cam 1
# SERIAL_NUMBER = "P050HYX5421E2A004"
# EXPOSURE = 16000
# cam 2
SERIAL_NUMBER = "P100RYB4C03M2B322"
EXPOSURE = 10000

NODE_CFGS = [
    {
        "name": "cam_berxel_cli",
        "venv": f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/cam_berxel/cli.py",
        "cfg_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/cam_berxel/cli.json",
    },
    {
        "name": "cam_berxel_srv",
        "venv": f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_berxel"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_berxel"],
        "cfg": {
            "params": {
                "serial_number": SERIAL_NUMBER,
                "exposure": EXPOSURE,
            },
        },
    },
]


def main():
    launch = HexLaunch(NODE_CFGS)
    launch.run()


if __name__ == '__main__':
    main()
