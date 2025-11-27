#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT

# device config
# cam 0
CAM_PATH = "/dev/video0"
RESOLUTION = [640, 480]
EXPOSURE = 70
TEMPERATURE = 0
FRAME_RATE = 30
SENS_TS = True

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"
NODE_PARAMS_DICT = {
    # cli
    "cam_rgb_cli": {
        "name": "cam_rgb_cli",
        "node_path": f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/cam_rgb/cli.py",
        "cfg_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/cam_rgb/cli.json",
        "cfg": {
            "net": {
                "ip": "127.0.0.1",
                "port": 12345,
            },
        }
    },
    # srv
    "cam_rgb_srv": {
        "name": "cam_rgb_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["cam_rgb"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["cam_rgb"],
        "cfg": {
            "net": {
                "ip": "127.0.0.1",
                "port": 12345,
            },
            "params": {
                "cam_path": CAM_PATH,
                "resolution": RESOLUTION,
                "exposure": EXPOSURE,
                "temperature": TEMPERATURE,
                "frame_rate": FRAME_RATE,
                "sens_ts": SENS_TS,
            },
        },
    },
}


def get_node_cfgs(node_params_dict: dict = NODE_PARAMS_DICT):
    return HexNodeConfig.parse_node_params_dict(
        node_params_dict,
        NODE_PARAMS_DICT,
    )


def main():
    node_cfgs = get_node_cfgs()
    launch = HexLaunch(node_cfgs)
    launch.run()


if __name__ == '__main__':
    main()
