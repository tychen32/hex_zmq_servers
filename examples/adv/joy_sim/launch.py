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

# server ports
MUJOCO_SRV_PORT = 12345

NODE_CFGS = [
    {
        "name": "joy_sim_cli",
        "venv": f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path": f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/joy_sim/cli.py",
        "cfg_path": f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/joy_sim/cli.json",
        "cfg": {
            "model_path":
            f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/urdf/archer_d6y.urdf",
            "last_link": "link_6",
            "mujoco_net_cfg": {
                "port": MUJOCO_SRV_PORT,
            },
        },
    },
    {
        "name": "mujoco_archer_d6y_srv",
        "venv": f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["mujoco_archer_d6y"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["mujoco_archer_d6y"],
        "cfg": {
            "net": {
                "port": MUJOCO_SRV_PORT,
            },
            "params": {
                "states_rate": 250,
                "img_rate": 30,
                "headless": False,
                "sens_ts": True,
            },
        },
    },
]


def main():
    launch = HexLaunch(NODE_CFGS)
    launch.run()


if __name__ == '__main__':
    main()
