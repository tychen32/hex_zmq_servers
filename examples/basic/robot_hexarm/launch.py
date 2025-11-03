#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../hex_zmq_servers"

from hex_zmq_servers import HexLaunch, HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT

# robot model config
ARM_TYPE = "archer_l6y"
GRIPPER_TYPE = "gp100"
if GRIPPER_TYPE == "empty":
    USE_GRIPPER = False
else:
    USE_GRIPPER = True

# device config
DEVICE_IP = "192.168.1.101"
HEXARM_DEVICE_PORT = 8439

NODE_CFGS = [
    {
        "name": "hexarm_robot_cli",
        "venv": f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/hex_arm_robot_cs/cli.py",
        "cfg_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/hex_arm_robot_cs/cli.json",
    },
    {
        "name": "robot_hexarm_srv",
        "venv": f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hexarm"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hexarm"],
        "cfg": {
            "params": {
                "device_ip": DEVICE_IP,
                "control_hz": 500,
                "arm_type": ARM_TYPE,
                "use_gripper": USE_GRIPPER,
                "sens_ts": True,
            }
        }
    },
]


def main():
    launch = HexLaunch(NODE_CFGS)
    launch.run()


if __name__ == '__main__':
    main()
