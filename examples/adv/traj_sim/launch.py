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

from hex_zmq_servers import (
    HexLaunch,
    HEX_ZMQ_SERVERS_PATH_DICT,
    HEX_ZMQ_CONFIGS_PATH_DICT,
    HEXARM_URDF_PATH_DICT,
)

# robot model config
ARM_TYPE = "archer_d6y"
GRIPPER_TYPE = "gp100_p050"

# server ports
MUJOCO_SRV_PORT = 12345

NODE_CFGS = [
    {
        "name": "traj_sim_cli",
        "venv": f"{HEX_ZMQ_SERVERS_DIR}/../.venv",
        "node_path": f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/traj_sim/cli.py",
        "cfg_path": f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/traj_sim/cli.json",
        "cfg": {
            "model_path": HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
            "last_link": "link_6",
            "traj_center": [0.4, 0.0, 0.35, 1.0, 0.0, 0.0, 0.0],
            "traj_radius": 0.2,
            "traj_period": 1.0,
            "traj_center_duration": 0.3,
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
                "tau_ctrl": True,
                "mit_kp": [0.0] * 7,
                "mit_kd": [0.0] * 7,
                "headless": False,
                "sens_ts": True,
            }
        },
    },
]


def main():
    launch = HexLaunch(NODE_CFGS)
    launch.run()


if __name__ == '__main__':
    main()
