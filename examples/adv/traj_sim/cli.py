#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json, time, csv, os
import numpy as np
from hex_zmq_servers import (
    HexRate,
    HEX_LOG_LEVEL,
    hex_log,
    HexMujocoArcherD6yClient,
)
from hex_robo_utils import HexDynUtil as DynUtil
from hex_robo_utils import HexCtrlUtilMit as CtrlUtil
from hex_robo_utils import part2trans, trans2part, trans_inv

INIT_JOINT = np.array(
    [0.0, -0.0205679922, 2.57081467, -0.978840246, 0.0, 0.0],
    dtype=np.float64,
)
END_POSE = np.array(
    [0.0, 0.0, 0.12, 0.7071068, 0.0, -0.7071068, 0.0],
    dtype=np.float64,
)
POSE_END_IN_STABLE = [
    np.array(
        [0.0, 0.0, -0.17],
        dtype=np.float64,
    ),
    np.array(
        [0.7071068, 0.0, -0.7071068, 0.0],
        dtype=np.float64,
    ),
]


def wait_client_working(client, timeout: float = 5.0) -> bool:
    for _ in range(int(timeout * 10)):
        working = client.is_working()
        if working is not None and working["cmd"] == "is_working_ok":
            return True
        else:
            time.sleep(0.1)
    return False


def interp_joint(cur_q, tar_joint, err_limit=0.05):
    err = tar_joint - cur_q
    max_err_fab = np.fabs(err).max()
    if max_err_fab < err_limit:
        return tar_joint, False
    else:
        err_norm = err / max_err_fab
        return cur_q + err_norm * err_limit, True


def interp_arm(cur_q, tar_joint, grip_flag, use_gripper=True, err_limit=0.05):
    mid_joint = np.zeros(7 if use_gripper else 6)
    if use_gripper:
        mid_joint[:-1], interp_flag = interp_joint(cur_q[:-1],
                                                   tar_joint,
                                                   err_limit=err_limit)
        mid_joint[-1], _ = interp_joint(
            cur_q[-1],
            1.33 if grip_flag else 0.2,
            err_limit=err_limit,
        )
    else:
        mid_joint, interp_flag = interp_joint(
            cur_q,
            tar_joint[:-1],
            err_limit=err_limit,
        )
    return mid_joint, interp_flag


def create_traj_arr(traj_center, traj_radius, traj_period,
                    traj_center_duration, hz):
    center_pos = traj_center[:3]
    center_quat = traj_center[3:]
    trans_center_in_base = part2trans(center_pos, center_quat)

    trans_start_in_center = part2trans(np.array([
        0.0,
        0.0,
        traj_radius,
    ]), center_quat)
    trans_start_in_base = trans_center_in_base @ trans_start_in_center
    start_pos, _ = trans2part(trans_start_in_base)

    center_num = int(traj_center_duration * hz)
    center_pos_list = np.linspace(center_pos, start_pos, center_num).tolist()
    center_quat_list = [center_quat] * center_num

    circle_pos_list = []
    circle_quat_list = []
    circle_num = int(traj_period * hz)
    for i in range(circle_num):
        theta = 2 * np.pi * i / circle_num
        trans_pt_in_center = part2trans(
            np.array([
                0.0,
                traj_radius * np.sin(theta),
                traj_radius * np.cos(theta),
            ]), center_quat)
        trans_pt_in_base = trans_center_in_base @ trans_pt_in_center
        circle_pos, circle_quat = trans2part(trans_pt_in_base)
        circle_pos_list.append(circle_pos)
        circle_quat_list.append(circle_quat)

    traj_pos_list = center_pos_list + circle_pos_list + center_pos_list[::-1]
    traj_quat_list = center_quat_list + circle_quat_list + center_quat_list[::
                                                                            -1]
    traj_num = center_num + circle_num + center_num
    return np.array(traj_pos_list), np.array(traj_quat_list), traj_num


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        model_path = cfg["model_path"]
        last_link = cfg["last_link"]
        traj_center = np.array(cfg["traj_center"])
        traj_radius = cfg["traj_radius"]
        traj_period = cfg["traj_period"]
        traj_center_duration = cfg["traj_center_duration"]
        csv_dir = cfg["csv_dir"]
        mujoco_net_cfg = cfg["mujoco_net_cfg"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    mujoco_client = HexMujocoArcherD6yClient(net_config=mujoco_net_cfg)
    dyn_util = DynUtil(
        model_path=model_path,
        last_link=last_link,
        end_pose=END_POSE,
    )
    kp = np.array([200.0, 200.0, 200.0, 75.0, 30.0, 30.0, 20.0])
    kd = np.array([5.0, 5.0, 5.0, 5.0, 1.5, 1.5, 1.0])
    ctrl_util = CtrlUtil()

    # wait servers to work
    if not wait_client_working(mujoco_client):
        hex_log(HEX_LOG_LEVEL["err"], "mujoco server is not working")
        return

    # create CSV file for logging
    os.makedirs(csv_dir, exist_ok=True)
    csv_filename = f"{csv_dir}/traj_sim.csv"
    csv_file = open(csv_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)

    # write CSV header
    header = ['ts']
    for i in range(7):
        header.append(f'q{i}')
    for i in range(7):
        header.append(f'dq{i}')
    for i in range(7):
        header.append(f'eff{i}')
    for i in range(7):
        header.append(f'mid_q{i}')
    for i in range(7):
        header.append(f'ik_q{i}')
    csv_writer.writerow(header)
    hex_log(HEX_LOG_LEVEL["info"], f"CSV logging to: {csv_filename}")

    # work loop
    hz = 500.0
    rate = HexRate(hz)
    err_limit = 0.2
    cur_q = None
    tau_comp = np.zeros(7)
    tar_joint = INIT_JOINT.copy()
    traj_pos_arr, traj_quat_arr, traj_num = create_traj_arr(
        traj_center,
        traj_radius,
        traj_period,
        traj_center_duration,
        hz,
    )
    trans_stable_in_end = part2trans(
        POSE_END_IN_STABLE[0],
        POSE_END_IN_STABLE[1],
    )
    trans_end_in_stable = trans_inv(trans_stable_in_end)
    traj_idx = 0
    try:
        while True:
            # current states
            robot_states_hdr, robot_states = mujoco_client.get_states("robot")
            if robot_states_hdr is not None:
                cur_ts = robot_states_hdr["ts"][
                    "s"] * 1_000_000_000 + robot_states_hdr["ts"]["ns"]
                cur_q = robot_states[:, 0]
                cur_dq = robot_states[:, 1]
                cur_eff = robot_states[:, 2]

                arm_q = cur_q[:-1]
                arm_dq = cur_dq[:-1]
                _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
                tau_comp = c_mat @ arm_dq + g_vec
                tau_comp = np.concatenate((tau_comp, np.zeros(1)), axis=0)
            else:
                cur_q = None

            if cur_q is not None:
                # update target pose
                tar_pos, tar_quat = traj_pos_arr[traj_idx], traj_quat_arr[
                    traj_idx]

                # proj pose
                tar_end_in_base = part2trans(tar_pos, tar_quat)
                tar_stable_in_base = tar_end_in_base @ trans_stable_in_end
                tar_stable_pos = tar_stable_in_base[:3, 3].copy()
                tar_stable_dist = np.linalg.norm(tar_stable_pos)
                valid_dist = np.clip(tar_stable_dist, 0.3, 0.7)
                tar_stable_pos = tar_stable_pos / tar_stable_dist * valid_dist
                tar_stable_in_base[:3, 3] = tar_stable_pos
                tar_pos, tar_quat = trans2part(
                    tar_stable_in_base @ trans_end_in_stable)

                # interp joint
                mid_q = cur_q.copy()
                if tar_joint is not None:
                    mid_q, interp_flag = interp_arm(
                        cur_q,
                        tar_joint,
                        grip_flag=False,
                        use_gripper=True,
                        err_limit=err_limit,
                    )
                    # arrive target joint
                    if not interp_flag:
                        tar_joint = None
                else:
                    ik_success, ik_q, _ = dyn_util.inverse_kinematics(
                        (tar_pos, tar_quat), cur_q[:-1])
                    if ik_success:
                        mid_q, _ = interp_arm(
                            cur_q,
                            ik_q,
                            grip_flag=False,
                            use_gripper=True,
                            err_limit=err_limit,
                        )
                    else:
                        tar_pos, tar_quat = dyn_util.forward_kinematics(
                            cur_q[:-1])[-1]

                    # log data to CSV
                    if robot_states_hdr is not None:
                        csv_row = [cur_ts]
                        csv_row.extend(cur_q.tolist())
                        csv_row.extend(cur_dq.tolist())
                        csv_row.extend(cur_eff.tolist())
                        csv_row.extend(mid_q.tolist())
                        csv_row.extend(ik_q.tolist())
                        csv_row.append(0.2)
                        csv_writer.writerow(csv_row)

                    # update traj point
                    traj_idx = (traj_idx + 1) % traj_num

                # set cmds
                cmds = ctrl_util(
                    kp=kp,
                    kd=kd,
                    q_tar=mid_q,
                    dq_tar=np.zeros(7),
                    q_cur=cur_q,
                    dq_cur=cur_dq,
                    tau_comp=tau_comp,
                )
                _ = mujoco_client.set_cmds(cmds)

            rate.sleep()
    finally:
        csv_file.close()
        hex_log(HEX_LOG_LEVEL["info"], f"CSV file closed: {csv_filename}")


if __name__ == '__main__':
    main()
