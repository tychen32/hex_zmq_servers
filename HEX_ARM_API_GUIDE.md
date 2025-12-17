# HEX 机械臂 API 使用指南

## 1. 架构概述

`hex_zmq_servers` 是基于 ZeroMQ 构建的分布式设备控制框架。它采用客户端-服务器架构:
- **服务器端**: 处理硬件通信,运行在机械臂连接的机器上
- **客户端**: 发送控制命令并接收状态更新,可以在网络中的任何机器上运行

### 核心组件

1. **ZMQ Base Layer** (`zmq_base.py`):
   - `HexZMQClientBase`: REQ socket 客户端,支持自动重连
   - `HexZMQServerBase`: ROUTER-DEALER 代理,默认4个工作线程
   - `HexSafeValue`: 线程安全的值容器
   - `HexRate`: 精确的频率控制

2. **Device Base Layer** (`device_base.py`):
   - `HexDeviceBase`: 硬件设备抽象基类
   - `HexRobotBase`: 机器人设备基类

3. **通信协议**:
   - 消息结构: JSON 头部 + numpy 缓冲区(多部分消息)
   - 序列号跟踪防止过时/重复数据
   - 使用模 `1e12` 算法处理序列号回绕

## 2. 基础 API 方法

`HexRobotHexarmClient` 继承自 `HexRobotClientBase`,提供以下核心方法:

### 2.1 初始化客户端

```python
from hex_zmq_servers import HexRobotHexarmClient

# 使用默认配置(本地连接 127.0.0.1:12345)
client = HexRobotHexarmClient()

# 或自定义网络配置
net_config = {
    "ip": "192.168.1.100",  # 服务器 IP
    "port": 12345,           # ZMQ 端口
    "client_timeout_ms": 200,
    "server_timeout_ms": 1000,
}
client = HexRobotHexarmClient(net_config=net_config)
```

### 2.2 检查服务器状态

```python
# 检查服务器是否正在运行
response = client.is_working()
if response is not None and response["cmd"] == "is_working_ok":
    print("Server is ready")
```

### 2.3 获取机械臂信息

```python
# 获取自由度数量
dofs_header, dofs = client.get_dofs()
print(f"DOFs: {dofs[0]}")  # 例如: 6 或 7 (6轴 + 夹爪)

# 获取关节限位
limits_header, limits = client.get_limits()
# limits shape: (N, 2) - N 个关节,每个关节 [lower_bound, upper_bound]
print(f"Joint limits:\n{limits}")
```

### 2.4 获取机械臂状态

```python
# 获取当前状态(位置、速度、力矩)
states_hdr, states = client.get_states()

if states_hdr is not None and states_hdr["cmd"] == "get_states_ok":
    # states shape: (N, 3) - N个关节 × [位置, 速度, 力矩]
    positions = states[:, 0]  # 关节位置(弧度)
    velocities = states[:, 1]  # 关节速度(rad/s)
    torques = states[:, 2]     # 关节力矩(N·m)

    # 获取时间戳和序列号
    seq_num = states_hdr["args"]
    timestamp = states_hdr["ts"]  # {"s": seconds, "ns": nanoseconds}

    print(f"Sequence: {seq_num}")
    print(f"Positions: {positions}")
    print(f"Velocities: {velocities}")
    print(f"Torques: {torques}")
```

### 2.5 发送控制命令

```python
import numpy as np

# 创建命令数组(与 DOFs 数量一致)
# 对于 6 轴 + 夹爪的配置,需要 7 个值
cmds = np.array([
    0.5,   # 关节 1 目标位置(rad)
    -1.5,  # 关节 2 目标位置(rad)
    3.0,   # 关节 3 目标位置(rad)
    0.0,   # 关节 4 目标位置(rad)
    0.0,   # 关节 5 目标位置(rad)
    0.0,   # 关节 6 目标位置(rad)
    0.5,   # 夹爪位置(如果有)
])

# 发送命令
success = client.set_cmds(cmds)
if success:
    print("Command sent successfully")
else:
    print("Command failed")
```

## 3. 完整示例代码

以下是用于 3D 重建、分割和抓取的完整示例:

```python
#!/usr/bin/env python3
import numpy as np
import time
from hex_zmq_servers import (
    HexRobotHexarmClient,
    HexRate,
    hex_zmq_ts_now,
    hex_zmq_ts_delta_ms,
)

class HexArmController:
    def __init__(self, net_config=None):
        """初始化机械臂控制器"""
        if net_config is None:
            net_config = {"ip": "127.0.0.1", "port": 12345}

        self.client = HexRobotHexarmClient(net_config=net_config)
        self.dofs = None
        self.limits = None

        # 等待服务器就绪
        self._wait_for_server()

        # 获取机械臂参数
        self._get_robot_info()

    def _wait_for_server(self, timeout=10):
        """等待服务器准备好"""
        print("Waiting for robot server...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            response = self.client.is_working()
            if response is not None and response["cmd"] == "is_working_ok":
                print("Robot server is ready!")
                return True
            time.sleep(0.5)

        raise TimeoutError("Robot server did not respond in time")

    def _get_robot_info(self):
        """获取机械臂信息"""
        # 获取自由度
        _, dofs_array = self.client.get_dofs()
        self.dofs = int(dofs_array[0])

        # 获取限位
        _, self.limits = self.client.get_limits()

        print(f"Robot DOFs: {self.dofs}")
        print(f"Joint limits:\n{self.limits}")

    def get_current_state(self):
        """获取当前状态"""
        hdr, states = self.client.get_states()

        if hdr is not None and hdr["cmd"] == "get_states_ok":
            return {
                "positions": states[:, 0],
                "velocities": states[:, 1],
                "torques": states[:, 2],
                "timestamp": hdr["ts"],
                "sequence": hdr["args"],
            }
        return None

    def move_to_position(self, target_positions):
        """移动到目标位置

        Args:
            target_positions: numpy array of target joint positions (in radians)
        """
        if len(target_positions) != self.dofs:
            raise ValueError(f"Expected {self.dofs} joint commands, got {len(target_positions)}")

        # 确保在限位范围内
        clamped_positions = self._clamp_to_limits(target_positions)

        # 发送命令
        success = self.client.set_cmds(clamped_positions)
        return success

    def _clamp_to_limits(self, positions):
        """将位置限制在关节限位内"""
        clamped = np.copy(positions)
        for i in range(len(positions)):
            lower, upper = self.limits[i]
            clamped[i] = np.clip(positions[i], lower, upper)
        return clamped

    def move_smooth(self, target_positions, duration=2.0, hz=500):
        """平滑移动到目标位置

        Args:
            target_positions: 目标关节位置
            duration: 移动持续时间(秒)
            hz: 控制频率
        """
        # 获取当前位置
        current_state = self.get_current_state()
        if current_state is None:
            print("Failed to get current state")
            return False

        start_positions = current_state["positions"]

        # 生成轨迹
        rate = HexRate(hz)
        steps = int(duration * hz)

        for step in range(steps):
            # 线性插值
            alpha = step / steps
            interpolated = start_positions + alpha * (target_positions - start_positions)

            # 发送命令
            self.move_to_position(interpolated)
            rate.sleep()

        # 最终位置
        self.move_to_position(target_positions)
        return True

    def close_gripper(self, position=0.0):
        """关闭夹爪"""
        current_state = self.get_current_state()
        if current_state is None:
            return False

        target = current_state["positions"].copy()
        target[-1] = position  # 最后一个关节是夹爪
        return self.move_to_position(target)

    def open_gripper(self, position=1.0):
        """打开夹爪"""
        return self.close_gripper(position)


# === 使用示例 ===
def main():
    # 创建控制器
    controller = HexArmController(net_config={"ip": "127.0.0.1", "port": 12345})

    # 获取当前状态
    state = controller.get_current_state()
    print(f"Current positions: {state['positions']}")

    # 移动到目标位置
    target = np.array([0.5, -1.0, 2.0, 0.0, 0.0, 0.0, 0.5])
    controller.move_smooth(target, duration=3.0)

    # 抓取演示
    print("Opening gripper...")
    controller.open_gripper(1.0)
    time.sleep(1.0)

    print("Closing gripper...")
    controller.close_gripper(0.0)
    time.sleep(1.0)

    # 持续监控状态
    rate = HexRate(10)  # 10Hz
    for i in range(100):
        state = controller.get_current_state()
        if state:
            print(f"Step {i}: pos={state['positions'][:3]}")  # 只打印前3个关节
        rate.sleep()


if __name__ == "__main__":
    main()
```

## 4. 用于 3D 重建和抓取的集成建议

对于 3D 重建、分割、抓取应用场景,建议这样集成:

```python
class RoboticGraspingSystem:
    def __init__(self):
        # 初始化机械臂控制器
        self.arm_controller = HexArmController()

        # TODO: 初始化相机(可能是 HexCamClient)
        # self.camera = HexCamBerxelClient()

        # TODO: 初始化你的 3D 重建模块
        # self.reconstructor = Your3DReconstructor()

        # TODO: 初始化分割模块
        # self.segmentor = YourSegmentor()

        # TODO: 初始化抓取规划器
        # self.grasp_planner = YourGraspPlanner()

    def capture_scene(self):
        """捕获场景数据"""
        # 获取 RGB 和深度图像
        # rgb = self.camera.get_rgb()
        # depth = self.camera.get_depth()
        # return rgb, depth
        pass

    def reconstruct_3d(self, rgb, depth):
        """3D 重建"""
        # point_cloud = self.reconstructor.reconstruct(rgb, depth)
        # return point_cloud
        pass

    def segment_objects(self, rgb):
        """物体分割"""
        # masks = self.segmentor.segment(rgb)
        # return masks
        pass

    def plan_grasp(self, point_cloud, mask):
        """规划抓取姿态"""
        # grasp_pose = self.grasp_planner.plan(point_cloud, mask)
        # return grasp_pose
        pass

    def execute_grasp(self, grasp_pose):
        """执行抓取"""
        # 1. 移动到预抓取位置
        pre_grasp_joints = self._ik_solve(grasp_pose.pre_grasp)
        self.arm_controller.move_smooth(pre_grasp_joints)

        # 2. 打开夹爪
        self.arm_controller.open_gripper()

        # 3. 移动到抓取位置
        grasp_joints = self._ik_solve(grasp_pose.grasp)
        self.arm_controller.move_smooth(grasp_joints)

        # 4. 关闭夹爪
        self.arm_controller.close_gripper()

        # 5. 提升物体
        lift_joints = grasp_joints.copy()
        lift_joints[2] -= 0.3  # 假设关节2控制高度
        self.arm_controller.move_smooth(lift_joints)

    def _ik_solve(self, cartesian_pose):
        """逆运动学求解(需要你自己实现或使用库)"""
        # 将笛卡尔空间姿态转换为关节空间
        # joint_angles = your_ik_solver(cartesian_pose)
        # return joint_angles
        pass
```

## 5. 启动服务器

在运行客户端代码之前,需要先启动服务器。参考 `examples/basic/robot_hexarm/launch.py`:

### 5.1 修改配置

```python
# 机器人模型配置
ARM_TYPE = "archer_l6y"  # 根据你的机械臂型号修改
GRIPPER_TYPE = "gp100"   # 根据你的夹爪型号修改
if GRIPPER_TYPE == "empty":
    USE_GRIPPER = False
else:
    USE_GRIPPER = True

# 设备配置
DEVICE_IP = "10.42.0.101"      # 修改为你的机械臂 IP
HEXARM_DEVICE_PORT = 8439       # CAN0=8439, CAN1=9439
```

### 5.2 运行启动脚本

```bash
cd examples/basic/robot_hexarm
source ../../../.venv/bin/activate
python launch.py
```

## 6. 核心 API 参考

### HexRobotClientBase 方法

| 方法 | 参数 | 返回值 | 描述 |
|------|------|--------|------|
| `__init__` | `net_config: dict` | - | 初始化客户端 |
| `is_working` | - | `dict` | 检查服务器状态 |
| `get_dofs` | - | `(header, ndarray)` | 获取自由度数量 |
| `get_limits` | - | `(header, ndarray)` | 获取关节限位 |
| `get_states` | - | `(header, ndarray)` | 获取当前状态 |
| `set_cmds` | `cmds: ndarray` | `bool` | 发送控制命令 |
| `close` | - | - | 关闭连接 |

### 通信消息格式

**请求结构**:
```python
{
    "cmd": str,           # 命令名称
    "ts": dict,           # 时间戳 {"s": seconds, "ns": nanoseconds}
    "args": Any,          # 可选参数(序列号等)
    "dtype": str,         # 缓冲区数据类型
    "shape": tuple,       # numpy 数组形状
}
```

**响应结构**:
- Header: `{"cmd": "command_ok/command_failed", "ts": {...}, "args": sequence_num}`
- Buffer: 包含请求数据的 Numpy 数组

### 状态数据结构

`get_states()` 返回的状态数组格式:
```python
# shape: (N, 3) 其中 N = 关节数量
states[:, 0]  # 位置 (radians)
states[:, 1]  # 速度 (rad/s)
states[:, 2]  # 力矩 (N·m)
```

## 7. 关键注意事项

1. **序列号机制**: 所有状态和命令使用序列号防止过时数据,使用模 `1e12` 算法
2. **时间戳**: 命令包含时间戳用于同步和延迟测量
3. **线程安全**: 使用 `HexSafeValue` 在设备线程和服务器线程间安全传递数据
4. **角度单位**: 所有角度使用弧度(radians)
5. **坐标归一化**: `_rads_normalize()` 将角度归一化到 [-π, π]
6. **限位保护**: `_apply_pos_limits()` 自动限制关节在安全范围内
7. **控制频率**: 推荐使用 500Hz 用于实时控制,10-50Hz 用于监控
8. **网络延迟**: 默认客户端超时 200ms,服务器超时 1000ms
9. **安全第一**: 始终确保机械臂周围有足够的安全空间,并可随时切断电源

## 8. 常见问题

### Q: 如何连接远程机械臂?
A: 修改 `net_config` 中的 IP 地址为服务器所在机器的 IP。

### Q: 如何提高控制频率?
A: 使用 `HexRate` 类精确控制循环频率。注意网络延迟可能限制实际可达频率。

### Q: 如何实现笛卡尔空间控制?
A: 需要实现逆运动学(IK)求解器,将笛卡尔坐标转换为关节角度。可以使用 PyBullet、MoveIt 等库。

### Q: 序列号回绕怎么办?
A: 框架自动处理序列号回绕(模 `1e12`),无需手动处理。

### Q: 如何处理通信失败?
A: 客户端会自动重连。检查返回的 header 中 `cmd` 字段是否为 `*_ok` 或 `*_failed`。

## 9. 相关文件路径

- **客户端基类**: `hex_zmq_servers/robot/robot_base.py`
- **HexArm 客户端**: `hex_zmq_servers/robot/hexarm/robot_hexarm_cli.py`
- **ZMQ 基础**: `hex_zmq_servers/zmq_base.py`
- **示例代码**: `examples/basic/robot_hexarm/`
- **配置文件**: `examples/basic/robot_hexarm/cli.json`

## 10. 参考资源

- CLAUDE.md: 项目架构和开发指南
- examples/: 各种设备的使用示例
- hex_device_python: 底层硬件驱动库(依赖项)
