# hex_device Python 库使用指南

## 概述

`hex_device` 是 HEXFELLOW 的底层硬件驱动库,提供了直接与硬件设备通信的接口。它使用 Protocol Buffers 进行消息序列化,使用 WebSocket 进行实时通信。

**重要关系**:
- `hex_device` (底层库): 直接与硬件通信的 Python 库
- `hex_zmq_servers` (上层框架): 基于 `hex_device` 构建的 ZeroMQ 分布式控制框架

## 安装信息

### 已安装版本
```bash
# 在你的环境中已安装
Package: hex_device
Version: 1.2.2
Location: .venv/lib/python3.11/site-packages/hex_device/
```

### 从 PyPI 安装
```bash
pip install hex_device
```

### 从源码安装
```bash
git clone --recurse-submodules https://github.com/hexfellow/hex_device_python.git
cd hex_device_python
pip install .
```

## 支持的硬件

根据 METADATA 文件,`hex_device` 支持以下硬件:

- ✅ **ChassisMaver**: 底盘系统
- ✅ **ChassisMark2**: 底盘系统第二代
- ✅ **ChassisTriggerA3**: Trigger A3 底盘
- ✅ **ArmArcher**: Archer 系列机械臂
- ✅ **ArmSaber**: Saber 系列机械臂
- ✅ **HandsHtGp100**: GP100 夹爪
- 🚧 **hex_lift**: 升降装置(开发中)

## 核心组件

### 1. 设备基类

```python
from hex_device import DeviceBase, OptionalDeviceBase

# DeviceBase: 所有设备的基类
# OptionalDeviceBase: 可选设备的基类
```

### 2. 设备工厂

```python
from hex_device import DeviceFactory

# 用于创建和管理设备实例
```

### 3. 电机控制

```python
from hex_device import (
    MotorBase,        # 电机基类
    MotorError,       # 电机错误
    MotorCommand,     # 电机命令
    CommandType,      # 命令类型
    MitMotorCommand   # MIT 电机命令
)
```

### 4. 机械臂配置系统

```python
from hex_device import (
    ArmConfig,                    # 机械臂配置类
    ArmConfigManager,             # 配置管理器
    DofType,                      # 自由度类型
    JointParam,                   # 关节参数
    JointParams,                  # 关节参数集合
    load_default_arm_config,      # 加载默认配置
    get_arm_config,               # 获取配置
    add_arm_config,               # 添加配置
    arm_config_manager,           # 全局配置管理器
    set_arm_initial_positions,    # 设置初始位置
    set_arm_initial_velocities,   # 设置初始速度
    clear_arm_position_history,   # 清除位置历史
    clear_arm_velocity_history,   # 清除速度历史
    clear_arm_motion_history,     # 清除运动历史
    get_arm_last_positions,       # 获取最后位置
    get_arm_last_velocities       # 获取最后速度
)
```

### 5. 设备实现

```python
from hex_device import Chassis, Hands

# Chassis: 底盘控制
# Hands: 夹爪/手控制
```

### 6. API 工具

```python
from hex_device import HexDeviceApi

# 提供高级 API 接口
```

### 7. 错误类型

```python
from hex_device import WsError, ProtocolError

# WsError: WebSocket 错误
# ProtocolError: 协议错误
```

### 8. 日志系统

```python
import hex_device
import logging

# 设置日志级别
hex_device.set_log_level(logging.INFO)
# 或使用字符串
hex_device.set_log_level('DEBUG')

# 获取日志记录器
logger = hex_device.get_logger()
logger.info("Custom log message")
```

## Arm 类详细说明

### 支持的机械臂类型

根据 `arm.py` 源码:

```python
SUPPORTED_ROBOT_TYPES = [
    RobotType.RtArmArcherD6Y,   # Archer D6Y (6轴)
    RobotType.RtArmSaberD6X,    # Saber D6X (6轴)
    RobotType.RtArmSaberD7X,    # Saber D7X (7轴)
]

# 系列号到机器人类型的映射
ARM_SERIES_TO_ROBOT_TYPE = {
    9: RtArmSaber750d3Lr3DmDriver,
    10: RtArmSaber750d4Lr3DmDriver,
    11: RtArmSaber750h3Lr3DmDriver,
    12: RtArmSaber750h4Lr3DmDriver,
    14: RtArmSaberD6X,
    15: RtArmSaberD7X,
    16: RtArmArcherD6Y,
}
```

### Arm 类初始化参数

```python
from hex_device import Arm

arm = Arm(
    robot_type,              # 机器人类型(从上述列表中选择)
    motor_count,             # 电机数量
    name="Arm",              # 设备名称(可选)
    control_hz=500,          # 控制频率,默认 500Hz
    send_message_callback    # 发送消息的回调函数
)
```

### Arm 类主要属性

```python
# 控制频率
arm._control_hz = 500
arm._period = 1.0 / control_hz

# 机械臂状态
arm._arm_mode           # 机械臂模式(刹车/运行等)
arm._calibrated         # 是否已校准
arm._session_holder     # 会话持有者
arm._my_session_id      # 当前会话ID

# 命令超时检查
arm._command_timeout_check = True
arm._command_timeout = 0.3  # 300ms 超时
```

## hex_device vs hex_zmq_servers

### hex_device (底层库)
- **通信方式**: WebSocket + Protocol Buffers
- **使用场景**: 直接连接到硬件设备
- **连接方式**: 点对点连接
- **适用于**: 单机控制、硬件驱动开发

### hex_zmq_servers (上层框架)
- **通信方式**: ZeroMQ + JSON + NumPy
- **使用场景**: 分布式系统、多进程架构
- **连接方式**: 客户端-服务器架构,支持多个客户端
- **适用于**: 机器人应用开发、远程控制、多设备协同

### 架构关系

```
┌─────────────────────────────────────────────────┐
│         你的应用程序                              │
│    (3D重建、分割、抓取等)                         │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│         hex_zmq_servers                         │
│    (HexRobotHexarmClient)                       │
│    - ZeroMQ 通信                                 │
│    - 分布式架构                                   │
│    - 多客户端支持                                 │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│         hex_zmq_servers                         │
│    (HexRobotHexarmServer)                       │
│    - 服务器端                                     │
│    - 线程池处理                                   │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│         hex_device                              │
│    (Arm 类)                                      │
│    - WebSocket 通信                              │
│    - Protocol Buffers                           │
│    - 直接硬件控制                                 │
└─────────────────┬───────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────┐
│         HEX 机械臂硬件                            │
│    (通过 CAN 总线连接)                            │
└─────────────────────────────────────────────────┘
```

## 在 hex_zmq_servers 中的使用

查看 `hex_zmq_servers/robot/hexarm/` 目录下的实现,可以看到:

1. **robot_hexarm_dev.py**: 使用 `hex_device.Arm` 创建设备实例
2. **robot_hexarm_srv.py**: 包装设备为 ZMQ 服务器
3. **robot_hexarm_cli.py**: 提供 ZMQ 客户端接口

### 示例:设备层实现

```python
# 在 hex_zmq_servers 中的典型使用
from hex_device import Arm, get_arm_config

class HexRobotHexarm(HexRobotBase):
    def __init__(self, device_ip, arm_type, use_gripper, control_hz):
        # 获取机械臂配置
        arm_config = get_arm_config(arm_type)

        # 创建 Arm 实例
        self.arm = Arm(
            robot_type=arm_config.robot_type,
            motor_count=arm_config.motor_count,
            control_hz=control_hz,
            send_message_callback=self._send_callback
        )

        # 连接到硬件
        self.arm.connect(device_ip)

    def work_loop(self, hex_values):
        # 从硬件读取状态
        states = self.arm.get_states()

        # 发送命令到硬件
        self.arm.set_commands(commands)
```

## 开发建议

### 对于应用开发者(你的情况)

**推荐使用 `hex_zmq_servers`**:

优点:
- ✅ 更高层次的抽象
- ✅ 简单的 API(get_states, set_cmds 等)
- ✅ 自动处理线程安全
- ✅ 支持远程控制
- ✅ 序列号跟踪防止过时数据
- ✅ 自动重连机制

示例:
```python
from hex_zmq_servers import HexRobotHexarmClient

client = HexRobotHexarmClient()
states_hdr, states = client.get_states()
client.set_cmds(commands)
```

### 对于驱动开发者

**需要使用 `hex_device`**:

使用场景:
- 开发新的设备驱动
- 需要直接访问硬件特性
- 实现自定义通信协议
- 底层性能优化

示例:
```python
from hex_device import Arm

arm = Arm(robot_type, motor_count, control_hz=500)
arm.connect(device_ip)
```

## 依赖关系

### hex_device 依赖
```
numpy>=1.17.4
protobuf>=5.29.4,<6.0.0
websockets>=13.1
```

### hex_zmq_servers 依赖
```
pyzmq>=27.0.1
opencv-python>=4.2
mujoco>=3.3.3
hex_device>=1.2.1          # 依赖于 hex_device
hex_robo_utils>=0.1.18
dynamixel-sdk==3.8.4
```

## 相关资源

### hex_device 资源
- **GitHub**: https://github.com/hexfellow/hex_device_python
- **Wiki**: https://github.com/hexfellow/hex_device_python/wiki
- **API 列表**: https://github.com/hexfellow/hex_device_python/wiki/API-List
- **Change Log**: https://github.com/hexfellow/hex_device_python/wiki/Change-Log

### hex_zmq_servers 资源
- **GitHub**: https://github.com/hexfellow/hex_zmq_servers
- **本地文档**: CLAUDE.md, HEX_ARM_API_GUIDE.md

### 示例代码
- **hex_device 示例**: `tests/main.py`, `tests/archer_traj_test.py`
- **hex_zmq_servers 示例**: `examples/basic/robot_hexarm/`

## 常见问题

### Q: 我应该使用哪个库?
A:
- **应用开发**: 使用 `hex_zmq_servers` (HexRobotHexarmClient)
- **驱动开发**: 使用 `hex_device` (Arm 类)

### Q: hex_device 和 hex_zmq_servers 可以同时使用吗?
A: 不建议。`hex_zmq_servers` 已经封装了 `hex_device`,直接使用上层接口即可。

### Q: 如何更新 hex_device?
A:
```bash
pip install --upgrade hex_device
```

### Q: 跨版本兼容性问题?
A: 请查看 hex_device 的 Change Log,确保硬件固件版本与软件包版本匹配。

### Q: 需要硬件升级怎么办?
A: 联系 HEXFELLOW 售后服务,获取硬件升级说明。

## 总结

- `hex_device` 是底层硬件驱动库,使用 WebSocket + Protobuf
- `hex_zmq_servers` 是基于 `hex_device` 的分布式控制框架,使用 ZeroMQ
- 对于机器人应用开发(包括你的 3D 重建和抓取项目),推荐使用 `hex_zmq_servers`
- `hex_device` 主要用于驱动开发和底层硬件访问

**你的开发路径**:
```
你的应用 → hex_zmq_servers (HexRobotHexarmClient) → [网络] → hex_zmq_servers (Server) → hex_device (Arm) → 硬件
```
