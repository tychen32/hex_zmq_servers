# robot_hexarm 示例详细解读

## 概述

`examples/basic/robot_hexarm/` 展示了如何使用 `hex_zmq_servers` 框架控制 HEX 机械臂。该示例采用**多进程架构**,包含一个服务器进程和一个客户端进程。

## 文件结构

```
examples/basic/robot_hexarm/
├── launch.py       # 启动脚本 - 协调服务器和客户端进程
├── cli.py          # 客户端代码 - 发送命令和接收状态
├── cli.json        # 客户端网络配置
├── README.md       # 说明文档
└── logs/           # 日志目录(自动生成)
    ├── info/       # 信息日志
    └── err/        # 错误日志
```

## 架构流程图

```
┌─────────────────────────────────────────────────────────────┐
│  launch.py (主进程)                                          │
│  - 使用 HexLaunch 协调多个子进程                              │
│  - 管理日志、重启、信号处理                                   │
└────────────┬────────────────────────────┬───────────────────┘
             │                            │
             │ 启动                        │ 启动
             ▼                            ▼
┌──────────────────────────┐   ┌──────────────────────────────┐
│  robot_hexarm_srv.py     │   │  cli.py                      │
│  (服务器进程)             │   │  (客户端进程)                 │
│                          │   │                              │
│  1. 创建 HexRobotHexarm  │◄──┤  1. 创建客户端               │
│  2. 连接硬件(WebSocket)  │   │  2. 等待服务器就绪            │
│  3. 启动 ZMQ 服务器      │   │  3. 获取机械臂信息            │
│  4. 处理客户端请求       │   │  4. 循环发送命令/读取状态     │
│  5. 与硬件通信           │   │                              │
└────────────┬─────────────┘   └──────────────────────────────┘
             │
             │ WebSocket + Protobuf
             ▼
┌──────────────────────────┐
│  HEX 机械臂硬件          │
│  IP: 10.42.0.101        │
│  Port: 8439 (CAN0)      │
└──────────────────────────┘
```

## 文件详解

### 1. launch.py - 启动脚本

这是程序的入口点,负责启动和管理所有进程。

#### 关键配置

```python
# 机器人模型配置
ARM_TYPE = "archer_l6y"      # 机械臂型号
GRIPPER_TYPE = "gp100"       # 夹爪型号
USE_GRIPPER = True           # 是否使用夹爪

# 设备连接配置
DEVICE_IP = "10.42.0.101"    # 机械臂的 IP 地址
HEXARM_DEVICE_PORT = 8439    # 端口 (CAN0=8439, CAN1=9439)
```

#### 节点配置

```python
NODE_CFGS = [
    # 节点1: 客户端进程
    {
        "name": "hexarm_robot_cli",           # 进程名称
        "venv": ".venv",                      # 虚拟环境路径
        "node_path": "cli.py",                # 脚本路径
        "cfg_path": "cli.json",               # 配置文件路径
    },
    # 节点2: 服务器进程
    {
        "name": "robot_hexarm_srv",
        "venv": ".venv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hexarm"],  # 服务器脚本
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hexarm"],   # 服务器配置
        "cfg": {                              # 运行时覆盖配置
            "params": {
                "device_ip": DEVICE_IP,
                "control_hz": 500,            # 控制频率 500Hz
                "arm_type": ARM_TYPE,
                "use_gripper": USE_GRIPPER,
                "sens_ts": True,              # 启用时间戳感知
            }
        }
    },
]
```

#### 工作流程

1. **创建 HexLaunch 实例**: `launch = HexLaunch(NODE_CFGS)`
2. **启动所有节点**: `launch.run()`
   - 为每个节点创建独立的子进程
   - 设置日志重定向
   - 配置信号处理(Ctrl+C)
   - 可选的进程自动重启

#### 配置合并机制

配置按以下优先级合并(后者覆盖前者):
1. 服务器默认配置(在 `robot_hexarm_srv.py` 中)
2. JSON 配置文件(`robot_hexarm.json`)
3. 运行时 `cfg` 覆盖(在 `NODE_CFGS` 中)

**示例**:
```python
# 默认配置: device_ip = "172.18.8.161"
# JSON 配置: device_ip = "172.18.8.161"
# 运行时覆盖: device_ip = "10.42.0.101"  ← 最终使用这个
```

---

### 2. cli.json - 客户端网络配置

```json
{
    "net": {
        "ip": "127.0.0.1",              // ZMQ 服务器地址
        "port": 12345,                  // ZMQ 服务器端口
        "client_timeout_ms": 200,       // 客户端请求超时(200ms)
        "server_timeout_ms": 1000,      // 服务器处理超时(1s)
        "server_num_workers": 4         // 服务器工作线程数
    }
}
```

**注意**: 这里的 IP/端口是 ZMQ 通信的配置,不是机械臂硬件的 IP。

---

### 3. cli.py - 客户端实现

这是客户端的完整实现,展示了如何使用 API。

#### 3.1 初始化和连接

```python
# 第 22-33 行: 解析配置
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)  # launch.py 会传入配置
    args = parser.parse_args()
    cfg = json.loads(args.cfg)  # 解析 JSON 配置

    net_config = cfg["net"]  # 提取网络配置
    client = HexRobotHexarmClient(net_config=net_config)  # 创建客户端
```

**关键点**:
- `--cfg` 参数由 `HexLaunch` 自动传入,包含 `cli.json` 的内容
- `net_config` 指定了 ZMQ 服务器的连接信息

#### 3.2 等待服务器就绪

```python
# 第 36-43 行: 等待机器人服务器启动
for i in range(5):
    hex_log(HEX_LOG_LEVEL["info"], f"waiting for robot to work: {i}s")
    working = client.is_working()  # 检查服务器状态
    if working is not None and working["cmd"] == "is_working_ok":
        break  # 服务器就绪,退出等待
    else:
        time.sleep(1.0)  # 等待 1 秒后重试
```

**工作原理**:
- `is_working()` 发送心跳请求
- 服务器响应 `{"cmd": "is_working_ok"}` 表示就绪
- 最多等待 5 秒

#### 3.3 获取机械臂信息

```python
# 第 45-48 行: 查询机械臂参数
dofs = client.get_dofs()[0]      # 获取自由度数量(返回元组,取第一个元素)
limits = client.get_limits()     # 获取关节限位 [N×2] 数组
hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")
hex_log(HEX_LOG_LEVEL["info"], f"limits: {limits}")
```

**返回值**:
- `dofs`: 例如 `7` (6 轴 + 1 夹爪)
- `limits`: 例如
  ```
  [[-3.14, 3.14],   # 关节1 限位
   [-2.0,  2.0],    # 关节2 限位
   ...
   [0.0,   1.0]]    # 夹爪限位
  ```

#### 3.4 主控制循环

```python
# 第 50-74 行: 实时控制循环
rate = HexRate(500)  # 创建 500Hz 频率控制器
while True:
    # 1. 读取状态
    states_hdr, states = client.get_states()
    if states_hdr is not None:
        curr_ts = hex_zmq_ts_now()
        # 计算延迟
        delay = hex_zmq_ts_delta_ms(curr_ts, states_hdr['ts'])

        # 打印状态信息
        hex_log(HEX_LOG_LEVEL["info"],
                f"states_seq: {states_hdr['args']}; delay: {delay}ms")
        hex_log(HEX_LOG_LEVEL["info"], f"states pos: {states[:, 0]}")
        hex_log(HEX_LOG_LEVEL["info"], f"states vel: {states[:, 1]}")
        hex_log(HEX_LOG_LEVEL["info"], f"states eff: {states[:, 2]}")

    # 2. 发送命令
    cmds = np.array([
        0.5,   # 关节1 目标位置(rad)
        -1.5,  # 关节2
        3.0,   # 关节3
        0.0,   # 关节4
        0.0,   # 关节5
        0.0,   # 关节6
        0.5,   # 夹爪
    ])
    client.set_cmds(cmds)

    # 3. 精确延时到下一个周期
    rate.sleep()
```

**关键概念**:

1. **HexRate(500)**:
   - 确保循环精确运行在 500Hz
   - 自动补偿代码执行时间
   - `rate.sleep()` 会等待到下一个周期

2. **states 数据结构**:
   ```python
   # states.shape = (N, 3), N 是关节数
   states[:, 0]  # 所有关节的位置(rad)
   states[:, 1]  # 所有关节的速度(rad/s)
   states[:, 2]  # 所有关节的力矩/力(N·m 或 N)
   ```

3. **序列号跟踪**:
   - `states_hdr['args']` 包含序列号
   - 防止接收过时的数据
   - 自动在客户端/服务器端处理

4. **时间戳和延迟**:
   - `states_hdr['ts']`: 服务器发送时的时间戳
   - `hex_zmq_ts_delta_ms()`: 计算网络延迟
   - 典型延迟: 1-5ms (本地), 10-50ms (网络)

---

### 4. 服务器端 (robot_hexarm_srv.py)

虽然不在示例目录中,但了解服务器如何工作很重要。

#### 配置结构

```python
NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1000,
    "server_num_workers": 4,  # 4个工作线程处理请求
}

ROBOT_CONFIG = {
    "device_ip": "172.18.8.161",    # 硬件 IP(会被 launch.py 覆盖)
    "device_port": 8439,
    "control_hz": 250,              # 硬件通信频率
    "arm_type": "archer_l6y",
    "use_gripper": True,
    "mit_kp": [...],                # PD 控制器 Kp 参数
    "mit_kd": [...],                # PD 控制器 Kd 参数
    "sens_ts": True,                # 启用时间戳感知
}
```

#### 请求处理流程

```python
def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
    if recv_hdr["cmd"] == "is_working":
        return self.no_ts_hdr(recv_hdr, self._device.is_working()), None

    elif recv_hdr["cmd"] == "get_dofs":
        dofs = self._device.get_dofs()
        return self.no_ts_hdr(recv_hdr, dofs is not None), dofs

    elif recv_hdr["cmd"] == "get_limits":
        limits = self._device.get_limits()
        return self.no_ts_hdr(recv_hdr, limits is not None), limits

    elif recv_hdr["cmd"] == "get_states":
        return self._get_states(recv_hdr)  # 带序列号跟踪

    elif recv_hdr["cmd"] == "set_cmds":
        return self._set_cmds(recv_hdr, recv_buf)  # 处理命令数据

    else:
        raise ValueError(f"unknown command: {recv_hdr['cmd']}")
```

#### 服务器架构

```
┌─────────────────────────────────────────────┐
│  HexRobotHexarmServer (ZMQ 服务器)          │
│                                             │
│  ROUTER Socket (前端)                       │
│       │                                     │
│       ├─► Worker 1 (处理请求)               │
│       ├─► Worker 2                          │
│       ├─► Worker 3                          │
│       └─► Worker 4                          │
│                                             │
└────────────────┬────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────┐
│  HexRobotHexarm (设备层)                    │
│  - 设备线程: work_loop() 与硬件通信         │
│  - 使用 HexSafeValue 线程安全传递数据       │
└────────────────┬────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────┐
│  hex_device.Arm (底层驱动)                  │
│  WebSocket + Protobuf 与硬件通信            │
└─────────────────────────────────────────────┘
```

---

## 运行流程时序图

```
时间轴 │  launch.py    │ robot_hexarm_srv │   cli.py    │  硬件
──────┼───────────────┼──────────────────┼─────────────┼────────
  t0  │ 启动          │                  │             │
  t1  │ fork子进程1 ──┼─►启动服务器      │             │
  t2  │               │ 连接硬件 ────────┼─────────────┼─►连接
  t3  │               │ 启动ZMQ监听      │             │
  t4  │ fork子进程2 ──┼──────────────────┼─►启动客户端 │
  t5  │               │                  │ is_working? │
  t6  │               │◄─────────────────┤ (重试...)   │
  t7  │               │ working_ok ──────┼─►就绪!      │
  t8  │               │◄─────────────────┤ get_dofs    │
  t9  │               │ dofs=7 ──────────┼─►           │
  t10 │               │◄─────────────────┤ get_states  │
  t11 │               │ 读取硬件 ─────────┼─────────────┼─►
  t12 │               │◄─────────────────┼─────────────┤ 状态
  t13 │               │ states ───────────┼─►           │
  t14 │               │◄─────────────────┤ set_cmds    │
  t15 │               │ 发送到硬件 ───────┼─────────────┼─►命令
  t16 │               │ ok ───────────────┼─►           │
  ... │               │ (循环: 500Hz)     │             │
```

---

## 配置参数详解

### launch.py 中的配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ARM_TYPE` | str | "archer_l6y" | 机械臂型号 |
| `GRIPPER_TYPE` | str | "gp100" | 夹爪型号 |
| `DEVICE_IP` | str | "10.42.0.101" | 硬件 IP 地址 |
| `HEXARM_DEVICE_PORT` | int | 8439 | CAN 端口 |
| `control_hz` | int | 500 | 控制循环频率 |
| `sens_ts` | bool | True | 启用时间戳感知 |

### cli.json 中的配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `ip` | str | "127.0.0.1" | ZMQ 服务器 IP |
| `port` | int | 12345 | ZMQ 服务器端口 |
| `client_timeout_ms` | int | 200 | 客户端超时 |
| `server_timeout_ms` | int | 1000 | 服务器超时 |
| `server_num_workers` | int | 4 | 工作线程数 |

### 服务器配置 (robot_hexarm.json)

| 参数 | 类型 | 说明 |
|------|------|------|
| `device_ip` | str | 机械臂硬件 IP |
| `device_port` | int | 硬件通信端口 |
| `control_hz` | int | 硬件控制频率 |
| `arm_type` | str/int | 机械臂型号 |
| `use_gripper` | bool | 是否使用夹爪 |
| `mit_kp` | list | PD 控制 Kp 增益 |
| `mit_kd` | list | PD 控制 Kd 增益 |
| `sens_ts` | bool | 时间戳感知 |

---

## 日志系统

### 日志位置

```
logs/
├── info/
│   ├── hexarm_robot_cli_20251110_115602.log    # 客户端信息日志
│   └── robot_hexarm_srv_20251110_115602.log    # 服务器信息日志
└── err/
    ├── hexarm_robot_cli_20251110_115602.log    # 客户端错误日志
    └── robot_hexarm_srv_20251110_115602.log    # 服务器错误日志
```

### 日志级别

```python
HEX_LOG_LEVEL = {
    "info": 0,   # 信息
    "warn": 1,   # 警告
    "err": 2,    # 错误
}

# 使用方式
hex_log(HEX_LOG_LEVEL["info"], "这是一条信息")
hex_log(HEX_LOG_LEVEL["warn"], "这是一条警告")
hex_log(HEX_LOG_LEVEL["err"], "这是一条错误")
```

### 设置日志级别

在 `HexLaunch` 中可以设置最小日志级别:

```python
launch = HexLaunch(NODE_CFGS, min_level=1)  # 只显示 warn 和 err
```

---

## 常见修改场景

### 1. 修改控制频率

```python
# 在 launch.py 中修改
"cfg": {
    "params": {
        "control_hz": 1000,  # 改为 1000Hz
    }
}

# 在 cli.py 中对应修改
rate = HexRate(1000)  # 保持一致
```

### 2. 修改目标位置

```python
# 在 cli.py 第 63-71 行修改
cmds = np.array([
    1.0,    # 修改关节1目标
    0.0,    # 修改关节2目标
    0.0,    # ...
    0.0,
    0.0,
    0.0,
    1.0,    # 修改夹爪开合
])
```

### 3. 添加轨迹跟踪

```python
# 在 cli.py 主循环中添加
t = 0.0
dt = 1.0 / 500.0  # 时间步长

while True:
    # 正弦波轨迹
    cmds = np.array([
        np.sin(t),
        np.cos(t),
        0.0,
        0.0,
        0.0,
        0.0,
        0.5,
    ])
    client.set_cmds(cmds)
    t += dt
    rate.sleep()
```

### 4. 添加力矩反馈控制

```python
# 在 cli.py 主循环中
states_hdr, states = client.get_states()
if states_hdr is not None:
    torques = states[:, 2]  # 获取力矩

    # 如果力矩超过阈值,停止运动
    if np.any(np.abs(torques) > 10.0):
        hex_log(HEX_LOG_LEVEL["warn"], "Torque limit exceeded!")
        cmds = states[:, 0]  # 保持当前位置
    else:
        cmds = target_positions  # 继续运动

    client.set_cmds(cmds)
```

---

## 故障排查

### 问题1: 客户端无法连接服务器

**现象**: `waiting for robot to work` 重复 5 次后仍失败

**可能原因**:
1. 服务器未启动或启动失败
2. IP/端口配置错误
3. 防火墙阻止连接

**解决方法**:
```bash
# 检查日志
cat logs/err/robot_hexarm_srv_*.log

# 检查端口占用
netstat -tlnp | grep 12345

# 测试 ZMQ 连接
python -c "import zmq; ctx = zmq.Context(); sock = ctx.socket(zmq.REQ); sock.connect('tcp://127.0.0.1:12345'); print('OK')"
```

### 问题2: 无法连接硬件

**现象**: 服务器日志显示连接超时

**可能原因**:
1. 硬件 IP 配置错误
2. 硬件未上电或网络未连接
3. 防火墙阻止 WebSocket 连接

**解决方法**:
```bash
# ping 硬件
ping 10.42.0.101

# 检查端口
telnet 10.42.0.101 8439

# 修改 launch.py 中的 DEVICE_IP
```

### 问题3: 控制延迟高

**现象**: `delay` 超过 50ms

**可能原因**:
1. 网络拥塞
2. 控制频率过高
3. CPU 负载高

**解决方法**:
```python
# 降低控制频率
"control_hz": 250,  # 从 500 降到 250

# 在 cli.py 中
rate = HexRate(250)
```

### 问题4: 关节超限

**现象**: 机械臂不动或抖动

**可能原因**:
1. 目标位置超出关节限位
2. 初始位置不在限位内

**解决方法**:
```python
# 检查限位
limits = client.get_limits()
print(limits)

# 确保命令在限位内
cmds = np.clip(cmds, limits[:, 0], limits[:, 1])
```

---

## 总结

### 核心概念

1. **多进程架构**: 服务器和客户端运行在独立进程中
2. **ZMQ 通信**: 使用 REQ-REP 模式,支持多客户端
3. **序列号跟踪**: 防止过时数据
4. **线程池**: 服务器使用 4 个工作线程并发处理请求
5. **精确定时**: HexRate 确保精确的控制频率

### 关键 API

- `client.is_working()`: 检查服务器状态
- `client.get_dofs()`: 获取自由度
- `client.get_limits()`: 获取关节限位
- `client.get_states()`: 获取当前状态(位置/速度/力矩)
- `client.set_cmds(cmds)`: 发送控制命令
- `HexRate(hz)`: 精确频率控制

### 下一步

1. **阅读 HEX_ARM_API_GUIDE.md**: 了解完整 API
2. **修改 cli.py**: 实现自己的控制逻辑
3. **集成视觉系统**: 添加相机和 3D 重建
4. **开发抓取算法**: 基于状态反馈实现闭环控制
