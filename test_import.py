#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
测试脚本:验证 hex_zmq_servers 环境配置
"""

print("=" * 60)
print("HEX_ZMQ_SERVERS 环境验证")
print("=" * 60)

# 1. 测试基础导入
print("\n1. 测试基础模块导入...")
try:
    import hex_zmq_servers
    print("   ✓ hex_zmq_servers 导入成功")
    print(f"   版本: {hex_zmq_servers.__version__ if hasattr(hex_zmq_servers, '__version__') else '未知'}")
except ImportError as e:
    print(f"   ✗ hex_zmq_servers 导入失败: {e}")
    exit(1)

# 2. 测试 HexRobotHexarmClient 导入
print("\n2. 测试 HexRobotHexarmClient 导入...")
try:
    from hex_zmq_servers import HexRobotHexarmClient
    print("   ✓ HexRobotHexarmClient 导入成功")
except ImportError as e:
    print(f"   ✗ HexRobotHexarmClient 导入失败: {e}")
    exit(1)

# 3. 测试其他常用类
print("\n3. 测试其他常用类...")
try:
    from hex_zmq_servers import (
        HexRate,
        hex_zmq_ts_now,
        hex_zmq_ts_delta_ms,
        HEX_LOG_LEVEL,
        hex_log,
    )
    print("   ✓ HexRate 导入成功")
    print("   ✓ 时间戳工具导入成功")
    print("   ✓ 日志工具导入成功")
except ImportError as e:
    print(f"   ✗ 导入失败: {e}")
    exit(1)

# 4. 测试底层 hex_device
print("\n4. 测试底层 hex_device...")
try:
    import hex_device
    print(f"   ✓ hex_device 导入成功")
    print(f"   版本: {hex_device.__version__}")
except ImportError as e:
    print(f"   ✗ hex_device 导入失败: {e}")

# 5. 检查可用方法
print("\n5. HexRobotHexarmClient 可用方法:")
methods = [m for m in dir(HexRobotHexarmClient) if not m.startswith('_')]
for method in methods:
    print(f"   - {method}")

# 6. 测试创建客户端实例(不连接)
print("\n6. 测试创建客户端实例...")
try:
    client = HexRobotHexarmClient(net_config={
        "ip": "127.0.0.1",
        "port": 12345,
        "client_timeout_ms": 200,
    })
    print("   ✓ 客户端实例创建成功")
    print(f"   注意: 这只是创建了实例,并未连接到服务器")
except Exception as e:
    print(f"   ✗ 客户端创建失败: {e}")

# 7. 测试 numpy 导入(hex_zmq_servers 依赖)
print("\n7. 测试依赖库...")
try:
    import numpy as np
    print(f"   ✓ numpy 导入成功 (版本: {np.__version__})")
except ImportError:
    print("   ✗ numpy 导入失败")

try:
    import cv2
    print(f"   ✓ opencv-python 导入成功 (版本: {cv2.__version__})")
except ImportError:
    print("   ✗ opencv-python 导入失败")

try:
    import zmq
    print(f"   ✓ pyzmq 导入成功 (版本: {zmq.__version__})")
except ImportError:
    print("   ✗ pyzmq 导入失败")

# 8. 显示已安装的 hex 相关包
print("\n8. 已安装的 HEX 相关包:")
import subprocess
result = subprocess.run(
    ["uv", "pip", "list"],
    capture_output=True,
    text=True
)
for line in result.stdout.split('\n'):
    if 'hex' in line.lower():
        print(f"   {line}")

print("\n" + "=" * 60)
print("✓ 环境验证完成!")
print("=" * 60)
print("\n你现在可以:")
print("1. 在任何 Python 脚本中导入: from hex_zmq_servers import HexRobotHexarmClient")
print("2. 运行示例: python examples/basic/robot_hexarm/launch.py")
print("3. 开发你的 3D 重建和抓取应用")
print("\n提示: 使用前请确保:")
print("- 机械臂服务器已启动 (通过 launch.py)")
print("- 网络配置正确 (IP 和端口)")
print("=" * 60)
