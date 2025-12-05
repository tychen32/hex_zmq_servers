# GELLO Real Teleoperation Example

## Description

This example shows how to use GELLO robot to control a real E3 Desktop robot in real-time. It demonstrates bilateral teleoperation where GELLO serves as the master device and E3 Desktop follows its movements.

## Structure

```bash
gello_real/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- GELLO robot device * 2
- E3 Desktop robot

### Environment

**For GELLO robot:**

1. Check USB connection:

    ```bash
    ls /dev/ttyUSB*
    ```

2. Set device permission:

    ```bash
    sudo chmod 666 /dev/ttyUSB0
    sudo chmod 666 /dev/ttyUSB1
    ```

3. (Optional) Add user to dialout group (permanent solution):

    ```bash
    sudo usermod -aG dialout $USER
    # Logout and login again
    ```

4. (**Important**) Modify the `LEFT_GELLO_DEVICE` and `RIGHT_GELLO_DEVICE` in `launch.py` to match your device port (e.g., `/dev/ttyUSB0` and `/dev/ttyUSB1`) before running the example.

**For E3 Desktop robot:**

1. Find the robot IP address.

2. (**Important**) Modify the `ARM_TYPE` and `GRIPPER_TYPE` in `launch.py` to match your device model before running the example.

3. (**Important**) Modify the `DEVICE_LEFT_IP`, `DEVICE_RIGHT_IP`, `HEXARM_LEFT_DEVICE_PORT` and `HEXARM_RIGHT_DEVICE_PORT` in `launch.py` to match your device port (e.g., `DEVICE_LEFT_IP = "192.168.1.101"`, `DEVICE_RIGHT_IP = "192.168.1.102"`, `HEXARM_LEFT_DEVICE_PORT = 8439` and `HEXARM_RIGHT_DEVICE_PORT = 9439`) before running the example.
   1. `CAN0` => `8439`
   2. `CAN1` => `9439`

## Usage

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/adv/double_gello_real`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```
