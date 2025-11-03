# Robot GELLO Example

## Description

This example shows how to use GELLO robot device. It reads the joint positions from GELLO robot and provides position, velocity, and torque feedback.

## Structure

```bash
robot_gello/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- GELLO robot device

### Environment

1. Check USB connection:

    ```bash
    ls /dev/ttyUSB*
    ```

2. Set device permission:

    ```bash
    sudo chmod 666 /dev/ttyUSB0
    ```

3. (Optional) Add user to dialout group (permanent solution):

    ```bash
    sudo usermod -aG dialout $USER
    # Logout and login again
    ```

4. (**Important**) Modify the `GELLO_DEVICE` in `launch.py` to match your device port (e.g., `/dev/ttyUSB0`) before running the example.

## Usage

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/basic/robot_gello`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```

- ⚠️ **Safety Notice**: This controls a real robot. Make sure there is enough safe space around the robot and you can cut off power at any time.
