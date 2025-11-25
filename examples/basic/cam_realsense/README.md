# Camera Realsense Example

## Description

This example shows how to use Realsense RGB-D camera device. It captures RGB and depth images from Realsense camera.

## Structure

```bash
cam_realsense/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- RealSense RGB-D camera

### Environment

1. Check device connection:

    ```bash
    lsusb | grep RealSense
    ```

2. (**Important**) Modify the `SERIAL_NUMBER` and `SENS_TS` in `launch.py` to match your device before running the example.

## Usage

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/basic/cam_realsense`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```
