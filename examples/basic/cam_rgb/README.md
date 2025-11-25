# Camera RGB Example

## Description

This example shows how to use RGB camera device. It captures RGB images from /dev/video* camera.

## Structure

```bash
cam_rgb/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- RGB camera

### Environment

1. Check device connection:

    ```bash
    ls /dev/video*
    ```

2. (**Important**) Modify the `CAM_PATH` in `launch.py` to match your device before running the example.

## Usage

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/basic/cam_rgb`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```
