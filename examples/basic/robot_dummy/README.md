# Robot Dummy Example

## Description

This example shows how to use the robot device. It simulates a 7-DOF robot and provides position, velocity, and torque feedback and control.

## Structure

```bash
robot_dummy/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

None.

### Environment

None.

## Usage

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/basic/robot_dummy`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```
