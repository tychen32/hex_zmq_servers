# Mujoco Archer D6Y Example

## Description

This example shows how to use Mujoco simulation for Archer D6Y robot. It simulates the Archer D6Y robot in Mujoco physics engine and provides position, velocity, torque control, as well as RGB and depth image feedback.

## Structure

```bash
mujoco_archer_d6y/
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

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/basic/mujoco_archer_d6y`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```

- The Mujoco visualization window will open automatically.
