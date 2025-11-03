# ZMQ Dummy Example

## Description

This example shows the basic ZeroMQ communication mechanism of the framework. It demonstrates the request-response pattern between client and server without any specific device.

## Structure

```bash
zmq_dummy/
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

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/basic/zmq_dummy`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```

- This example is useful for understanding the underlying communication protocol and testing network performance.
