# Trajectory Tracking Simulation Example

## Description

This example shows how to track a trajectory in Mujoco Archer D6Y simulation.

## Structure

```bash
traj_sim/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
├── plot.py    # data analysis and plotting tool
└── README.md  # this file
```

## Dependencies

### Hardware

None

### Environment

None

## Usage

### Running the Simulation

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/adv/traj_sim`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```

- The simulation will generate a CSV file with trajectory data: `traj_sim_data_YYYYMMDD_HHMMSS.csv`

### Analyzing the Data

After running the simulation, you can analyze the generated CSV data using the `plot.py` script:

1. **Display all plots interactively:**

    ```bash
    python3 plot.py traj_sim_data_20251105_201914.csv
    ```

2. **Save plots to files (recommended for headless systems):**

    ```bash
    python3 plot.py traj_sim_data_20251105_201914.csv --save
    ```

    This will create a `plots/` directory with the following images:
    - `joint_positions.png` - Joint positions (actual vs target)
    - `joint_velocities.png` - Joint velocities over time
    - `joint_efforts.png` - Joint efforts/torques over time
    - `tracking_errors.png` - Tracking errors with statistics
    - `summary_statistics.png` - Summary bar charts for all joints
    - `3d_trajectory.png` - 3D visualization of joint space trajectory

3. **Plot specific joints only:**

    ```bash
    python3 plot.py traj_sim_data_20251105_201914.csv --joints "0,1,2" --save
    ```

4. **Specify output directory:**

    ```bash
    python3 plot.py traj_sim_data_20251105_201914.csv --save --output-dir my_analysis
    ```

5. **View statistics without plotting:**

    ```bash
    python3 plot.py traj_sim_data_20251105_201914.csv --save > /dev/null
    # Statistics will still be printed to console
    ```

### Plot Descriptions

- **Joint Positions**: Shows actual joint positions vs target positions for each joint
- **Joint Velocities**: Displays joint velocities over time
- **Joint Efforts**: Shows the torques/efforts applied to each joint
- **Tracking Errors**: Visualizes the difference between target and actual positions with mean/max error statistics
- **Summary Statistics**: Bar charts showing position ranges, max velocities, max efforts, and tracking errors
- **3D Trajectory**: A 3D visualization of the first 3 joints' movement in joint space
