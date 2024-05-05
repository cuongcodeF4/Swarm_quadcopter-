
"""
Example Usage:
--------------
To generate a CSV file for a circular trajectory, use the following code snippet:

create_active_csv(shape_name="circle", diameter=5.0, direction=1, maneuver_time=60.0, start_x=0.0, start_y=0.0, initial_altitude=10.0, climb_rate=2.0, move_speed=2.5, hold_time=2.0)

Visualization:
--------------
After generating the CSV file, you can visualize the trajectory using plot functions and save the trajectory plot in the "shaped" folder along with the CSV file.

Note:
-----
Make sure to have the necessary dependencies installed and correctly set up the offboard control system to use the generated CSV file for controlling the drone in an offboard mode.

Usage:
------
The output generated by `csvCreator.py` can be utilized in the `offboard_from_csv.py` file, also available in the same GitHub repository, to control a drone in an offboard mode. The generated CSV file contains the necessary information for each step of the drone's trajectory, including position, velocity, acceleration, yaw angle, and LED colors.

Offboard Control in PX4:
------------------------
The offboard mode in PX4 is a flight mode that allows external systems to control the drone's position and velocity directly. It enables autonomous flight and is commonly used in research, development, and testing scenarios. The offboard control system communicates with the drone's flight controller through a communication protocol like MAVLink.

To use the CSV output generated by `csvCreator.py` for offboard control in PX4, you can follow these steps:
1. Load the generated CSV file, which represents the desired trajectory for the drone.
2. Extract the position and velocity information from the CSV file.
3. Send the position and velocity commands to the drone's flight controller using an offboard control system, such as the `offboard_from_csv.py` script.
4. The flight controller will interpret the commands and execute the desired trajectory, guiding the drone accordingly.

CSV File Structure and Guide:
----------------------------
The CSV file created by `csvCreator.py` follows a specific structure, where each row represents a step of the drone's trajectory. The columns in the CSV file contain the following information:
- `idx`: Index or step number of the trajectory.
- `t`: Time in seconds for the given step.
- `px`: Drone's position in the X-axis.
- `py`: Drone's position in the Y-axis.
- `pz`: Drone's position in the Z-axis (negative value indicates altitude).
- `vx`: Drone's velocity in the X-axis.
- `vy`: Drone's velocity in the Y-axis.
- `vz`: Drone's velocity in the Z-axis.
- `ax`: Drone's acceleration in the X-axis.
- `ay`: Drone's acceleration in the Y-axis.
- `az`: Drone's acceleration in the Z-axis.
- `yaw`: Drone's yaw angle.
- 'mode' : Flight Phase Mode
- `ledr`: Red component value for the drone's LED color.
- `ledg`: Green component value for the drone's LED color.
- `ledb`: Blue component value for the drone's LED color.

Flight Modes and Codes:
- 0: On the ground
- 10: Initial climbing state
- 20: Initial holding after climb
- 30: Moving to start point
- 40: Holding at start point
- 50: Moving to maneuvering start point
- 60: Holding at maneuver start point
- 70: Maneuvering (trajectory)
- 80: Holding at the end of the trajectory coordinate
- 90: Returning to home coordinate
- 100: Landing

Each flight mode is represented by an integer code. These codes are used to indicate the different phases of the flight in the CSV file.

To create a valid CSV file for offboard control, make sure to adhere to the structure described above. Each row should represent a specific time step with the corresponding position, velocity, acceleration, and LED color values.

"""

import csv
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
from functions.export_and_plot_shape import export_and_plot_shape
from functions.trajectories import *
from functions.create_active_csv import create_active_csv

# Example usage

num_repeats = 1
shape_name="circle"
diameter = 10
direction = 1
maneuver_time = 60.0
start_x = 0
start_y = 0
initial_altitude = 10
move_speed = 2.0  # m/s
hold_time = 4.0 #s
step_time = 0.05 #s
output_file = "shapes/active.csv"

create_active_csv(
    shape_name=shape_name,
    num_repeats=num_repeats,
    diameter=diameter,
    direction=direction,
    maneuver_time=maneuver_time,
    start_x=start_x,
    start_y=start_y,
    initial_altitude=initial_altitude,
    move_speed = move_speed,
    hold_time = hold_time,
    step_time = step_time,
    output_file = output_file,
)

export_and_plot_shape(output_file)