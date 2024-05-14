import time
import math


def circle_trajectory(step,yawToMaster, maneuver_time, diameter, direction, initial_alt, step_time):
    thetaCurrent = math.radians(180 + yawToMaster)
    t = step * step_time
    theta = 2 * direction * math.pi * t / maneuver_time
    
    x = (diameter / 2) * math.cos(theta + thetaCurrent )
    y = (diameter / 2) * math.sin(theta + thetaCurrent )
    z = -1 * initial_alt

    vx = -(diameter / 2) * math.sin(theta + thetaCurrent ) * 2 * direction * math.pi / maneuver_time
    vy = (diameter / 2) * math.cos(theta + thetaCurrent ) * 2 * direction * math.pi / maneuver_time
    vz = 0

    ax = -(diameter / 2) * math.cos(theta + thetaCurrent ) * 4 * direction * math.pi ** 2 / maneuver_time ** 2
    ay = -(diameter / 2) * math.sin(theta + thetaCurrent ) * 4 * direction * math.pi ** 2 / maneuver_time ** 2
    az = 0

    return x, y, z, vx, vy, vz, ax, ay, az

def square_trajectory(step, maneuver_time, diameter, direction, initial_alt, step_time):
    t = step * step_time
    side_length = diameter / math.sqrt(2)
    side_time = maneuver_time / 4
    side_steps = int(maneuver_time / (4 * step_time))

    current_side = step // side_steps
    side_progress = (step % side_steps) / side_steps

    if current_side == 0:
        x = side_length * side_progress
        y = 0
    elif current_side == 1:
        x = side_length
        y = side_length * side_progress
    elif current_side == 2:
        x = side_length * (1 - side_progress)
        y = side_length
    else:
        x = 0
        y = side_length * (1 - side_progress)

    if direction == -1:
        x, y = y, x

    z = -1 * initial_alt

    vx = side_length / side_time if (current_side == 0 or current_side == 2) else 0
    vy = side_length / side_time if (current_side == 1 or current_side == 3) else 0
    vz = 0

    if direction == -1:
        vx, vy = vy, vx

    ax = -side_length / side_time ** 2 * math.sin(2 * direction * math.pi * t / maneuver_time) if (current_side == 0 or current_side == 2) else 0
    ay = -side_length / side_time ** 2 * math.cos(2 * direction * math.pi * t / maneuver_time) if (current_side == 1 or current_side == 3) else 0
    az = 0

    return x, y, z, vx, vy, vz, ax, ay, az
