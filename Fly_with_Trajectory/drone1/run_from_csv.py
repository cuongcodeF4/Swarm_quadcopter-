from pymavlink import mavutil
import csv
from pymavlinkFunction1 import *

arm_command = 1
guided_mode = 4

def readWaypoints(path_to_csv):
    #-----------------------------------Read the waypoints from the CSV file-----------------------------------#
    print("[INFO] Reading waypoints from the CSV file...")
    waypoints_list = []
    with open(path_to_csv, newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            t = float(row["t"])
            px = float(row["px"])
            py = float(row["py"])
            pz = float(row["pz"])
            vx = float(row["vx"])
            vy = float(row["vy"])
            vz = float(row["vz"])
            yaw = float(row["yaw"])
            waypoints_list.append((t, px, py, pz, vx, vy, vz))
    initial_alt = -waypoints_list[0][3] # Initial altitude is the z-coordinate of the first waypoint
    return waypoints_list, initial_alt, yaw

def main():
    # Create an instance of the Drone class
    drone = MAV()
    # Read the waypoints from the CSV file
    waypoints_list, initial_alt, yaw = readWaypoints("shapes/active.csv")
    # Wait for the drone to be ready
    print("[INFO] Waiting for the drone to be ready...")
    drone.wait_drone_ready()
    # Change to GUIDED mode
    print("[INFO] Changing to GUIDED mode...")
    drone.setMode(guided_mode)
    # Arm the drone
    print("[INFO] Arming the drone...")
    drone.arm(arm_command)
    # Takeoff to the initial altitude
    print("[INFO] Taking off...")
    drone.takeoff(initial_alt)
    # Perform the mission
    print("[INFO] Performing the mission...")
    drone.performMission(waypoints_list, yaw)
    # Return to home
    # print("[INFO] Returning to home...")
    # drone.RTL()

if __name__ == "__main__":
    main()