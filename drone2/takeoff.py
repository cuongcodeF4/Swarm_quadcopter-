from pymavlink import mavutil
import csv
from pymavlinkFunction import *

arm_command = 1
guided_mode = 4

def main():
    # Create an instance of the Drone class
    drone = MAV()
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
    drone.takeoff(15)

if __name__ == "__main__":
    main()
