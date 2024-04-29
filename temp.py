def main2():
    #-----------------------------------Connect to the drone-----------------------------------#
    print("[INFO] Waiting for connection...")
    drone = mavutil.mavlink_connection("127.0.0.1:14550")
    drone.wait_heartbeat()
    print("[INFO] Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))

    #-----------------------------------Read the waypoints from the CSV file-----------------------------------#
    print("[INFO] Reading waypoints from the CSV file...")
    waypoints = []
    with open("shapes/active.csv", newline="") as csvfile:
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
            waypoints.append((t, px, py, pz, vx, vy, vz))
    initial_alt = -waypoints[0][3] # Initial altitude is the z-coordinate of the first waypoint

    #-----------------------------------Wait for the drone to be ready-----------------------------------#
    print("[INFO] Waiting for the drone to be ready...")
    while True:
        msg = drone.recv_match(type="LOCAL_POSITION_NED", blocking=True)
        if msg:
            print("[INFO] Drone is ready!")
            break

    #-----------------------------------Change to GUIDED mode-----------------------------------#
    print("[INFO] Changing to GUIDED mode...")
    drone.mav.command_long_send(drone.target_system,
                                drone.target_component,
                                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, 
                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                guided_mode, 0, 0, 0, 0, 0)

    #-----------------------------------Arm the drone-----------------------------------#
    print("[INFO] Arming the drone...")
    drone.mav.command_long_send(drone.target_system,
                                drone.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 
                                arm_command, 0, 0, 0, 0, 0, 0)

    #-----------------------------------Takeoff to the initial altitude-----------------------------------#
    print("[INFO] Taking off...")
    drone.mav.command_long_send(drone.target_system,
                                drone.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 
                                0, 0, 0, 0, 0, 0, initial_alt)
    # Wait until the drone reaches the initial altitude
    while True:
        msg = drone.recv_match(type="LOCAL_POSITION_NED", blocking=True)
        current_alt = -msg.z # Altitude is negative in NED coordinates
        # print("[INFO] Current altitude: ", current_alt)
        if current_alt > (initial_alt * 0.95):
            break

    #-----------------------------------Perform the mission-----------------------------------#
    print("[INFO] Performing the mission...")
    total_duration = waypoints[-1][0]  # Total duration is the time of the last waypoint
    t = 0  # Time variable

    while t <= total_duration:
        # Find the current waypoint based on time
        current_waypoint = None
        for waypoint in waypoints:
            if t <= waypoint[0]:
                current_waypoint = waypoint
                break

        if current_waypoint is None:
            # Reached the end of the trajectory
            break

        position = current_waypoint[1:4]  # Extract position (px, py, pz)
        velocity = current_waypoint[4:7]  # Extract velocity (vx, vy, vz)

        # Send the SET_POSITION_TARGET_LOCAL_NED command to the drone
        drone.mav.set_position_target_local_ned_send(0,
                                            drone.target_system,
                                            drone.target_component,
                                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                            0b100111000000,
                                            position[0], position[1], position[2],
                                            velocity[0], velocity[1], velocity[2],
                                            0, 0, 0,
                                            yaw, 0)
        time.sleep(0.1)
        t += 0.1

    print("[INFO] Mission completed!")

    #-----------------------------------Return to home-----------------------------------#    
    print("[INFO] Returning to home...")
    drone.mav.command_long_send(drone.target_system,
                                drone.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                0, 0, 0, 0, 0, 0, 0, 0)
    
    while True:
        msg = drone.recv_match(type="LOCAL_POSITION_NED", blocking=True)
        current_alt = -msg.z
        if current_alt < 0.1:
            break
    print("[INFO] Drone has landed!")