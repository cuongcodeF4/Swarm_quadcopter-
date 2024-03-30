from pymavlink import mavutil

print("Waiting for connection...")
drone = mavutil.mavlink_connection("127.0.0.1:14550")
drone.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (drone.target_system, drone.target_component))

drone.mav.set_position_target_local_ned_send(0,
                                            drone.target_system,
                                            drone.target_component,
                                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                            0b110111000000,
                                            50, 0, -10,
                                            0, 0, 0,
                                            0, 0, 0,
                                            0, 0)