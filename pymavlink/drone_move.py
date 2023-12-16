from pymavlink import mavutil
import time


arm_command = 1  # Start arm 
type_mask = int(0b110111111000)  #just position
# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udp:172.30.144.1:14550')
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

while True:
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

    time.sleep(1)

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    

    time.sleep(7)
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,the_connection.target_system, 
                            the_connection.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED ,type_mask,-10,-10,0,0,0,0,0,0,0,0,0))
    msg = the_connection.recv_match(type= "LOCAL_POSITION",blocking = True)
    print("[INFO] Message feedback from local position:",msg)
