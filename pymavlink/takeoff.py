from pymavlink import mavutil
import time
arm_command = 1
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
    
    msg1 = the_connection.recv_match(type= "COMMAND_ACK",blocking = True)

    time.sleep(4)

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    
    msg2 = the_connection.recv_match(type= "COMMAND_ACK",blocking = True)
    print("[INFO] Acknowledge message:", msg1,"\n",msg2)
    if msg1.result ==0 or msg2.result == 0 :
        break
