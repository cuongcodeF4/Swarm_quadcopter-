# import argparse
# from pymavlink import mavutil

# def arm(mav_connection, arm_command):
#     # Wait for the first heartbeat
#     # This sets the system and component ID of remote system for the link
#     mav_connection.wait_heartbeat()
#     print("Heartbeat from system (system %u component %u)" %
#           (mav_connection.target_system, mav_connection.target_component))

#     mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
#                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)

#     msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
#     print("....",msg)

#     # return the result of the ACK message
#     return msg.result

# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description='Send arm/disarm commands using MAVLink protocol.')
#     parser.add_argument('-c', '--connect', help="Connection string", default='udp:172.30.144.1:14550')
#     parser.add_argument('-a', '--arm', type=int, choices=[0, 1], help="Arm/Disarm command", default=1)
    
#     args = parser.parse_args()

#     mav_connection = mavutil.mavlink_connection(args.connect)
#     result = arm(mav_connection, args.arm)
#     print(f'Result of arm/disarm command: {result}')


from pymavlink import mavutil

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
    msg = the_connection.recv_match(type= "COMMAND_ACK",blocking = True)
    print(msg)
