from pymavlink import mavutil

# Connect to the vehicle
drone1 = mavutil.mavlink_connection('127.0.0.1:14550')
drone1.target_system = 1  # Thiết lập System ID cho drone1
drone1.target_component = 1  # Thiết lập Component ID cho drone1
# Wait for the heartbeat message to find the system ID
drone1.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (drone1.target_system, drone1.target_component))

# # Change mode to guided
# connection.mav.command_long_send(
#     connection.target_system, connection.target_component,
#     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     4, 0, 0, 0, 0, 0
# )

# # Arm the vehicle
# connection.mav.command_long_send( 0, 0, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0 )