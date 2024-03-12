from pymavlink import mavutil

# Connect to the drone 1 and wait for the heartbeat
drone1 = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
drone1.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (drone1.target_system, drone1.target_component))

# Connect to the drone 2 and wait for the heartbeat
drone2 = mavutil.mavlink_connection('udpin:127.0.0.1:14560')
drone2.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (drone2.target_system, drone2.target_component))

