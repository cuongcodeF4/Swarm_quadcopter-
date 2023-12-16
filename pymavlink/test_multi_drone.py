from pymavlink import mavutil
import time
import threading

# Start a connection listening on a UDP port
drone1 = mavutil.mavlink_connection('udp:172.30.144.1:14560')
drone2 = mavutil.mavlink_connection('udp:172.30.144.1:14550')
# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
drone1.wait_heartbeat()
drone2.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (drone1.target_system, drone1.target_component))
print("Heartbeat from system (system %u component %u)" % (drone2.target_system, drone2.target_component))
    
def basicFly(drone,direction):
    arm_command = 1  # Start arm 
    type_mask = int(0b110111111000)  #just position
    mode = 4 #mode GUIDED
    # Set mode: Guided for Drone
    drone.mav.command_long_send(drone.target_system, drone.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode, 0, 0, 0, 0, 0)

    time.sleep(2)

    # Prepare for arm: Disarm all  
    drone.mav.command_long_send(drone.target_system, drone.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    #Initial arm 
    drone.mav.command_long_send(drone.target_system, drone.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_command, 0, 0, 0, 0, 0, 0)
    time.sleep(2)
    #Start takeoff in altitude is 10m
    drone.mav.command_long_send(drone.target_system, drone.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    print("[INFO] Drone:",str(drone), "takeoff success")
    time.sleep(7)

       
if __name__ =="__main__":
    basicFly(drone1,10)
    basicFly(drone2,-10)

    

