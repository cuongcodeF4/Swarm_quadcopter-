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

class Drone():
    def __init__(self):
        self.arm_command = 1  # Start arm 
        self.type_mask = int(0b110111111000)  #just position
        self.mode = 4 #mode GUIDED

    def basicFly(self,drone,direction):
        # Set mode: Guided for Drone
        drone.mav.command_long_send(drone.target_system, drone.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, self.mode, 0, 0, 0, 0, 0)

        time.sleep(2)

        # Prepare for arm: Disarm all  
        drone.mav.command_long_send(drone.target_system, drone.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        #Initial arm 
        drone.mav.command_long_send(drone.target_system, drone.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, self.arm_command, 0, 0, 0, 0, 0, 0)
        time.sleep(2)

        #Start takeoff in altitude is 10m
        drone.mav.command_long_send(drone.target_system, drone.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
        print("[INFO] Drone:",str(drone), "takeoff success")
        time.sleep(7)

        # Movement for drone 
        drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,drone.target_system, 
                                    drone.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED ,self.type_mask,direction,direction,0,0,0,0,0,0,0,0,0))
        print("[INFO] Direction =",direction)
        # msg = drone.recv_match(type= "LOCAL_POSITION_NED",blocking = False)
        # print("[INFO] Message feedback from local position:",msg)

if __name__ == "__main__":
    # # Create multiple threads
    # drone = Drone()
    # threads = []
    # thread1 = threading.Thread(target=drone.basicFly, args=(drone1,10,))
    # threads.append(thread1)
    # thread2 = threading.Thread(target=drone.basicFly, args=(drone2,-10,))
    # threads.append(thread2)
    # for thread in threads:
    #     thread.start()
    drone = Drone()
    drone.basicFly(drone1,10)
    drone.basicFly(drone2,-10)

    

