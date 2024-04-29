#khai bao thu vien can thiet
from pymavlink import mavutil
import time

# type of command will be use, make fucntion of those command
#some simple command like: take off, arm
#what will dev put in there code line to run the mavlink control
#exg: MAV.takeoff(10)

#data type input {'sysCom': '"command","value"', 'droneCom': '"command","value"'}
#but the exacts data we receive is only the afer mesg 

#so that we need to build a class that have all the control algorithm
#when init the class, an mavlink object gonna be create and can be use to 



#main class
class MAV():
    #contain only the fucntion to run all the pymavlink msg needed
    #no computational involve and only take in basic data 
    def __init__(self):
        #set up the connection when the class being create 
        self.drone  = mavutil.mavlink_connection('127.0.0.1:14550')
        #setup as the drone is waiting on connect, wait for the confirm heartbeat before doing anything
        self.drone.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.drone.target_system, self.drone.target_component))

    def wait_drone_ready(self):
        #-----------------------------------Wait for the drone to be ready-----------------------------------#
        while True:
            msg = self.drone.recv_match(type="LOCAL_POSITION_NED", blocking=True)
            if msg:
                print("[INFO] Drone is ready!")
                break
    
    def performMission(self, waypoints_list, yaw):
        total_duration = waypoints_list[-1][0]  # Total duration is the time of the last waypoint
        t = 0  # Time variable

        while t <= total_duration:
            # Find the current waypoint based on time
            current_waypoint = None
            for waypoint in waypoints_list:
                if t <= waypoint[0]:
                    current_waypoint = waypoint
                    break

            if current_waypoint is None:
                # Reached the end of the trajectory
                break

            position = current_waypoint[1:4]  # Extract position (px, py, pz)
            velocity = current_waypoint[4:7]  # Extract velocity (vx, vy, vz)

            # Send the SET_POSITION_TARGET_LOCAL_NED command to the drone
            self.drone.mav.set_position_target_local_ned_send(0,
                                                self.drone.target_system,
                                                self.drone.target_component,
                                                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                0b100111000000,
                                                position[0], position[1], position[2],
                                                velocity[0], velocity[1], velocity[2],
                                                0, 0, 0,
                                                yaw, 0)
            time.sleep(0.1)
            t += 0.1

    def checkACK(self):
        while True:
            msg = self.drone.recv_match(type= "COMMAND_ACK",blocking = True)
            print(msg)

    #take off func
    def takeoff(self, altitude):
        # all the altitude use is in meter
        self.drone.mav.command_long_send(
            self.drone.target_system, 
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0,
            0, 
            altitude
        )
        # Wait until the drone reaches the initial altitude
        while True:
            msg = self.drone.recv_match(type="LOCAL_POSITION_NED", blocking=True)
            current_alt = -msg.z # Altitude is negative in NED coordinates
            # print("[INFO] Current altitude: ", current_alt)
            if current_alt > (altitude * 0.95):
                break
    
    #Arm func
    def arm(self, state):
        # arm is 1 and disarm is 0 on the state bit
        self.drone.mav.command_long_send(
            self.drone.target_system, 
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            0, 
            state, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )
    
    #Setmode func
    def setMode(self, mode):
        #need a code lines to scan for all the 
        self.drone.mav.command_long_send(
            self.drone.target_system, 
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
            0, 
            1, 
            mode, 
            0, 
            0, 
            0, 
            0, 
            0
        )
    #land mode
    def land(self, decentSpeed, maxDecentAngle):
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            decentSpeed,
            maxDecentAngle,
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

    #RTL mode
    def RTL(self):
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )
        while True:
            msg = self.drone.recv_match(type="LOCAL_POSITION_NED", blocking=True)
            current_alt = -msg.z
            if current_alt < 0.1:
                break
        print("[INFO] Drone has landed!")

    #get value
    #user input in a ;ist of data and para user wanna take out
    #The function will scan through all the para and get all the info need for the listed para
    #then it will sum it up in a dict and output that out for other func to use
    def getValue(self,param):
        #scan for all the Attribute in the param_group, how to check?
        #how can we sure that all the data that we receive is right with the attribute?
        #create and empty dict that store all the data
        dict_data = {}
        #dict for condition
        # ALT func 
        if "ALT"  == param:
            while True:
                timeout  = time.time() + 2
                #continouslt listen dor messages with a 2 - second timeout 
                msg = self.drone.recv_match(type='ALTITUDE', blocking=True, timeout  = timeout)
                if msg:
                    #get only the needed data for the need of using
                    output_msg = {
                        "ATL" : msg.altitude_relative
                    }    
                    print("Local ALT data receive successfully!")
                    break
                else:
                    print("Timeout waiting for message. No ALT found!")
                    break
        # GPS func
        if "GPS" == param :
            #Scan the data stream and search for GPS coordinate
            while True:
                #get the abs timeout time by using real time in the instant of the code occur
                timeout = time.time() + 2
                # Continuously listen for messages with a 2-second timeout
                msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout = timeout)
                if msg:
                    #Scan the data and take only lat lon and alt data that needed for the position estimation
                    output_msg = {
                        'lon' : msg.lon,
                        'lat' : msg.lat,
                        'MSL_alt' : msg.relative_alt
                    }
                    break
                else:
                    output_msg = {'data' : 'NAN'}
                    break  # Exit the loop after timeout
        # Battery check up func
        if "BAT" == param :
            while True:
                timeout  = time.time() + 2
                #continouslt listen dor messages with a 2 - second timeout 
                msg = self.drone.recv_match(type='SYS_STATUS', blocking=True, timeout  = timeout)
                if msg:
                    #get only the needed data for the need of using
                    output_msg = {
                        "BAT_LV" : msg.battery_remaining
                    }    
                    break
                else:
                    output_msg = {'data' : 'NAN'}
                    break
        if "SENSOR_STATE" == param:
            while True:
                timeout  = time.time() + 2
                #continouslt listen dor messages with a 2 - second timeout 
                msg = self.drone.recv_match(type='SYS_STATUS', blocking=True, timeout  = timeout)
                if msg:
                    #get only the needed data for the need of using
                    output_msg = {
                        "SENSOR_HEALTH" : msg.onboard_control_sensors_health
                    }    
                    break
                else:
                    output_msg = {'data' : 'NAN'}
                    break
        return output_msg
    def setPara(self):
        pass
