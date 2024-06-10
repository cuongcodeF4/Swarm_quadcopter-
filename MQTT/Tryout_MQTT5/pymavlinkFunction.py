#khai bao thu vien can thiet
from pymavlink import mavutil
from functions.trajectories import *
from functions.create_active_csv import create_active_csv
# from mavutil.mavlink import MAVLinkMessage
import time
import csv

# type of command will be use, make fucntion of those command
#some simple command like: take off, arm
#what will dev put in there code line to run the mavlink control
#exg: MAV.takeoff(10)

#data type input {'sysCom': '"command","value"', 'droneCom': '"command","value"'}
#but the exacts data we receive is only the afer mesg 

#so that we need to build a class that have all the control algorithm
#when init the class, an mavlink object gonna be create and can be use to 
#what 


#main class
class Mav(object):
    #contain only the fucntion to run all the pymavlink msg needed
    #no computational involve and only take in basic data 
    def __init__(self,targSys,ip):

        self.ip = ip
        self.targetSys = targSys
        #set up the connection when the class being create 
        self.drone  = mavutil.mavlink_connection(ip)
        self.drone.target_system = targSys  # Thiết lập System ID cho drone
        self.drone.target_component = 1  # Thiết lập Component ID cho drone
        #setup as the drone is waiting on connect, wait for the confirm heartbeat before doing anything
        self.drone.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.drone.target_system, self.drone.target_component))
        self.wait_drone_ready()
        #ACK bit
        self.ACK_msg = None 
        #timer variable
        self.timerInit = True


    def timeout(self, timeOutDuration):
        if self.timerInit:
            endTime = time.time() + timeOutDuration
            self.timerInit = False
        currentTime = time.time()
        if currentTime >= endTime:
            #reset timer when timer init
            self.timerInit = True
            return True
        else:
            return False

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
        #ACK handle
        #check to see if the command was send successful
        #scan for the ACK msg
        #what if the drone never reach the wanted high? 
        #check ACK
        while self.ACK_msg == None and self.timeout(2) != True:
            self.ACK_msg = self.drone.recv_match(type='COMMAND_ACK',blocking = True)
        #if successful receive the ACK msg
        if self.ACK_msg is not None and self.ACK_msg:
            ACK = self.ACK_msg.result
            if ACK == 0:
                while True:
                    #check for alt
                    instance_ALT = self.getValue("ALT")
                    if altitude - 0.1 < instance_ALT < altitude + 0.1:
                        #send ACK bit
                        print("DEBUG: MISSION SUCCESSFUL!")
                        return True
                    else:
                        print("DEBUG: MISSION FAIL TO SEND")
                        return False
        else:
            print("DEBUG: NO ACK FOUND!")
            return False
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
        while self.ACK_msg == None and self.timeout(2) != True:
            self.ACK_msg = self.drone.recv_match(type='COMMAND_ACK',blocking = True)
        #if successful receive the ACK msg
        if self.ACK_msg is not None and self.ACK_msg:
            ACK = self.ACK_msg.result
            if ACK == 0:
                #send ACK bit
                print("DEBUG: MISSION SUCCESSFUL!")
                return True
            else:
                print("DEBUG: MISSION FAIL TO SEND")
                return False
        else:
            print("DEBUG: NO ACK FOUND!")
            return False
    
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
        while self.ACK_msg == None and self.timeout(2) != True:
            self.ACK_msg = self.drone.recv_match(type='COMMAND_ACK',blocking = True)
        #if successful receive the ACK msg
        if self.ACK_msg is not None and self.ACK_msg:
            ACK = self.ACK_msg.result
            if ACK == 0:
                #send ACK bit
                print("DEBUG: MISSION SUCCESSFUL!")
                return True
            else:
                print("DEBUG: MISSION FAIL TO SEND")
                return False
        else:
            print("DEBUG: NO ACK FOUND!")
            return False
        
    
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
        while self.ACK_msg == None and self.timeout(2) !=True :
            self.ACK_msg = self.drone.recv_match(type='COMMAND_ACK',blocking = True)
        #if successful receive the ACK msg
        if self.ACK_msg is not None and self.ACK_msg:
            ACK = self.ACK_msg.result
            if ACK == 0:
                while True:
                    #check for alt
                    instance_ALT = self.getValue("ALT")
                    if instance_ALT < 0.1:
                        #send ACK bit
                        print("DEBUG: MISSION SUCCESSFUL!")
                        return True
                    else:
                        print("DEBUG: MISSION FAIL TO SEND")
                        return False
        else:
            print("DEBUG: NO ACK FOUND!")
            return False


        
    #RLT mode
    #land mode
    def RTL(self):
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,  # Confirmation
            0,  # Empty parameters (might vary for specific needs)
            0,
            0,
            0,
            0,
            0,
            0
        )
        while self.ACK_msg == None and self.timeout(2) != True:
            self.ACK_msg = self.drone.recv_match(type='COMMAND_ACK',blocking = True)
        #if successful receive the ACK msg
        if self.ACK_msg is not None and self.ACK_msg:
            ACK = self.ACK_msg.result
            if ACK == 0:
                while True:
                    #check for alt
                    instance_ALT = self.getValue("ALT")
                    if instance_ALT < 0.1:
                        #send ACK bit
                        print("DEBUG: MISSION SUCCESSFUL!")
                        return True
                    else:
                        print("DEBUG: MISSION FAIL TO SEND")
                        return False
        else:
            print("DEBUG: NO ACK FOUND!")
            return False


    ############### MISSION FUNCTION #################
    def creatorCsv(self,shapeName,diameterCir,alt,distance,posXMaster=0,posYMaster=0):
        num_repeats = 1
        shape_name=shapeName
        diameter = diameterCir
        direction = 1
        maneuver_time = 60.0
        start_x = diameterCir/2 - distance
        start_y = posYMaster
        initial_altitude = alt
        move_speed = 2.0  # m/s
        hold_time = 4.0 #s
        step_time = 0.05 #s
        output_file = "shapes/active.csv"
        create_active_csv(
            shape_name=shape_name,
            num_repeats=num_repeats,
            diameter=diameter,
            direction=direction,
            maneuver_time=maneuver_time,
            start_x=start_x,
            start_y=start_y,
            initial_altitude=initial_altitude,
            move_speed = move_speed,
            hold_time = hold_time,
            step_time = step_time,
            output_file = output_file,
        )
    def readWaypoints(self,path_to_csv):
        #-----------------------------------Read the waypoints from the CSV file-----------------------------------#
        print("[INFO] Reading waypoints from the CSV file...")
        waypoints_list = []
        with open(path_to_csv, newline="") as csvfile:
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
                waypoints_list.append((t, px, py, pz, vx, vy, vz))
        return waypoints_list, yaw
    #perform the mission
    def missionCircle(self, distanceDrones, yaw, circleDiameter, alt):
        #init shape name
        shapeName="Circle"
        # Create csv file to store parameter of each trajectory
        self.creatorCsv(shapeName, diameterCir= circleDiameter,alt=alt,distance=round(distanceDrones,0))
        #Add condition to perform this cmd 
        waypoints_list, yaw = self.readWaypoints("shapes/active.csv")
        self.arm(1)
        self.takeoff(float(alt))
        time.sleep(3)
        self.performMission(waypoints_list, yaw)
        # return ACK bit after finish the mission
        return True
    
    def recvMsgResp(self):
        while True:        
            msg = self.drone.recv_match(type='SYS_STATUS', blocking=True, timeout = 2)
            msgGps = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout = 2)
            if msg != None :
                if msg.get_srcSystem() == self.targetSys:       
                    self.msg = msg
            else:
                self.msg = None
            if msgGps != None :
                if msgGps.get_srcSystem() == self.targetSys:       
                    self.msgGps = msgGps
            else:
                self.msgGps = None
            # if msgYaw != None :
            #     if msgYaw.get_srcSystem() == 1:       
            #         self.msgYaw = msgYaw
            # else:
            #     self.msgYaw = None
    #get value
    def getValue(self,param):
        # ALT func 
        if param == "ALT":
            while msgAlt is None and self.timeout(2) != True:
                msgAlt = self.drone.recv_match(type='ALTITUDE', blocking=True, timeout = 2)
            if msgAlt is not None and msgAlt:
                outputData = msgAlt.altitude_relative 
            else:
                outputData = None
        # GPS func
        elif param == "GPS":
            while msgGps  == None and self.timeout(2) != True:
                msgGps = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout = 2)
            if msgGps is not None and msgGps:
                gps = [0]*2
                #Scan the data stream and search for GPS coordinate
                gps[0]= msgGps.lon/ 1e7
                gps[1]= msgGps.lat/ 1e7
                outputData = gps
            else:
                outputData = None
        # Battery check up func
        elif param == "BAT" and msgSys:
            while msgSys == None and self.timeout(2) != True:
                msgSys = self.drone.recv_match(type='BATTERY_STATUS', blocking=True, timeout = 2)
            if msgSys is not None and msgSys:
                outputData = msgSys.battery_remaining
            else:
                outputData = None
        #elif param == "SENSOR_STATE" and msgSys:
        #    outputData = msgSYS.onboard_control_sensors_health
        elif param == "YAW":
            self.msgYaw = self.drone.recv_match(type='ATTITUDE', blocking=True, timeout = 5)
            # Extract yaw value from the ATTITUDE message
            if self.msgYaw:
                return self.msgYaw.yaw
            else:
                return None
        return outputData