#khai bao thu vien can thiet
from pymavlink import mavutil
from mavutil.mavlink import MAVLinkMessage
import time

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
class MAV():
    #contain only the fucntion to run all the pymavlink msg needed
    #no computational involve and only take in basic data 
    def __init__(self):
        #set up the connection when the class being create 
        self.drone  = mavutil.mavlink_connection('udp:172.30.144.1:14550')
        #setup as the drone is waiting on connect, wait for the confirm heartbeat before doing anything
        self.drone.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.drone.target_system, self.drone.target_component))

    def checkACK(self):
        #wait to see if the matching message was present in the data stream
        ACK = self.drone.wait_for_message(MAVLINK_MSG_ID_DO_ACKNOWLEDGE, timeout=1.0)
        if ACK:
            #there is matching message in the data stream and it ready to be read out
            ACK = self.drone.recvmsg().get_payload()[1]
            #confirm that command was sent to ardupilot
            if ACK == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return True
            else:
                return False

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
        ACK_check = self.drone.wait_for_message(MAVLINK_MSG_ID_DO_ACKNOWLEDGE, timeout = 1)
        if ACK_check:
            ACK_check = self.drone.recv_msg().get_payload()[1]
            if ACK_check == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                while True:
                    #check for alt
                    instance_ALT = self.getValue("ALT")
                    if  altitude - 0.1 < instance_ALT < altitude + 0.1:
                        #send ACK bit
                        ACK  = True
                        break
                    else:
                        ACK =  False
        return ACK
    
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
        return self.checkACK()
    
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
        return self.checkACK()
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
        #ACK handle
        #check to see if the command was send successful
        #scan for the ACK msg
        #what if the drone never reach the wanted high? 
        ACK_check = self.drone.wait_for_message(MAVLINK_MSG_ID_DO_ACKNOWLEDGE, timeout = 1)
        if ACK_check:
            ACK_check = self.drone.recv_msg().get_payload()[1]
            if ACK_check == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                while True:
                    #check for alt
                    instance_ALT = self.getValue("ALT")
                    if instance_ALT < 0.1:
                        #send ACK bit
                        ACK  = True
                        break
                    else:
                        ACK =  False
        return ACK
        return ACK

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
        #ACK handle
        #check to see if the command was send successful
        #scan for the ACK msg
        #what if the drone never reach the wanted high? 
        ACK_check = self.drone.wait_for_message(MAVLINK_MSG_ID_DO_ACKNOWLEDGE, timeout = 1)
        if ACK_check:
            ACK_check = self.drone.recv_msg().get_payload()[1]
            if ACK_check == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                while True:
                    #check for alt
                    instance_ALT = self.getValue("ALT")
                    if instance_ALT < 0.1:
                        #send ACK bit
                        ACK  = True
                        break
                    else:
                        ACK =  False
        return ACK
    
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
                #continouslt listen dor messages with a 2 - second timeout 
                msg = self.drone.recv_match(type='ALTITUDE', blocking=True)
                if msg:
                    #get only the needed data for the need of using
                    output_msg = {
                        "ATL" : msg.altitude_relative
                    }    
                    break
                else:
                    output_msg = {
                        'ALT' : 'NAN'
                    }
                    break
        # GPS func
        if "GPS" == param :
            #Scan the data stream and search for GPS coordinate
            while True:
                # Continuously listen for messages with a 2-second timeout
                msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg:
                    #Scan the data and take only lat lon and alt data that needed for the position estimation
                    output_msg = {
                        'LON' : msg.lon,
                        'LAT' : msg.lat,
                    }
                    break
                else:
                    output_msg = {
                        'LON' : 'NAN',
                        'LAT' : 'NAN'
                    }
                    break  # Exit the loop after timeout
        # Battery check up func
        if "BAT" == param :
            while True:
                #continouslt listen dor messages with a 2 - second timeout 
                msg = self.drone.recv_match(type='SYS_STATUS', blocking=True)
                if msg:
                    #get only the needed data for the need of using
                    output_msg = {
                        "Battery_percent" : msg.battery_remaining
                    }    
                    break
                else:
                    output_msg = {
                        'Battery_percent' : 'NAN'
                    }
                    break
        if "SENSOR_STATE" == param:
            while True:
                #continouslt listen dor messages with a 2 - second timeout 
                msg = self.drone.recv_match(type='SYS_STATUS', blocking=True)
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
class getFeedbackData():
    def __init__(self) -> None:
        #set up the connection when the class being create 
        self.drone  = mavutil.mavlink_connection('udp:172.30.144.1:14550')
        #setup as the drone is waiting on connect, wait for the confirm heartbeat before doing anything
        self.drone.wait_heartbeat()
        self.mavlink = MAV()
    #packing the GPS cooordinate data and ready to be send out 
    def GPS(self):
        #getting the long lat and alt of the drone itself
        output_data = self.mavlink.getValue("ALT")
        GPSdata = self.mavlink.getValue("GPS")
        return output_data.update(GPSdata)
    def BAT(self):
        output_msg = self.mavlink.getValue("BAT")
        return output_msg