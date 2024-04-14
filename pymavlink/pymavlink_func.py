#khai bao thu vien can thiet
from pymavlink import mavutil
import time

# type of command will be use, make fucntion of those command
#some simple command like: take off, arm
#what will dev put in there code line to run the mavlink control
#exg: MAV.takeoff(10)

#data type input {'sysCom': '"command","value"', 'droneCom': '"command","value"'}
#but the exacts data we receive is on ly thhe afer mesg 

#so that we need to build a class that have all the control algorithm
#when init the class, an mavlink object gonna be create and can be use to 



#main class
class MAV():
    def __init__(self):
        #set up the connection when the class being create 
        self.drone  = mavutil.mavlink_connection('udp:172.30.144.1:14550')
    def heartbeat(self):
        self.drone.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.drone.target_system, self.drone.target_component))

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

    #get value
    #user input in a ;ist of data and para user wanna take out
    #The function will scan through all the para and get all the info need for the listed para
    #then it will sum it up in a dict and output that out for other func to use
<<<<<<< HEAD
    def getValue(self,param_group):
=======
    def getValue(self,param):
>>>>>>> cuong
        #scan for all the Attribute in the param_group, how to check?
        #how can we sure that all the data that we receive is right with the attribute?
        #create and empty dict that store all the data
        dict_data = {}
        #dict for condition
        # ALT func 
<<<<<<< HEAD
        if "ALT" in param_group:
=======
        if "ALT"  == param:
>>>>>>> cuong
            while True:
                timeout  = time.time() + 2
                #continouslt listen dor messages with a 2 - second timeout 
                msg = self.drone.recv_match(type=mavutil.mavlink.ALTITUDE, blocking=True, timeout  = timeout)
                if msg:
                    #get only the needed data for the need of using
                    output_msg = msg.altitude_local    
<<<<<<< HEAD
                    print("ALT data receive successfully!")
=======
                    print("Local ALT data receive successfully!")
>>>>>>> cuong
                else:
                    print("Timeout waiting for message. No GPS found!")
                    break
        # GPS func
<<<<<<< HEAD
        if "GPS" in param_group:
=======
        if "GPS" == param :
>>>>>>> cuong
            #Scan the data stream and search for GPS coordinate
            while True:
                #get the abs timeout time by using real time in the instant of the code occur
                timeout = time.time() + 2
                # Continuously listen for messages with a 2-second timeout
                msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout = timeout)
                if msg:
                    # Process the received GPS data if there is data in the parameter 
                    #DO SOME THINGS
                    #Scan the data and take only lat lon and alt data that needed for the position estimation
                    output_msg = {
                        'lon' : msg.lon,
                        'lat' : msg.lat,
                        'MSL_alt' : msg.relative_alt
                    }
                    #exg:
                    print("GPS Coordinate receive success!")
                    #decode func will be written in the MMQTT for least processing in the drone MCU it self
                else:
                    print("Timeout waiting for message. No GPS found!")
                    break  # Exit the loop after timeout
        # Battery check up func
<<<<<<< HEAD
        if "BAT" in param_group:
=======
        if "BAT" == param :
>>>>>>> cuong
            pass
        return output_msg
    #func two

