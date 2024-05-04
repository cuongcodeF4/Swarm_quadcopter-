import pymavlinkFunction
from SymbolicName import *
import time
import threading


msg_recv = {
    "TYPE" : "ALL or UNIT",
    "ALL_CMD" : {
        "CMD" : 'value',
        'SYS_REPORT' : 'BAT',
        'ALT' : 'alt value',
        'LON' : 'lon value',
        'LAT' : 'lat value',
    },
    'UNIT_CMD' : {
        "UNIT_ENABLE" : [1111],
        'CLIENT_DATA' : {
            'CMD' : 'value',
            'SYS_REPORT' : 'GPS',
            'ALT' : 'alt value',
            'LON' : 'lon value',
            'LAT' : 'lat value',
        }
    }
}

class Command ():
    #########data transfer to the master will retain as dict form ############
    def __init__(self):
        #init needed object
        #self.MQTT = droneMQTT()
        #self.mavlink = pymavlinkFunction.MAV()
        #self.GPS = pymavlinkFunction.GPS()
        #init needed variable
        self.HANDLE_DATA = None
        self.outputData = None
        self.ID = 1111
    def comCheckUp(self):
        #send ACK bit to the drone master
        ACKbit = {
            #"STATE" : "ACK" + self.MQTT.client_id
            "STATE" : "comCheckUp running"
        }
        self.outputData = ACKbit
    def posReport(self):
        trial = 0
        while self.outputData and trial <= MAX_TRIAL:
            #get the needed pos coordinate of the droene it self
            #self.outputData = self.mavlink.getValue("GPS")
            #self.outputData.update(self.mavlink.getValue("ALT"))
            self.outputData = {
                    "GPS" : "VALUE",
                    "ALT" : "VALUE"                }
            trial += 1
    def sysReport(self,valueType):
        trial = 0
        #scan for all the needed value to see what system value the user wanna get
        while not self.outputData and trial <= MAX_TRIAL:
            if valueType == "BAT":
                #self.outputData = self.mavlink.getValue("BAT")
                self.outputData = {
                    "BAT" : "VALUE"
                }
            elif valueType == "SEN":
                #self.outputData = self.mavlink.getValue("SENSOR_STATE")
                self.outputData = {
                    "SENSOR_STATE" : "VALUE"
                }
            trial += 1
    ############ CONTROL COMMAND ############
    ############ GPS AVOIDANCE ############
    #def sendGPSfeedback(self):
        #take the GPS data first
        #feedbackData = self.GPS.packingData()
        #pub the data in to the master
        #self.MQTT.connectBroker()
        #time.sleep(WAIT_TO_CONNECT)
        #self.MQTT.publishMsg(self.topic,feedbackData,prop=GPSINFO)
        #self.MQTT.Client.loop_forever()

    ############ handling function ############
    def handle(self, DATA):
        if DATA["TYPE"] == ALL:
            #get the data
            self.HANDLE_DATA = DATA["ALL_CMD"] #this become a dictionary
            ############# SYSTEM COMMAND ############
            if self.HANDLE_DATA["CMD"] == "COMMUNICATION_CHECKUP":
                self.comCheckUp()
            elif self.HANDLE_DATA["CMD"] == "POSITION_REPORT":
                self.posReport()
            elif self.HANDLE_DATA["CMD"] == "SYSTEM_REPORT":
                self.sysReport(self.HANDLE_DATA["SYS_REPORT"])
            ############# CONTROL COMMAND ############
            if self.HANDLE_DATA["CMD"] == "ARM":
                #self.mavlink.arm(1)
                self.outputData = "ARMED"
            elif self.HANDLE_DATA["CMD"] == "TAKE_OFF":
                #self.mavlink.takeoff(self.HANDLE_DATA["ALT"])
                self.outputData = "TAKE_ OFF " + str(self.HANDLE_DATA["ALT"])
            #scan to see if the return feedback data is needed or not
            if self.outputData:
                #self.MQTT.connectBroker()
                #time.sleep(WAIT_TO_CONNECT)
                #self.MQTT.publishMsg(self.topic,self.outputData,prop=DRONE_COM)
                #self.MQTT.Client.loop_forever()
                print(self.outputData)
        if DATA["TYPE"] == UNIT:
            #scan to make sure that the drone is in controlled
            if self.ID in DATA["UNIT_CMD"]["UNIT_ENABLE"]:
                #get the command for easy use 
                CMD = DATA["UNIT_CMD"]["CLIENT_DATA"]["CMD"]
                self.HANDLE_DATA = DATA["UNIT_CMD"]["CLIENT_DATA"]
                ############# SYSTEM COMMAND ############
                if CMD == "COMMUNICATION_CHECK_UP":
                    self.comCheckUp()
                elif CMD == "POSITION_REPORT":
                    self.posReport()
                elif CMD == "SYSTEM_REPORT":
                    self.sysReport(self.HANDLE_DATA["SYS_REPORT"])
                ############# CONTROL COMMAND ############
                if CMD == "ARM":
                    #self.mavlink.arm(1)
                    self.outputData = "ARMED"
                elif CMD == "TAKE_OFF":
                    #self.mavlink.takeoff(self.HANDLE_DATA["ALT"])
                    self.outputData = "TAKE_OFF " + str(self.HANDLE_DATA["ALT"])
                #scan to see if the return feedback data is available, if yes return it
                if self.outputData:
                    #self.MQTT.connectBroker()
                    #time.sleep(WAIT_TO_CONNECT)
                    #self.MQTT.publishMsg(self.topic,self.outputData,prop=DRONE_COM)
                    #self.MQTT.Client.loop_forever()
                    print(self.outputData)



############ Function ############
class function():
    #data format
    '''
msg_recv = {
    “TYPE” : “ALL or UNIT”,
    “ALL_CMD” : {
        “CMD” : “value”,
        “SYS_REPORT” : “BAT or GPS, etc”,
        “ALT” : “alt value”,
        “LON” : “lon value”,
        “LAT” : “lat value”,
        ……
    },
    “UNIT_CMD” : {
        "UNIT_ENABLE" : [all the unit ID]
        “CLIENT_DATA” : {
            “CMD” : “value”,
            “SYS_REPORT” : “BAT or GPS, etc”,
            “ALT” : “alt value”,
            “LON” : “lon value”,
            “LAT” : “lat value”,
        }
    }
}

def takeoff():
    if 

    '''
    def __init__(self):
        #self.MQTT = droneMQTT()
        self.Command = Command()

    def execute(self,DICT_DATA):
        #start the thread and run the need code
        if DICT_DATA:
            dataExecuteThread = threading.Thread(target=self.Command.handle(), args=(DICT_DATA))
            dataExecuteThread.start()
    def report(self):
        pass



if __name__ == '__main__':
    function = function()
    function.execute()