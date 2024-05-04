import time
import json
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import QtCore, QtGui
from paho.mqtt import client as mqtt_client
from paho.mqtt import packettypes as props
from queue import Queue
from SymbolicName import *
import threading
import ujson
from datetime import datetime
import pymavlinkFunction


class droneMQTT(object):
    def __init__(self,client_id,broker="mqtt.eclipseprojects.io",port =1883,username="swarmDrone",password="flyIsOkay"):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.payload = None
        self.Client = mqtt_client.Client(self.client_id,protocol=5)  # Use the MQTT version 5
        self.queueData  = Queue()
        self.log = Queue()
        self.status = CONNECT_FAILED
    def connectBroker(self,prop=None,typeClient=TYPE_CLIENT_NORMAL):
        
        """Connect a client to the broker.
        prop: (MQTT v5.0 only) a Properties instance setting the MQTT v5.0 properties
        to be included. Optional - if not set, no properties are sent.
        """
        def on_connect(client, userdata, flags, reasonCode, properties):
            self.status = CONNECT_SUCCESS
            self.log.put( self.client_id + " connected to MQTT Broker: " +str(reasonCode))
            print("[INFO]Connected to MQTT Broker!")
            print("[INFO]Status",str(reasonCode) )
        
        def on_publish(client, userdata, mid):
            self.log.put( self.client_id + " published message")
            #print("[INFO]Message published with MID "+str(mid))
        def on_disconnect( clinet,userdata, rc, properties):
            self.log.put( self.client_id + " disconnected with Broker and rc = "+ str(rc))
            print(self.client_id +" disconnected with MQTT Broker! with rc = " + str(rc))
            
        self.Client.username_pw_set(self.username, self.password)
        self.Client.on_connect = on_connect
        self.Client.on_publish = on_publish
        self.Client.on_disconnect = on_disconnect

        if typeClient == TYPE_CLIENT_NORMAL:
            propLW = mqtt_client.Properties(props.PacketTypes.WILLMESSAGE)
            propLW.UserProperty = [("typeMsg",LSTWILLMSG),("nameDrone",self.client_id)]
            self.Client.will_set(DRONE_COM,payload=SUB_CONNECT ,qos=2,retain=False,properties=propLW)

        elif typeClient == TYPE_CLIENT_MASTER:
            propLW = mqtt_client.Properties(props.PacketTypes.WILLMESSAGE)
            propLW.UserProperty = [("typeMsg",MASTERLSTWIL)]
            self.Client.will_set(DRONE_COM,payload= MASTER_OFFLINE ,qos=2,retain=False,properties=propLW)
        self.Client.connect(self.broker, self.port, 5 ,clean_start =0)

    def publishMsg(self,topic,payload,prop):
        # payload= {}
        # for drone in range(3):
        #     payload[f"drone{drone+1}"] = self.payload[drone]
        # msg = json.dumps(payload)

        ####################################
        # This code to handle payload type #
        ####################################

        resultPub = self.Client.publish(topic,payload, qos=2,retain=False,properties=prop)
        # result: [0, 1]
        # status = resultPub[0]
        # if status == 0:
        #     print(f"[INFO] Message <`{payload}`> sent to topic:`{topic}`")
        # else:
        #     print(f"[ERROR] Failed to send message to topic {topic}")

    def subscribe(self,topic):
        self.Client.subscribe(topic,qos=2)
        def on_message(client, userdata, msg):
            self.queueData.put(msg) 
            #print("[INFO] Received message :{0}".format(msg.payload.decode()))
        self.Client.on_message = on_message

    def logger(self):
        pass
        # def on_log(client, userdata, level, buf):
        #     print(f"[INFO]Log level{level}: {buf}")
        # self.Client.on_log = on_log
        
    def droneInit(self,topic):
        print("[DEBUG] Drone start init...")
        self.Client.loop_start()
        propInit = mqtt_client.Properties(props.PacketTypes.PUBLISH)
        propInit.UserProperty =[("typeMsg",INITMSG),("nameDrone",self.client_id)]
        self.Client.publish(topic= topic, payload=ADD_CONNECT, qos=2,retain=False,properties=propInit)
        # self.Client.loop_stop()

    def droneDisconnect(self,topic):
        propDis = mqtt_client.Properties(props.PacketTypes.PUBLISH)
        propDis.UserProperty = [("typeMsg",INITMSG)]
        self.Client.publish(topic,SUB_CONNECT, qos=2,retain=False,properties=propDis)


class droneInstance():
    def __init__(self,drone):
        self.drone = drone
        self.droneConnected = 0
        self.masterSts   = MASTER_OFFLINE
        self.sendInit = NOT_SEND_INIT
        self.function = function()

        self.clientRecvMsg = droneMQTT(client_id = "LasWil:" + self.drone)
        thdRevDataFromMaster = threading.Thread(target= self.receive_data, args=(self.clientRecvMsg,DRONE_COM)) 
        thdRevDataFromMaster.start()

        self.clientInit    = droneMQTT(client_id="Init:"+self.drone)
        self.clientInit.connectBroker(typeClient= TYPE_CLIENT_INIT)
        time.sleep(WAIT_TO_CONNECT)

    def receive_data(self,Drone:droneMQTT,topic):
        # Initial the Client to receive command 
        Drone.connectBroker()
        Drone.logger()
        time.sleep(WAIT_TO_CONNECT)
        Drone.subscribe(topic=topic)
        Drone.Client.loop_forever()

    def handleData(self,Drone:droneMQTT): 
        if not Drone.queueData.empty():
            message = Drone.queueData.get()
            properties = dict(message.properties.UserProperty)
            if properties['typeMsg'] == CMD: 
                msg_recv= message.payload.decode()
                msg_recv_dict = ujson.loads(msg_recv)
                msg_recv = msg_recv_dict[self.drone]
                #save this for later use
                #the usage for now is not know but we can use it later
                print(f"Received `{msg_recv}`")
                if self.droneConnected == DRONE_NUMBER:
                    #run the command when ready
                    self.function.execute(msg_recv)
                    print("[DEBUG] Send data to pymavlink")
            elif properties['typeMsg'] == MASTERLSTWIL: 
                print("[DEBUG] Master online",end="\r")
                msgInit= message.payload.decode()
                self.masterSts = int(msgInit)
                if self.masterSts == MASTER_OFFLINE:
                    self.droneConnected = 0
                    print("[DEBUG] Master disconnect...")
                    self.sendInit = NOT_SEND_INIT
                else:
                    self.droneConnected = int(properties["droneConnect"])
            elif properties['typeMsg'] == LSTWILLMSG: 
                print("[Execute] Handle last will")
                msgLstWil= message.payload.decode()
                if self.masterSts == MASTER_ONLINE:
                    self.droneConnected += int(msgLstWil) 
                    print("[DEBUG] Drone was connected = ", self.droneConnected)
                print("[DEBUG] {} disconnected. Waiting connect again... ".format(properties['nameDrone']))
            elif properties['typeMsg'] == INITMSG:              
                msgInit= message.payload.decode()
                if self.masterSts == MASTER_ONLINE:
                    self.droneConnected += int(msgInit) 
                    print("[DEBUG] Drone was connected = ", self.droneConnected)
            elif properties['typeMsg'] == GPSINFO:
                pass
        def on_log(client, userdata, level, buf):
            print(f"[INFO]Log level{level}: {buf}")
        self.Client.on_log = on_log

class Command ():
    #########data transfer to the master will retain as dict form ############
    def __init__(self):
        #init needed object
        self.MQTT = droneMQTT()
        self.mavlink = pymavlinkFunction.MAV()
        self.GPS = pymavlinkFunction.GPS()
        #init needed variable
        self.HANDLE_DATA = None
        self.outputData = None
    def comCheckUp(self):
        #send ACK bit to the drone master
        ACKbit = {
            "STATE" : "ACK" + self.MQTT.client_id
        }
        self.outputData = ACKbit
    def posReport(self):
        trial = 0
        while self.outputData and trial <= MAX_TRIAL:
            #get the needed pos coordinate of the droene it self
            self.outputData = self.mavlink.getValue("GPS")
            self.outputData.update(self.mavlink.getValue("ALT"))
            trial += 1
    def sysReport(self,valueType):
        trial = 0
        #scan for all the needed value to see what system value the user wanna get
        while not self.outputData and trial <= MAX_TRIAL:
            if valueType == "BAT":
                self.outputData = self.mavlink.getValue("BAT")
            elif valueType == "SEN":
                self.outputData = self.mavlink.getValue("SENSOR_STATE")
            trial += 1
    ############ CONTROL COMMAND ############
    ############ GPS AVOIDANCE ############
    def sendGPSfeedback(self):
        #take the GPS data first
        feedbackData = self.GPS.packingData()
        #pub the data in to the master
        self.MQTT.connectBroker()
        time.sleep(WAIT_TO_CONNECT)
        self.MQTT.publishMsg(self.topic,feedbackData,prop=GPSINFO)
        self.MQTT.Client.loop_forever()

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
                self.mavlink.arm(1)
            elif self.HANDLE_DATA["CMD"] == "TAKE_OFF":
                self.mavlink.takeoff(self.HANDLE_DATA["ALT"])
            #scan to see if the return feedback data is needed or not
            if self.outputData:
                self.MQTT.connectBroker()
                time.sleep(WAIT_TO_CONNECT)
                self.MQTT.publishMsg(self.topic,self.outputData,prop=DRONE_COM)
                self.MQTT.Client.loop_forever()
        if DATA["TYPE"] == UNIT:
            #scan to make sure that the drone is in controlled
            if self.MQTT.client_id in DATA["UNIT_CMD"]["UNIT_ENABLE"]:
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
                    self.mavlink.arm(1)
                elif CMD == "TAKE_OFF":
                    self.mavlink.takeoff(self.HANDLE_DATA["ALT"])
                #scan to see if the return feedback data is available, if yes return it
                if self.outputData:
                    self.MQTT.connectBroker()
                    time.sleep(WAIT_TO_CONNECT)
                    self.MQTT.publishMsg(self.topic,self.outputData,prop=DRONE_COM)
                    self.MQTT.Client.loop_forever()



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
    '''
    def __init__(self):
        self.MQTT = droneMQTT()
        self.Command = Command()

    def execute(self,DICT_DATA):
        #start the thread and run the need code
        if DICT_DATA:
            dataExecuteThread = threading.Thread(target=self.Command.handle(), args=(DICT_DATA))
            dataExecuteThread.start()
    def report(self):
        pass
