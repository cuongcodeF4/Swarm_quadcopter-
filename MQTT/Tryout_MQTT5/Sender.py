from droneMQTT import droneMQTT
from enum import Enum
import time
import ujson
import paho.mqtt.properties as props
import paho.mqtt.reasoncodes as reasonCodes
import paho.mqtt.packettypes as packetTypes
import threading
from queue import Queue
from SymbolicName import *
from MainUiSysDrone import MyWindow,MasterInit
from datetime import datetime

lastWillMsg = SYS_INVALID
LstData = ["command1","command2","command3"]


msgPayload1 = {'position':1,'accel':11,'vel':111}
msgPayload2 = {'position':2,'accel':22,'vel':222}
msgPayload3 = {'position':3,'accel':33,'vel':333}

# for number in range(DRONE_NUMBER):
    
messagePayload = [msgPayload1,msgPayload2,msgPayload3]
payload = {}

data= Queue()

for drone in range(DRONE_NUMBER):
    payload[f"drone{drone+1}"] = messagePayload[drone]
msg = ujson.dumps(payload)
data.put(msg)

########################################################################  UI    ##################################################################
class Master(object):
    def __init__(self):
        super().__init__()
        self.droneConnected = 0
        self.logMaster = Queue()
        self.stsConnectBroker = None
        self.droneConnectList = []
        self.connectStatus = None
    def masterConnectBroker(self):  
        print("Enter master")
        #Create a client to receive the message last will from the drones
        self.masterRecvLW    = droneMQTT(client_id="MaterLstWil")
        self.masterRecvLW.connectBroker(typeClient=TYPE_CLIENT_INIT)
        self.thdMasterRecvLW  = threading.Thread(target=self.MasterReceiveLW, args=(self.masterRecvLW,DRONE_COM,)) 
        self.thdMasterRecvLW.start()
        # Initial the Master to send command 
        self.Master           = droneMQTT(client_id="Master")
        self.Master.connectBroker(typeClient= TYPE_CLIENT_MASTER)
        self.Master.Client.loop_start()
        time.sleep(WAIT_TO_CONNECT)   # wait for master into on connect to set the var status is CONNECT_SUCCESS
        self.stsConnectBroker = self.Master.status 
        # while not (self.masterRecvLW.log.empty() and self.Master.log.empty()):
        #     self.log.put(self.masterRecvLW.log.get())
        #     self.log.put(self.Master.log.get()) 
        # self.Master.logger()
        
    def masterCheckConnect(self,droneNumber):
            if self.droneConnected < droneNumber:
                # print("DRONE CONNECT={}".format(self.droneConnected))                
                custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
                custom_properties.UserProperty = [("typeMsg",MASTERLSTWIL),("droneConnect",str(self.droneConnected))]
                self.Master.Client.publish(topic= DRONE_COM, payload= MASTER_ONLINE,qos=2, properties=custom_properties) 
            self.handleResp()     

    def masterStopConnect(self):
        self.Master.Client.loop_stop()
        self.masterRecvLW.Client.loop_stop()
        self.Master.Client.disconnect(reasonCodes.ReasonCodes( packetTypes.PacketTypes.DISCONNECT, "Disconnect", 4))
        self.masterRecvLW.Client.disconnect(reasonCodes.ReasonCodes( packetTypes.PacketTypes.DISCONNECT, "Disconnect", 4))

    def masterSendCommand(self):
        if not self.data.empty():
            print("[DEBUG] Master sends message when there are enough connections ")
            dataSend = self.data.get()
            custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
            custom_properties.UserProperty = [("typeMsg",CMD)] 
            #Send all command
            self.Master.Client.loop_start()
            self.Master.publishMsg(topic= DRONE_COM, payload= dataSend, prop =custom_properties)   
        self.handleLW(self.masterRecvLW)
    
    def MasterReceiveLW(self,masterLW:droneMQTT,topic):
        # Initial the Client to receive command 
        masterLW.connectBroker(typeClient=TYPE_CLIENT_INIT)
        masterLW.subscribe(topic=topic)
        masterLW.Client.loop_forever()

    def handleResp(self): 
        if not self.masterRecvLW.queueData.empty():
            message = self.masterRecvLW.queueData.get()
            properties = dict(message.properties.UserProperty)
            if properties['typeMsg'] == LSTWILLMSG:             
                msgLstWil= message.payload.decode()
                self.droneConnected += int(msgLstWil) 
                self.connectStatus = OFF
                idDrone = int(properties["nameDrone"].split(":")[1])
                self.droneConnectList.remove(idDrone)
                self.logMaster.put( " Drone was connected =  " + str(self.droneConnected)) 
                self.logMaster.put( "Drone"+ str(idDrone) + " disconnected. Waiting connect again... ")
                print("[INFO] Drone was connected = ", self.droneConnected)
                print("[INFO] {} disconnected. Waiting connect again... ".format(properties['nameDrone']))
            elif properties['typeMsg'] == INITMSG:             
                print("[Execute] Handle init message")
                msgInit= message.payload.decode()
                self.droneConnected += int(msgInit) 
                idDrone =  int(properties["nameDrone"].split(":")[1])
                self.connectStatus = ON
                self.droneConnectList.append(idDrone)
                self.logMaster.put( "Drone"+ str(idDrone) + " connected.")
                print("[DEBUG] Drone was connected = ", self.droneConnected)

# def MasterReceiveLW(masterLW:droneMQTT,topic):
#     # Initial the Client to receive command 
#     masterLW.connectBroker(typeClient=TYPE_CLIENT_INIT)
#     time.sleep(WAIT_TO_CONNECT)
#     masterLW.subscribe(topic=topic)
#     masterLW.Client.loop_forever()

# def handleLW(masterLW:droneMQTT): 
#     global droneConnected
#     if not masterLW.queueData.empty():
#         message = masterLW.queueData.get()
#         properties = dict(message.properties.UserProperty)
#         if properties['typeMsg'] == LSTWILLMSG:             
#             msgLstWil= message.payload.decode()
#             droneConnected += int(msgLstWil) 
#             print("[DEBUG] Drone was connected = ", droneConnected)
#             print("[DEBUG] {} disconnected. Waiting connect again... ".format(properties['nameDrone']))
            
#         elif properties['typeMsg'] == INITMSG:             
#             print("[Execute] Handle init message")
#             msgInit= message.payload.decode()
#             droneConnected += int(msgInit) 
#             print("[DEBUG] Drone was connected = ", droneConnected)
           


# if __name__ == '__main__':
#     #Initial the current drone connected to broker 
#     droneConnected = 0
#     masterRecvLW     = droneMQTT(client_id="MaterLstWil")
#     thdMasterRecvLW  = threading.Thread(target=MasterReceiveLW, args=(masterRecvLW,DRONE_COM,)) 
#     thdMasterRecvLW.start()

#     # Initial the Master to send command 
#     Master           = droneMQTT(client_id="Master")
#     Master.connectBroker(typeClient= TYPE_CLIENT_MASTER)
#     Master.logger()
#     time.sleep(WAIT_TO_CONNECT)

#     while True:       
#         if droneConnected < DRONE_NUMBER:
#             print("[DEBUG] DRONE CONNECT= ",droneConnected,end="\r")
#             custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
#             custom_properties.UserProperty = [("typeMsg",MASTERLSTWIL),("droneConnect",str(droneConnected))]
#             #Send all command
#             Master.Client.loop_start()
#             Master.Client.publish(topic= DRONE_COM, payload= MASTER_ONLINE,qos=2, properties=custom_properties)  
#             time.sleep(0.1) 
#         elif not data.empty():
#             Master.Client.disconnect(reasonCodes.ReasonCodes( packetTypes.PacketTypes.DISCONNECT, "Disconnect", 4))
           
#             # print("[DEBUG] Master sends message when there are enough connections ")
#             # dataSend = data.get()
#             # custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
#             # custom_properties.UserProperty = [("typeMsg",CMD)]  
#             # #Send all command
#             # Master.Client.loop_start()
#             # Master.publishMsg(topic= DRONE_COM, payload= dataSend, prop =custom_properties)   
#         handleLW(masterRecvLW)

