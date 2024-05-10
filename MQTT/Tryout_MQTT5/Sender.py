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

########################################################################  UI    ##################################################################
class Master(object):
    def __init__(self,droneNumber):
        super().__init__()
        self.droneNumber = droneNumber
        self.droneConnected = 0
        self.logMaster = Queue()
        self.stsConnectBroker = None
        self.droneConnectList = []
        self.pixhawkConnectList= []
        self.connectStatus = None
        self.FcConnectStatus = False
        self.listBattery = [0]*self.droneNumber
        self.updateStsBat = OFF
        self.completedCheck = False
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
        
    def masterCheckConnect(self):
        if self.droneConnected < self.droneNumber:
            self.completedCheck = False
            #print("DRONE CONNECT={}".format(self.droneConnected))                
            custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
            custom_properties.UserProperty = [("typeMsg",MASTERLSTWIL),("droneConnect",str(self.droneConnected))]
            self.Master.Client.publish(topic= DRONE_COM, payload= MASTER_ONLINE,qos=2, properties=custom_properties)
        elif self.droneConnected == self.droneNumber and self.completedCheck == False:
            self.completedCheck = True
            custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
            custom_properties.UserProperty = [("typeMsg",MASTERLSTWIL),("droneConnect",str(self.droneConnected))]
            self.Master.Client.publish(topic= DRONE_COM, payload= MASTER_ONLINE,qos=2, properties=custom_properties)
            
        self.handleResp()

    def masterStopConnect(self):
        self.Master.Client.loop_stop()
        self.masterRecvLW.Client.loop_stop()
        self.Master.Client.disconnect(reasonCodes.ReasonCodes( packetTypes.PacketTypes.DISCONNECT, "Disconnect", 4))
        self.masterRecvLW.Client.disconnect(reasonCodes.ReasonCodes( packetTypes.PacketTypes.DISCONNECT, "Disconnect", 4))

    def masterSendCommand(self,queueDataSend):
        if (not queueDataSend.empty()) and (self.droneConnected == self.droneNumber):
            dataSend = queueDataSend.get()
            custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
            custom_properties.UserProperty = [("typeMsg",CMD)]
            #Send all command
            self.Master.publishMsg(topic= DRONE_COM, payload= dataSend, prop =custom_properties)          
            print("[DEBUG] Master sends message") 
    
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
                if idDrone in self.droneConnectList:
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
                if idDrone not in self.droneConnectList:
                    self.droneConnectList.append(idDrone)
                self.logMaster.put( "Drone"+ str(idDrone) + " connected.")
                print("[DEBUG] Drone was connected = ", self.droneConnected)
                
            ##################################################################
            # Here is code to handle system report e.g: BAT, SENSOR, GPS,...  #
            ##################################################################
            ######################### FORMAT #############################
            # sysReport = {
            #                 “BAT” : {
            #                     “Client_ID” : 1,
            #                     “Battery_percent” : 20 (%),
            #                     ……
            #                 },
            #                 “GPS” : {
            #                         “Client_ID” : 1,
            #                         “ALT” : “alt value”,
            #                         “LON” : “lon value”,
            #                         “LAT” : “lat value”,
            #                         }
            #                 }
            elif properties['typeMsg'] == REPORTMSG:
                try:
                    msgReport = message.payload.decode()
                    msgReport = ujson.loads(msgReport)
                    idDrone = int(msgReport["BAT"]["Client_ID"])

                    # if int(msgReport["BAT"]["Battery_percent"]) < self.preBat:
                        # self.preBat = int((msgReport["BAT"]["Battery_percent"]))
                    self.updateStsBat = ON    
                    # Update value of battery with corresponding id
                    if msgReport["BAT"]["Battery_percent"] != None:
                        try:
                            if idDrone in self.pixhawkConnectList:
                                self.pixhawkConnectList.remove(idDrone)
                                self.FcConnectStatus = True
                                self.updateStsBat = ON
                        except:
                            pass
                        self.listBattery[idDrone-1] = int(msgReport["BAT"]["Battery_percent"])
                        
                    else:
                        self.listBattery[idDrone-1] = 0
                        if idDrone not in self.pixhawkConnectList:
                            self.pixhawkConnectList.append(idDrone)
                            self.FcConnectStatus = True
                except Exception as e:
                    print("An error occurred:", e)
                    



