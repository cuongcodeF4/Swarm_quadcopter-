
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
from CtrlSDPanel import MyWindow,MasterInit
from datetime import datetime
from pymavlinkFunction import *
from functions.export_and_plot_shape import export_and_plot_shape
from functions.trajectories import *
from functions.create_active_csv import create_active_csv
import csv
from MQTTProtocol import MQTTProtocol

class droneInstance(object):
    def __init__(self,drone):
        self.drone = drone
        self.droneConnected = 0
        self.masterSts   = MASTER_OFFLINE
        self.sendInit = NOT_SEND_INIT
        self.clientRecvMsg = MQTTProtocol(client_id = "LasWil:" + self.drone)
        thdRevDataFromMaster = threading.Thread(target= self.receive_data, args=(self.clientRecvMsg,DRONE_COM,)) 
        thdRevDataFromMaster.start()
        self.clientInit    = MQTTProtocol(client_id="Init:"+self.drone)
        self.clientInit.connectBroker(typeClient= TYPE_CLIENT_INIT)

        self.clientSendInfo   = MQTTProtocol(client_id="Info:"+self.drone)
        self.clientSendInfo.connectBroker(typeClient= TYPE_CLIENT_INFO)

        time.sleep(WAIT_TO_CONNECT)
        self.initDecode    = False 
        self.onHandleCmd   = False
        self.checkGPS = False
        self.yawMainDrone = None
        self.HANDLE_DATA = None
        self.firstTime = False
        self.queueCmd = Queue()
        self.nbrDrawCircle = 0
        self.preX = 0
        self.preY = 0
        self.droneCompleted = 0

    def receive_data(self,Drone:MQTTProtocol,topic):
        # Initial the Client to receive command 
        Drone.connectBroker()
        Drone.logger()
        time.sleep(WAIT_TO_CONNECT)
        Drone.subscribe(topic=topic)
        Drone.Client.loop_forever()

    def handleData(self,Drone:MQTTProtocol): 
        if not Drone.queueData.empty():
            message = Drone.queueData.get()
            properties = dict(message.properties.UserProperty)
            if properties['typeMsg'] == CMD: 
                msg_recv= message.payload.decode()
                msg_recv_dict = ujson.loads(msg_recv)
                print("[DEBUG]{} vs {}".format(self.droneConnected,DRONE_NUMBER))
                if self.droneConnected == DRONE_NUMBER:
                    self.queueCmd.put(msg_recv_dict)      

            elif properties['typeMsg'] == MASTERLSTWIL: 
                msgInit= message.payload.decode()
                self.masterSts = int(msgInit)
                if self.masterSts == MASTER_OFFLINE:
                    self.droneConnected = 0
                    print("[DEBUG] Master disconnect...")
                    self.sendInit = NOT_SEND_INIT
                elif self.masterSts == MASTER_ONLINE:
                    print("[DEBUG] Master online",end="\r")
                    self.droneConnected = int(properties["droneConnect"])
            elif properties['typeMsg'] == LSTWILLMSG: 
                print("[Execute] Handle last will")
                msgLstWil= message.payload.decode()
                print("[DEBUG] msgLstWil",msgLstWil)
                if self.masterSts == MASTER_ONLINE:
                    self.droneConnected += int(msgLstWil) 
                    print("[DEBUG] Drone was connected = ", self.droneConnected)
                print("[DEBUG] {} disconnected. Waiting connect again... ".format(properties['nameDrone']))
            elif properties['typeMsg'] == INITMSG:              
                msgInit= message.payload.decode()
                print("[DEBUG] msgInit",msgInit)
                if self.masterSts == MASTER_ONLINE:
                    self.droneConnected += int(msgInit) 
                    print("[DEBUG] Drone was connected = ", self.droneConnected)
            
            elif properties['typeMsg'] == REPORTMSG: 
                if self.checkGPS == False:
                    self.listLongitude = [0]*2
                    self.listLatitude  = [0]*2
                    self.checkGPS = True
                try:
                    msgReport = message.payload.decode()
                    msgReport = ujson.loads(msgReport)         
                    idDrone =  int(properties["nameDrone"])

                    # Update value of battery with corresponding id
                    if msgReport["GPS"]["LON"] != None and msgReport["GPS"]["LAT"] != None :
                        self.listLongitude[idDrone-1] = float(msgReport["GPS"]["LON"])
                        self.listLatitude[idDrone-1] = float(msgReport["GPS"]["LAT"])

                    else:
                        pass
                    if msgReport["YAW"] != None:
                        self.yawMainDrone = msgReport["YAW"]
                except Exception as e:
                    print("An error occurred when get GPS value:", e)
            elif properties['typeMsg'] == INFO:
                msgInfo = message.payload.decode()
                self.droneCompleted += int(msgInfo)
                print("[DEBUG] Drone completed the task: ",self.droneCompleted)
                if self.droneCompleted == DRONE_NUMBER:
                    print("[DEBUG] All drone completed the task")
                    self.onHandleCmd = False
                    self.droneCompleted = 0 
        if not self.queueCmd.empty():
            if self.onHandleCmd == False:
                print("[DEBUG] Send data to pymavlink")
                self.onHandleCmd = True
                handleThread = threading.Thread(target=self.decodeCmd , args=(self.queueCmd.get(),))
                handleThread.start()

    def decodeCmd(self, command):
        assert(isinstance(self.clientInit.droneMavLink,Mav))
        if self.firstTime == True:
            guided_mode = 4
            self.clientInit.droneMavLink.setMode(guided_mode)
            self.firstTime = False
        if command["TYPE"] == ALL:
            #get the command
            self.HANDLE_DATA = command["ALL_CMD"] #this become a dictionary
            ############# CONTROL COMMAND ############
            if self.HANDLE_DATA["CMD"] == "Arm":
                print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                ACK = self.clientInit.droneMavLink.arm(1)
            if self.HANDLE_DATA["CMD"] == "Disarm":
                print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                ACK = self.clientInit.droneMavLink.arm(0)
            elif self.HANDLE_DATA["CMD"] == "Takeoff":
                print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                if self.firstTime:
                    self.clientInit.droneMavLink.arm(1)
                    ACK = self.clientInit.droneMavLink.takeoff(float(self.HANDLE_DATA["ALT"]))
                else:
                    ACK = self.clientInit.droneMavLink.takeoff(float(self.HANDLE_DATA["ALT"]))
            elif self.HANDLE_DATA["CMD"] == "Land":
                print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                self.firstTime = True
                ACK = self.clientInit.droneMavLink.land(0,0)
            elif self.HANDLE_DATA["CMD"] == "Circle":
                #missionCircle()
                idDrone = int(self.drone)
                if idDrone == 1:
                    print("[INFO] Arming the drone...")
                    armACK = self.clientInit.droneMavLink.arm(1)
                    if armACK:
                        print("[INFO] Taking off...")
                        ACK = self.clientInit.droneMavLink.takeoff(float(self.HANDLE_DATA["ALT"]))
                else:
                    self.nbrDrawCircle += 1
                    diameter = float(self.HANDLE_DATA["PARA"])
                    alt      = float(self.HANDLE_DATA["ALT"])
                    # print("[DEBUG] List Lat",self.listLatitude)
                    # print("[DEBUG] List Lon",self.listLongitude)
                    distanceDrones = self.distance(self.listLatitude[0],self.listLongitude[0],self.listLatitude[idDrone-1],self.listLongitude[idDrone-1]) 
                    yaw  = self.calculateYaw(self.listLatitude[0],self.listLongitude[0],self.listLatitude[idDrone-1],self.listLongitude[idDrone-1])
                    print("[DEBUG] Distance:", distanceDrones)  
                    print("[DEBUG] Yaw:", yaw)  
                    print("[DEBUG] Đường kính = ", diameter)
                    self.clientInit.droneMavLink.missionCircle(
                        distanceDrones=distanceDrones, 
                        yaw= yaw, 
                        circleDiameter=diameter, 
                        alt=alt
                    )
            #ADD send out ACK bit
            if ACK:
                self.clientSendInfo.droneSendInfo(DRONE_COM,DONE)
        elif command["TYPE"] == UNIT:
            self.HANDLE_DATA = command["UNIT_CMD"] #this become a dictionary
            #scan to make sure that the drone is in controlled
            if self.drone in self.HANDLE_DATA["UNIT_SELECTED"]:
                ############# CONTROL COMMAND ############
                if self.HANDLE_DATA["CMD"] == "Arm":
                    print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                    ACK = self.clientInit.droneMavLink.arm(1)
                elif self.HANDLE_DATA["CMD"] == "Disarm":
                    print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                    ACK = self.clientInit.droneMavLink.arm(0)
                elif self.HANDLE_DATA["CMD"] == "Takeoff":
                    print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                    if self.firstTime:
                        self.clientInit.droneMavLink.arm(1)
                        ACK = self.clientInit.droneMavLink.takeoff(float(self.HANDLE_DATA["ALT"]))
                    else:
                        ACK = self.clientInit.droneMavLink.takeoff(float(self.HANDLE_DATA["ALT"]))
                elif self.HANDLE_DATA["CMD"] == "Land":
                    print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                    self.firstTime = True
                    ACK = self.clientInit.droneMavLink.land(0,0)
                elif self.HANDLE_DATA["CMD"] == "Prepare act":
                    pass
            #add ACK scan
            if ACK:
                self.clientSendInfo.droneSendInfo(DRONE_COM,DONE)

    def distance(self,lat1, lon1, lat2, lon2):
        R = 6371.0  
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon / 2) * math.sin(dLon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R *c*1000    # uint m
        return distance
    
    def calculateYaw(self,lat1, lon1, lat2, lon2):
        latDis = self.distance(lat1, lon2, lat2, lon2)
        lonDis = self.distance(lat2, lon1, lat2, lon2)

        #scan to see which quater the second drone belong
        if (lon2) > (lon1):
        #it's on the upper side
            if (lat2) > (lat1):
                quarter = 1
            else:
                quarter = 4
        else:
            if (lat2) > (lat1):
                quarter = 2
            else:
                quarter = 3
        #output the yaw angle
        alpha = math.degrees(math.atan(lonDis/latDis))
        if quarter  == 1:
            yaw_angle = -(180-alpha) 
        elif quarter == 2:
            yaw_angle = 180-alpha
        elif quarter == 3:
            yaw_angle = alpha
        else:
            yaw_angle = -alpha
        return round((yaw_angle),0)
