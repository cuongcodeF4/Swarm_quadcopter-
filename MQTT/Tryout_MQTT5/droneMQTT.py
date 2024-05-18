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
from MainUiSysDrone import MyWindow,MasterInit
from datetime import datetime
from pymavlinkFunction import *
from functions.export_and_plot_shape import export_and_plot_shape
from functions.trajectories import *
from functions.create_active_csv import create_active_csv
import csv


class droneMQTT(object):
    def __init__(self,client_id,broker="mqtt.eclipseprojects.io",port =1883,username="swarmDrone",password="flyIsOkay"):
        # self.uiSysDrone = MyWindow()
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

    def sendFeedbackInfo(self,lock,targSys,ip):
        with lock:
            self.droneMavLink = Mav(targSys=targSys, ip= ip)
            getBatThread = threading.Thread(target= self.droneMavLink.recvMsgResp)
            getBatThread.start()
            #init the base data when recycle
            idDrone = self.client_id.split(":")[1]
            sysReport = {
                        "BAT" : {
                            "Client_ID" : idDrone,
                            "Battery_percent":None,
                                },
                        "GPS" : {
                            "Client_ID" : idDrone,
                            "LON" : None,
                            "LAT" : None,
                                },
                        "YAW" : None
                        }

            GPS_Instance = time.time()
            BAT_Instance = time.time()
            GPS_update = False
            BAT_update = False
            while True:
                #get GPS data after 2 sec
                timeInstance = time.time()
                if timeInstance - GPS_Instance >= 1:
                    gps = self.droneMavLink.getValue("GPS")
                    sysReport["GPS"]["LON"] = gps[0]
                    sysReport["GPS"]["LAT"] = gps[1]
                    GPS_Instance =  time.time()
                    GPS_update = True
                #get BAT data after 30 sec
                if timeInstance - BAT_Instance >= 5:
                    sysReport["BAT"]["Battery_percent"] = self.droneMavLink.getValue("BAT")
                    BAT_Instance =  time.time()
                    BAT_update = True
                #send the msg
                if GPS_update or BAT_update:
                    GPS_update = False
                    BAT_update = False
                    payloadFB = ujson.dumps(sysReport)
                    self.Client.loop_start()
                    propFeedback = mqtt_client.Properties(props.PacketTypes.PUBLISH)
                    propFeedback.UserProperty =[("typeMsg",REPORTMSG),("nameDrone",idDrone)]
                    self.Client.publish(topic= DRONE_COM, payload=payloadFB, qos=2,retain=False,properties=propFeedback)

class droneInstance():
    def __init__(self,drone):
        self.drone = drone
        self.droneConnected = 0
        self.masterSts   = MASTER_OFFLINE
        self.sendInit = NOT_SEND_INIT
        self.clientRecvMsg = droneMQTT(client_id = "LasWil:" + self.drone)
        thdRevDataFromMaster = threading.Thread(target= self.receive_data, args=(self.clientRecvMsg,DRONE_COM,)) 
        thdRevDataFromMaster.start()
        self.clientInit    = droneMQTT(client_id="Init:"+self.drone)
        self.clientInit.connectBroker(typeClient= TYPE_CLIENT_INIT)
        time.sleep(WAIT_TO_CONNECT)
        self.initDecode    = False 
        self.onHandleCmd   = False
        self.checkGPS = False
        self.yawMainDrone = 0 
        

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
                if self.initDecode == False:
                    # The class to decode command receive from master
                    self.DecodeCommand = DecodeCommand(self.drone,self.clientInit,self.clientInit.droneMavLink,self.listLatitude,self.listLongitude)
                    self.initDecode = True
                msg_recv= message.payload.decode()
                msg_recv_dict = ujson.loads(msg_recv)
                print("[DEBUG]{} vs {}".format(self.droneConnected,DRONE_NUMBER))
                if self.droneConnected == DRONE_NUMBER:
                    print("[DEBUG] Send data to pymavlink")
                    handleThread = threading.Thread(target=self.DecodeCommand.handle , args=(msg_recv_dict,))
                    handleThread.start()

                    self.onHandleCmd = True
                    handleThread.join()
                    self.onHandleCmd = False

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
                except Exception as e:
                    print("An error occurred when get GPS value:", e)
            elif properties['typeMsg'] == YAWVALUE: 
                msgYaw= message.payload.decode()
                self.yawMainDrone = round(float(msgYaw),2)

class DecodeCommand ():
    #########data transfer to the master will retain as dict form ############
    def __init__(self,droneId,clientPub,pixhawk,listLat,listLon): 
        #init needed variable
        self.pixhawk = pixhawk
        self.clientPub = clientPub
        self.drone   = droneId
        self.listLat = listLat
        self.listLon = listLon
        self.HANDLE_DATA = None
        self.outputData = None
        self.firstTime = True

    ############ handling function ############
    def handle(self, command):
        assert(isinstance(self.pixhawk,Mav))
        ACK = False 
        while not ACK:
            if self.firstTime:
                guided_mode = 4
                self.pixhawk.setMode(guided_mode)
                self.firstTime = False
            if command["TYPE"] == ALL:
                #get the command
                self.HANDLE_DATA = command["ALL_CMD"] #this become a dictionary
                ############# CONTROL COMMAND ############
                if self.HANDLE_DATA["CMD"] == "Arm":
                    print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                    ACK = self.pixhawk.arm(1)
                if self.HANDLE_DATA["CMD"] == "Disarm":
                    print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                    ACK = self.pixhawk.arm(0)
                elif self.HANDLE_DATA["CMD"] == "Takeoff":
                    print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                    if self.firstTime:
                        #first time init so need to arm before fly
                        if self.pixhawk.arm(1):
                            ACK = self.pixhawk.takeoff(float(self.HANDLE_DATA["ALT"]))
                    else:
                        ACK = self.pixhawk.takeoff(float(self.HANDLE_DATA["ALT"]))
                elif self.HANDLE_DATA["CMD"] == "Land":
                    print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                    self.firstTime = True
                    ACK = self.pixhawk.land(0,0)
                elif self.HANDLE_DATA["CMD"] == "Prepare act":
                    pass
                elif self.HANDLE_DATA["CMD"] == "Circle":
                    idDrone = int(self.drone)
                    if idDrone == 1:
                        print("[INFO] Arming the drone...")
                        self.pixhawk.arm(1)
                        print("[INFO] Taking off...")
                        #initiate taking off to perform the mission
                        self.pixhawk.takeoff(float(self.HANDLE_DATA["ALT"]))
                        yawDroneMain = self.pixhawk.getValue("YAW")
                        assert(isinstance(self.clientPub,droneMQTT))
                        self.clientPub.Client.loop_start()
                        propFeedback = mqtt_client.Properties(props.PacketTypes.PUBLISH)
                        propFeedback.UserProperty =[("typeMsg",YAWVALUE),("nameDrone",idDrone)]
                        self.clientPub.Client.publish(topic= DRONE_COM, payload=str(yawDroneMain), qos=2,retain=False,properties=propFeedback)
                        ACK = True
                    else:
                        #get all the needed value
                        diameter = float(self.HANDLE_DATA["PARA"])
                        alt      = float(self.HANDLE_DATA["ALT"])
                        distanceDrones = self.distance(self.listLat[0],self.listLon[0],self.listLat[idDrone-1],self.listLon[idDrone-1]) 
                        yaw  = self.calculateYaw(self.listLat[0],self.listLon[0],self.listLat[idDrone-1],self.listLon[idDrone-1])
                        #send debug message
                        print("[DEBUG] Distance:", distanceDrones)  
                        print("[DEBUG] yaw:", yaw)  
                        ACK = self.pixhawk.missionCircle(
                            distanceDrones, 
                            yaw, 
                            shapeName="Circle", 
                            diameterCir= diameter,
                            alt=alt,
                        )
                        
            elif command["TYPE"] == UNIT:
                self.HANDLE_DATA = command["UNIT_CMD"] #this become a dictionary
                #scan to make sure that the drone is in controlled
                if self.drone in self.HANDLE_DATA["UNIT_SELECTED"]:
                    ############# CONTROL COMMAND ############
                    print("[DEBUG] Drone{} receive unit command".format(self.drone))
                    if self.HANDLE_DATA["CMD"] == "Arm":
                        print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                        ACK = self.pixhawk.arm(1)
                    elif self.HANDLE_DATA["CMD"] == "Disarm":
                        print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                        ACK = self.pixhawk.arm(0)
                    elif self.HANDLE_DATA["CMD"] == "Takeoff":
                        print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                        if self.firstTime:
                            #first time init so need to arm before fly
                            if self.pixhawk.arm(1):
                                ACK = self.pixhawk.takeoff(float(self.HANDLE_DATA["ALT"]))
                        else:
                            ACK = self.pixhawk.takeoff(float(self.HANDLE_DATA["ALT"]))
                    elif self.HANDLE_DATA["CMD"] == "Land":
                        print("[DEBUG] Command receive:",self.HANDLE_DATA["CMD"])
                        self.firstTime = False
                        ACK = self.pixhawk.land(0,0)
                    elif self.HANDLE_DATA["CMD"] == "Prepare act":
                        pass


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

        













