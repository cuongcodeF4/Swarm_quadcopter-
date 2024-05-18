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

class MQTTProtocol(object):
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
            """Handle connection events."""
            self.status = CONNECT_SUCCESS
            self.log.put( self.client_id + " connected to MQTT Broker: " +str(reasonCode))
            print("[INFO]Connected to MQTT Broker!")
            print("[INFO]Status",str(reasonCode) )
     
        
        def on_publish(client, userdata, mid):
            """Handle publish events."""
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
        self.Client.connect(self.broker, self.port, 5,clean_start =0)

    def publishMsg(self,topic,payload,prop):
        """Publish a message to a topic."""

        resultPub = self.Client.publish(topic,payload, qos=2,retain=False,properties=prop)
        #result: [0, 1]
        status = resultPub[0]
        if status == 0:
             print(f"[INFO] Message <`{payload}`> sent to topic:`{topic}`")
        else:
             print(f"[ERROR] Failed to send message to topic {topic}")

    def subscribe(self,topic):
        """Subscribe to a topic."""
        self.Client.subscribe(topic,qos=2)
        def on_message(client, userdata, msg):
            self.queueData.put(msg) 
            #print("[INFO] Received message :{0}".format(msg.payload.decode()))
        self.Client.on_message = on_message

    def logger(self):
        """Log events."""
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
    def droneSendInfo(self,topic,info):
        print("[DEBUG] Drone start send info...")
        idDrone = self.client_id.split(":")[1]
        self.Client.loop_start()
        propInfo = mqtt_client.Properties(props.PacketTypes.PUBLISH)
        propInfo.UserProperty =[("typeMsg",INFO),("nameDrone",idDrone)]
        self.Client.publish(topic= topic, payload=info, qos=2,retain=False,properties=propInfo)
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
                if timeInstance - GPS_Instance >= 5:
                    gps = self.droneMavLink.getValue("GPS")
                    try:
                        yawDroneMain = self.droneMavLink.getValue("YAW")
                        yawDroneMain = (round(yawDroneMain,0))
                        sysReport["YAW"] = yawDroneMain
                    except:
                        pass
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
