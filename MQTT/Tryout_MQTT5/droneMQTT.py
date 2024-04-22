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
from uiSysDrone import MyWindow,MasterInit
from datetime import datetime
import pymavlinkFunction

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
            pass
            #print("[INFO]Message published with MID "+str(mid))
        def on_disconnect( self,userdata, rc, properties):
            print("Disconnected with MQTT Broker! with rc = ",rc)
            
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

        self.clientRecvMsg = droneMQTT(client_id = "LasWil:" + self.drone)
        thdRevDataFromMaster = threading.Thread(target= self.receive_data, args=(self.clientRecvMsg,DRONE_COM,)) 
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
                print(f"Received `{msg_recv}`")
                if self.droneConnected == DRONE_NUMBER:
                    #this is where you take in and handle the CMD message that was sent from the master.
                    #the drone Num check was to make sure that all the drone are in connect
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
        def on_log(client, userdata, level, buf):
            print(f"[INFO]Log level{level}: {buf}")
        self.Client.on_log = on_log

class sysCom ():
    def __init__(self):
        self.MQTT = droneMQTT()
        #init client ID for later use
        self.topic = "droneFB"
        self.mavlink = pymavlinkFunction.MAV()
    def comCheckUp(self):
        #send ACK bit to the drone master
        ACKbit = "ACK" + self.MQTT.client_id
        # Initial publishing to the droneFB topic
        #only true if the code run only once
        self.MQTT.connectBroker()
        time.sleep(WAIT_TO_CONNECT)
        self.MQTT.publishMsg(self.topic,ACKbit,prop=None)
        self.MQTT.Client.loop_forever()
    def posReport(self):
        trial = 0
        while not outputData and trial <= MAX_TRIAL:
            #get the needed pos coordinate of the droene it self
            outputData = self.mavlink.getValue("GPS")
            outputData.update(self.mavlink.getValue("ALT"))
            #pub the data onto the droneFB with it own clientID contain in it
            #let just say that the clientID already be embedded in the msg it self
            trial += 1
            if outputData:
                #after having the data send it to the master to show it on the UI for the user to access.
                self.MQTT.connectBroker()
                time.sleep(WAIT_TO_CONNECT)
                self.MQTT.publishMsg(self.topic,outputData,prop=None)
                self.MQTT.Client.loop_forever()
                break
    def sysReport(self,valueType):
        trial = 0
        #scan for all the needed value to see what system value the user wanna get
        while not outputData and trial <= MAX_TRIAL:
            if valueType == "BAT":
                outputData = self.mavlink.getValue("BAT")
            elif valueType == None:
                pass
            elif valueType == None:
                pass
            trial += 1
            if outputData:
                self.MQTT.connectBroker()
                time.sleep(WAIT_TO_CONNECT)
                self.MQTT.publishMsg(self.topic,outputData,prop=None)
                self.MQTT.Client.loop_forever()
                break
    
    def decode():
        #scan the msg and excute the right command on the pixhawk
        pass

class conCom ():
    def __init__(self, topic, msg):
        self.MQTT = droneMQTT()
        self.mavlink = pymavlinkFunction.MAV()
    def arm(self, state):
        self.mavlink.arm(state=1)
    def decode():
        #scan the msg and excute the right command on the pixhawk
        pass

class function():
    def __init__(self, topic, cmd):
        MQTT = droneMQTT()
        systemCommand = sysCom()
        controlCommand = conCom()
        self.topic = topic
        self.cmd = cmd
        pass
    def excute(self):
        pass
    def report(self):
        pass
















