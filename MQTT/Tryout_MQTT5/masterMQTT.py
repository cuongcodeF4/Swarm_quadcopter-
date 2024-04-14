import time
import json
from paho.mqtt import client as mqtt_client
from paho.mqtt import packettypes as props
from queue import Queue
from SymbolicName import *
import threading
import ujson

#define main object 
class masterMQTT():
    def __init__(self,client_id = "Master",broker="mqtt.eclipseprojects.io",port = 1883):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.payload = None
        self.Client = mqtt_client.Client(self.client_id, protocol=5)   #create an empty client object for later us in the code it self
        self.queueData = Queue()

    #connection handle
    def connectBroker(self,prop=None, typeClient = TYPE_CLIENT_MASTER):
        """Connect a client to the broker.
        prop: (MQTT v5.0 only) a Properties instance setting the MQTT v5.0 properties
        to be included. Optional - if not set, no properties are sent.
        """
        def on_connect(client, userdata, flags, reasonCode, properties):
            print("[INFO]Connected to MQTT Broker!")
            print("[INFO]Status",reasonCode )
        def on_publish(client, userdata, mid):
            pass
        def on_disconnect( self,userdata, rc, properties):
            print("Disconnected with MQTT Broker!= ",rc)

        
        self.Client.username_pw_set(self.username, self.password)               # Use the MQTT version 5
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
    
    #publish handle
    def publishMsg(self,topic,payload):
        
        # payload= {}
        # for drone in range(3):
        #     payload[f"drone{drone+1}"] = self.payload[drone]
        # msg = json.dumps(payload)

        ####################################
        # This code to handle payload type #
        ####################################
        #publish the info into the broker
        resultPub = self.Client.publish(topic,payload, qos=2,retain=False)
        """
        result: [0, 1]
        status = resultPub[0]
        if status == 0:
            print(f"[INFO] Message <`{payload}`> sent to topic:`{topic}`")
        else:
            print(f"[ERROR] Failed to send message to topic {topic}")
        """

    #sub scribe handle
    def subscribe(self,topic):
        self.Client.subscribe(topic,qos=2)
        def on_message(client, userdata, msg):
            self.queueData.put(msg)
        self.Client.on_message = on_message
    
    def logger(self):
        '''
        def on_log(client, userdata, level, buf):
            print(f"[INFO]Log level{level}: {buf}")
        self.Client.on_log = on_log
        '''
        pass

    def droneInit(self,topic):
        print("[DEBUG] Drone start init...")
        self.Client.loop_start()
        propInit = mqtt_client.Properties(props.PacketTypes.PUBLISH)
        propInit.UserProperty = [("typeMsg",INITMSG)]
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

        self.clientRecvMsg = masterMQTT(client_id = self.drone)
        thdRevDataFromMaster = threading.Thread(target= self.receive_data, args=(self.clientRecvMsg,DRONE_COM,)) 
        thdRevDataFromMaster.start()

        self.clientInit = masterMQTT(client_id=self.drone + "Init")
        self.clientInit.connectBroker(typeClient= TYPE_CLIENT_INIT)
        time.sleep(WAIT_TO_CONNECT)

    def receive_data(self,Drone:masterMQTT,topic):
        # Initial the Client to receive command 
        Drone.connectBroker()
        Drone.logger()
        time.sleep(WAIT_TO_CONNECT)
        Drone.subscribe(topic=topic)
        Drone.Client.loop_forever()

    def handleData(self,Drone:masterMQTT): 
        if not Drone.queueData.empty():
            message = Drone.queueData.get()
            properties = dict(message.properties.UserProperty)
            if properties['typeMsg'] == CMD: 
                msg_recv= message.payload.decode()
                msg_recv_dict = ujson.loads(msg_recv)
                msg_recv = msg_recv_dict[self.drone]
                print(f"Received `{msg_recv}`")
                if self.droneConnected == DRONE_NUMBER:
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





##################### SYSTEM COMMAND ##########################
class sysCom():
    def __init__(self, droneNum, wait_to_connect = 1, timeout = 1, topic = "sysCom", subTopic = "droneFB"):
        self.wait_to_connect = wait_to_connect
        self.timeout = timeout
        self.droneNum = droneNum
        self.topic = topic
        self.subTopic = subTopic
        #init MQTT object for further use
        self.master = masterMQTT()

    def comCheckUp(self):                                               #Check up the connection between drone and master so that we know if they can send info to each other                            
        #set up as a normal publish func
        #but with the msg as a command that send to all the topic to test all the msg
        #when the master pub up the drone should answer with format "CP_droneNum" }
        #scan the string and do a check list to see if the master receive enough drone nums feed back
        msg = "comCheckUp"
        test_topic = ['sysCom', 'conCom']
        temp_msg_stack = []
        for topic in test_topic:
            # auto setup connection to the broker
            self.master.connectBroker()
            self.master.logger()
            time.sleep(self.wait_to_connect)
            #start the publishing event
            self.master.Client.loop_start()
            self.master.publishMsg(topic, msg)
            self.master.Client.loop_stop()
            timeBegin = time.time()
            while timeout < 2:
                #get the msg from the droneFB channel
                msg = self.master.subscribe(self.subTopic)
                temp_msg_stack.extend(msg)
                #timeout function
                timeEnd = time.time() 
                timeout = timeEnd - timeBegin 
            #Scan the msg to count out the amount of drone that is responsive
            droneNumCounted = len(temp_msg_stack)
            if self.droneNum == droneNumCounted:
                print(f"comCheckUp on '{topic}' successful")
                print("Continue to the next topic")
            else:
                #create an empty drone num pack to contain the success drone info
                successDroneNum = ""
                #report back the fail drone to the terminal so that the user know
                #scan for all the success drone
                for drone_msg in temp_msg_stack:
                    successDroneNum += ", " + drone_msg.split('_')[-1]
                print(f"Failure comCheckUp on '{topic}'")
                print("Success connection on Drone: %s", successDroneNum)
                print("Continue to the next topic")
    def posReport(self):
        #send the command to the sysCom topic to announce for the drone
        self.master.connectBroker()
        self.master.Client.loop_start()
        self.master.publishMsg(self.topic, "posReport")
        
        self.master.Client.loop_stop()
        time.sleep(1)
        counter = 0
        #get the return msg from the drone in the swarm
        dronePackPos = {}
        while counter < self.droneNum:
            #simply count the amount of msg return to know whether the number of drone receive the msg is enough
            msg = self.master.subscribe(self.subTopic)
            if msg:
                dronePackPos.update(msg)
                counter += 1
        #after that run through a decode func and safe func to hold the data for later use
    def sysReport(self):
        #send the command to the sysCom topic to announce for the drone
        self.master.connectBroker()
        self.master.Client.loop_start()
        self.master.publishMsg(self.topic, "sysReport")
        self.master.Client.loop_stop()
        time.sleep(1)
        counter = 0
        #get the return msg from the drone in the swarm
        #the msg format that we receive gonna be a dict that contain the data as
        #exg_data = {
        #    'Num': 1
        #    'data': {
        #        'bat':12,
        #        'sensor state':'good'
        #    }
        #}
        #take the msg from the droneFB topic
        while counter < self.droneNum:             #if success receive a mss
            #because it already have QoS so there's no duplicate in the data it self
            msg = self.master.subscribe(self.subTopic)
            if msg:
                counter += 1                    #count up to track the number of drone data receive
                #display the info out to the user
                print(f"sysReport drone {msg['Num']}")
                print(f"Data = {msg['data']}")
        #need to add timeout and fail case
    def setGPSOrigin(self):
        #connect to the broker and send out the command
        self.master.connectBroker()
        self.master.Client.loop_start()
        self.master.publishMsg(
            self.topic, 
            "setGPS"
        )
        self.master.Client.loop_stop()
        time.sleep(1)
        droneACK = []
        #simply count the amount of msg return to know whether the number of drone receive the msg is enough
        while len(droneACK) < self.droneNum:
            droneAckMsg = self.master.subscribe(self.subTopic)
            if droneAckMsg:
                #pretend the msg is another dict object that have droneID and pos value
                droneACK.append(droneAckMsg)
        print("Drone pack successfully set GPS origin")
    def unitSelect(self, client):
        #init connection to the broker
        self.master.connectBroker()
        self.master.Client.loop_start()
        self.master.publishMsg(
            self.topic, 
            client
        )
        self.master.Client.loop_stop()
        time.sleep(1)
        #waiting for acknowledgment
        ACK_msg = []
        while ACK_msg == None:
            ACK_msg = self.master.subscribe(self.subTopic)


##################### CONTROL COMMAND ##########################
class conCom():
    def __init__(self, droneNum, wait_to_connect = 1, timeout = 1, topic = "conCom", subTopic = "droneFB"):
        self.wait_to_connect = wait_to_connect
        self.timeout = timeout
        self.droneNum = droneNum
        self.topic = topic
        self.subTopic = subTopic
        #init MQTT class
        self.master = masterMQTT()
        
    def arm (self):
        #init connection to the broker
        self.master.connectBroker()
        self.master.Client.loop_start()
        #let say that it send the mission out successfull but is all the drone receive the msg?
        self.master.publishMsg(
            self.topic,
        )
        self.master.Client.loop_stop()
        time.sleep(1)
        
    def takeoff(self, altitude):
        #init connection to the broker
        self.master.connectBroker()
        self.master.Client.loop_start()
        self.master.publishMsg(
            self.topic,
            altitude
        )
        self.master.Client.loop_stop()
        time.sleep(1)
        #waiting for acknowledgment
        ACK_msg = []
        while ACK_msg == None:
            ACK_msg = self.master.subscribe(self.subTopic)
    def land(self):
        #init connection to the broker
        self.master.connectBroker()
        self.master.Client.loop_start()
        self.master.publishMsg(
            self.topic,
        )
        self.master.Client.loop_stop()
        time.sleep(1)
        #waiting for acknowledgment
        ACK_msg = []
        while ACK_msg == None:
            ACK_msg = self.master.subscribe(self.subTopic)
    def init(self):
        #init connection to the broker
        self.master.connectBroker()
        self.master.Client.loop_start()
        self.master.publishMsg(
            self.topic,
        )
        self.master.Client.loop_stop()
        time.sleep(1)
        #waiting for acknowledgment
        ACK_msg = []
        while ACK_msg == None:
            ACK_msg = self.master.subscribe(self.subTopic)
    def droneGoto(self, ):
        #init connection to the broker
        self.master.connectBroker()
        self.master.Client.loop_start()
        self.master.publishMsg(
            self.topic,
        )
        self.master.Client.loop_stop()
        time.sleep(1)
        #waiting for acknowledgment
        ACK_msg = []
        while ACK_msg == None:
            ACK_msg = self.master.subscribe(self.subTopic)





######################## SIDE FUNCTION ############################
class function():
    def __init__(self, droneNum):
        #format when use 
        #init the function by function = function(droneNum)
        #function.sysCom.setPara(value) or function.conCom.Arm(value)
        #init the two main topic function
        self.sysCom = sysCom(droneNum)
        self.conCom = conCom(droneNum)