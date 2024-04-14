import time
import json
from paho.mqtt import client as mqtt_client
from paho.mqtt import packettypes as props
from queue import Queue
from SymbolicName import *
import threading
import ujsonIS_IN_CONTROLLED = False 


"""
system frame:
trong hàm main của drone thì drone sẽ cần một hàm nhận lệnh từ hai kênh 
điều này giải quyết bằng function.subscribe(...)
sau đó thì phần topic và msg sẽ được lưu lại trong lib
Người dùng khi này sẽ cho chạy excute
Phần hàm excute này sẽ quét các giá trị topic và msg heienj thời và cho chạy, (các giá trị topic này chỉ bao gồm sysCom và conCom).
vì các giá trị topic hiện hành đã được lưu nên excute chỉ cần lấy và quét (cần add fail safe nếu không có topic và msg để excute chạy)
khi chạy thì excute sẽ quét topic trc và chạy những hàm phù hợp, sysCom hoặc conCom.
từ trong những hàm này mà excute sẽ tự innit thread và function trong thread sao cho phù hợp.
Phần việc còn lại là của hàm decode trong mỗi function
-.-
Hàm report 
chỉ dùng cho việc gửi data vào kênh droneFB
áp dụng tương tự với trình tự thực hiện hàm excute.
"""

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
    def connectBroker(self,prop=None,typeClient=TYPE_CLIENT_NORMAL):
        """Connect a client to the broker.
        prop: (MQTT v5.0 only) a Properties instance setting the MQTT v5.0 properties
        to be included. Optional - if not set, no properties are sent.
        """
        def on_connect(client, userdata, flags, reasonCode, properties):
            print("[INFO]Connected to MQTT Broker!")
            print("[INFO]Status",reasonCode )
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

        self.clientRecvMsg = droneMQTT(client_id = self.drone)
        thdRevDataFromMaster = threading.Thread(target= self.receive_data, args=(self.clientRecvMsg,DRONE_COM,)) 
        thdRevDataFromMaster.start()

        self.clientInit    = droneMQTT(client_id=self.drone + "Init")
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
        def on_log(client, userdata, level, buf):
            print(f"[INFO]Log level{level}: {buf}")
        self.Client.on_log = on_log

class sysCom ():
    def __init__(self, topic, msg):
        #create an object for later use
        drone = droneMQTT()
        self.topic = topic
        self.msg = topic
    def decode():
        #scan the msg and excute the right command on the pixhawk
        pass

class conCom ():
    def __init__(self, topic, msg):
        #create an object for later use
        drone = droneMQTT()
        self.IS_IN_CONTROLLED = IS_IN_CONTROLLED
        self.topic = topic
        self.msg = msg
    def decode():
        #scan the msg and excute the right command on the pixhawk
        pass

class function():
    def __init__(self) -> None:
        pass
    def excute(self):
        pass
    def report(self):
        pass
















