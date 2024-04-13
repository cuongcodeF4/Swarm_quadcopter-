import time
import json
from paho.mqtt import client as mqtt_client
from paho.mqtt import packettypes as props
from queue import Queue
from SymbolicName import *

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
            propLW.UserProperty = [("typeMsg",LSTWILLMSG)]
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













