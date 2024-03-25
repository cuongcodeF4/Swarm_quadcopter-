import time
import json
from paho.mqtt import client as mqtt_client

TOPIC_LAST_WILL = "Drone/LastWill"
SYS_INVALID = 0


class droneMQTT(object):
    def __init__(self,client_id,broker="mqtt.eclipseprojects.io",port = 1883,username="swarmDrone",password="flyIsOkay"):
        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.payload = None

    def connectBroker(self,prop=None):
        """Connect a client to the broker.
        prop: (MQTT v5.0 only) a Properties instance setting the MQTT v5.0 properties
        to be included. Optional - if not set, no properties are sent.
        """
        def on_connect(client, userdata, flags, reasonCode, properties):
            print("[INFO]Connected to MQTT Broker!")
            print("[INFO]Status",reasonCode )
        def on_publish(client, userdata, mid):
            print("[INFO]Message published with MID "+str(mid))
        def on_disconnect( self,userdata, rc, properties):
            print("Disconnected with MQTT Broker!= ",rc)

        self.Client = mqtt_client.Client(self.client_id,protocol=5)  # Use the MQTT version 5
        self.Client.username_pw_set(self.username, self.password)
        self.Client.on_connect = on_connect
        self.Client.on_publish = on_publish
        self.Client.will_set(TOPIC_LAST_WILL,payload=SYS_INVALID ,qos=1,retain=False)
        self.Client.connect(self.broker, self.port, 60,clean_start =0, properties=prop)
        return self.Client
    
    def publishMsg(self,topic,payload):
        # payload= {}
        # for drone in range(3):
        #     payload[f"drone{drone+1}"] = self.payload[drone]
        # msg = json.dumps(payload)

        ####################################
        # This code to handle payload type #
        ####################################

        resultPub = self.Client.publish(topic,payload, qos=2,retain=False)
        # result: [0, 1]
        status = resultPub[0]
        if status == 0:
            print(f"[INFO] Message <`{payload}`> sent to topic:`{topic}`")
        else:
            print(f"[ERROR] Failed to send message to topic {topic}")

    def subscribe(self,topic):
        self.Client.subscribe(topic,qos=2)
        def on_message(client, userdata, msg):
            msg_recv = msg.payload.decode()
            self.payload = msg_recv
            if userdata is not None:
                print("[INFO] Received message :{0} with user data : ".format(msg_recv,userdata))
            else: 
                print("[INFO] Received message :{0}".format(msg_recv))
            return msg_recv
        self.Client.on_message = on_message
        return self.payload
    def logger(self):
        def on_log(client, userdata, level, buf):
            print(f"[INFO]Log level{level}: {buf}")
        self.Client.on_log = on_log














