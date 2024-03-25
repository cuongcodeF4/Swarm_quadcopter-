import random
import time
import json
from paho.mqtt import client as mqtt_client
import subprocess
import re

DRONE_NUMBER = 3 


payload = {}

msgPayload1 = {'force':4000,'position':1}
msgPayload2 = {'force':1000,'position':2}
msgPayload3 = {'force':2000,'position':3}
# for number in range(DRONE_NUMBER):
    
messagePayload = [msgPayload1,msgPayload2,msgPayload3]

# ipconfig = subprocess.check_output(['ipconfig']).decode('utf-8')   # Run command: "ipconfig" in terminal and get output -> <ipconfig> variable
# ipv4_addr = re.findall(r'IPv4 Address\. . . . . . . . . . . : (\d+\.\d+\.\d+\.\d+)', ipconfig)  
# ipCurrent = str(ipv4_addr[-1] )   # Get ipv4_address in the last
broker = "mqtt.eclipseprojects.io"     # your ip address  - you can find it by : Go to  "Command Prompt" and input "ipconfig" to get your ip address 
port = 1883

TopicPuzzle = "TopicTest1"
# TopicDrone2 = "drone2"
# TopicDrone3 = "drone3"

# Topic= [TopicDrone1,TopicDrone2,TopicDrone3]
# Generate a Client ID with the publish prefix.
client_id = 'Drone1'
username = 'emqx'
password = 'cuong'


class Master():
    def __init__(self,broker,port,topic,client_id,username,password,payload):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client_id = client_id
        self.username = username
        self.password = password
        self.payload = payload
        self.run()

    def connect_mqtt(self):
        def on_connect(client, userdata, flags, reasonCode, properties):
            print("Connected to MQTT Broker!")
            print("Status",reasonCode )
        def on_disconnect( self,userdata, rc, properties):
            print("Disconnected with MQTT Broker!= ",rc)
        def on_publish(client, userdata, mid):
            print("Message published with MID " + str(mid))
        def on_log(client, userdata, level, buf):
            print(f"Log level {level}: {buf}")
        self.client = mqtt_client.Client(client_id,protocol=5)
        self.client.username_pw_set(username, password)
        self.client.on_connect = on_connect
        self.client.on_publish = on_publish
        self.client.on_log = on_log
        self.client.on_disconnect = on_disconnect
        self.client.connect("broker.hivemq.com", 1883, 60)
        return self.client
    

    def subscribe(self,client):
        def on_message(client, userdata, msg):
            msg_recv = msg.payload.decode()
            # Kiểm tra nếu là PUBREC hoặc PUBREL
            # packet_type = client._inflight_messages[msg.mid].message_type
            mqtt = mqtt_client.MQTTMessage(mid=0,topic=self.topic)
            print("Message from Master:",msg_recv)
            # msg_recv_dict = json.loads(msg_recv)
            # msg_drone2 = msg_recv_dict["drone2"]
            # print(f"Received `{msg_drone2}` from `{msg.topic}` topic")

        client.subscribe(self.topic)
        client.on_message = on_message

    def run(self):
        drone = self.connect_mqtt()
        self.subscribe(drone)
        print("Disconnecting...")
        drone.disconnect()
        time.sleep(5)
        drone = self.connect_mqtt()
        self.subscribe(drone)
        drone.loop_forever()

    

if __name__ == '__main__':
    PC = Master(broker,port,TopicPuzzle,client_id,username,password,messagePayload)








