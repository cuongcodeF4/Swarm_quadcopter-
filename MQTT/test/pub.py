# python 3.6

import random
import time
import json
from paho.mqtt import client as mqtt_client

DRONE_NUMBER = 3 

payload = {}

msgPayload1 = {'force':4000,'position':1}
msgPayload2 = {'force':1000,'position':2}
msgPayload3 = {'force':2000,'position':3}
# for number in range(DRONE_NUMBER):
    
messagePayload = [msgPayload1,msgPayload2,msgPayload3]

broker = '192.168.1.145'     # your ip address  - you can find it by : Go to  "Command Prompt" and input "ipconfig" to get your ip address 
port = 1883
# Topic_Direction = "direction"
# Topic_Arm = "arm"
# Topic_throttle = "throttle"
TopicPuzzle = "puzzle"
# TopicDrone2 = "drone2"
# TopicDrone3 = "drone3"
# Topic= [TopicDrone1,TopicDrone2,TopicDrone3]
# Generate a Client ID with the publish prefix.
client_id = f'Master '
username = 'emqx'
password = 'cuong'

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish(client):
    payload= {}
    for drone in range(DRONE_NUMBER):
        payload[f"drone{drone+1}"] = messagePayload[drone]
    msg = json.dumps(payload)
    
    result = client.publish(TopicPuzzle, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{msg}` to topic:`{TopicPuzzle}`")
    else:
        print(f"Failed to send message to topic {TopicPuzzle}")

def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)
    client.loop_stop()


if __name__ == '__main__':
    run()
