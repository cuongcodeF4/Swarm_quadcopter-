# python3.6

import random
import json
from paho.mqtt import client as mqtt_client


broker = 'broker.emqx.io'
port = 1883
TopicPuzzle = "puzzle"
# Generate a Client ID with the subscribe prefix.
client_id = f'Drone 2'
username = 'emqx'
password = 'cuong'


def connect_mqtt() -> mqtt_client:
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


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        msg_recv = msg.payload.decode()
        msg_recv_dict = json.loads(msg_recv)
        msg_drone2 = msg_recv_dict["drone2"]
        print(f"Received `{msg_drone2}` from `{msg.topic}` topic")

    client.subscribe(TopicPuzzle)
    client.on_message = on_message


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    run()
