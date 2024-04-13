from droneMQTT import droneMQTT
from enum import Enum
import time
import ujson
import paho.mqtt.properties as props
import paho.mqtt.packettypes as packetTypes
import threading
from queue import Queue
from SymbolicName import *


lastWillMsg = SYS_INVALID
LstData = ["command1","command2","command3"]


msgPayload1 = {'position':1,'accel':11,'vel':111}
msgPayload2 = {'position':2,'accel':22,'vel':222}
msgPayload3 = {'position':3,'accel':33,'vel':333}

# for number in range(DRONE_NUMBER):
    
messagePayload = [msgPayload1,msgPayload2,msgPayload3]
payload = {}

data= Queue()

for drone in range(DRONE_NUMBER):
    payload[f"drone{drone+1}"] = messagePayload[drone]
msg = ujson.dumps(payload)
data.put(msg)

def MasterReceiveLW(masterLW:droneMQTT,topic):
    # Initial the Client to receive command 
    masterLW.connectBroker(typeClient=TYPE_CLIENT_INIT)
    time.sleep(WAIT_TO_CONNECT)
    masterLW.subscribe(topic=topic)
    masterLW.Client.loop_forever()

def handleLW(masterLW:droneMQTT): 
    global droneConnected
    if not masterLW.queueData.empty():
        message = masterLW.queueData.get()
        properties = message.properties
        for key, value in properties.UserProperty:
                if key=="typeMsg" and value == LSTWILLMSG:
                    print("[Execute] Handle last will")
                    msgLstWil= message.payload.decode()
                    droneConnected += int(msgLstWil) 
                    print("[DEBUG] Drone was connected = ", droneConnected)
                elif key=="typeMsg" and value == INITMSG:
                    print("[Execute] Handle init message")
                    msgInit= message.payload.decode()
                    droneConnected += int(msgInit) 
                    print("[DEBUG] Drone was connected = ", droneConnected)
                elif key=="nameDrone":
                    print("[DEBUG] {} disconnected. Waiting connect again... ".format(value))
                else: 
                    pass
                    

if __name__ == '__main__':
    #Initial the current drone connected to broker 
    droneConnected = 0
    masterRecvLW     = droneMQTT(client_id="MaterLstWil")
    thdMasterRecvLW  = threading.Thread(target=MasterReceiveLW, args=(masterRecvLW,DRONE_COM,)) 
    thdMasterRecvLW.start()

    # Initial the Master to send command 
    Master           = droneMQTT(client_id="Master")
    Master.connectBroker(typeClient= TYPE_CLIENT_MASTER)
    Master.logger()
    time.sleep(WAIT_TO_CONNECT)

    while True:
        
        if droneConnected < DRONE_NUMBER:
            print("[DEBUG] DRONE CONNECT= ",droneConnected,end="\r")
            custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
            custom_properties.UserProperty = [("typeMsg",MASTERLSTWIL)]
            #Send all comman 
            Master.Client.loop_start()
            Master.Client.publish(topic= DRONE_COM, payload= MASTER_ONLINE,qos=2, properties=custom_properties)  
            time.sleep(0.5) 
        elif not data.empty():
            print("[DEBUG] Master sends message when there are enough connections ")
            dataSend = data.get()
            custom_properties = props.Properties( packetTypes.PacketTypes.PUBLISH)
            custom_properties.UserProperty = [("typeMsg",CMD)] 
            #Send all command
            Master.Client.loop_start()
            Master.publishMsg(topic= DRONE_COM, payload= dataSend, prop =custom_properties)   
        handleLW(masterRecvLW)



