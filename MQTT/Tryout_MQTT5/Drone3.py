from droneMQTT import droneMQTT
from queue import Queue
import time
import threading
import ujson
from SymbolicName import *

lastWillMsg = SYS_INVALID
masterSts   = MASTER_OFFLINE
sendInit = NOT_SEND_INIT

def receive_data(Drone:droneMQTT,topic):
    # Initial the Client to receive command 
    Drone.connectBroker()
    Drone.logger()
    time.sleep(WAIT_TO_CONNECT)
    Drone.subscribe(topic=topic)
    Drone.Client.loop_forever()

def handleData(Drone:droneMQTT): 
    global masterSts,sendInit,droneConnected
    if not Drone.queueData.empty():
        message = Drone.queueData.get()
        properties = message.properties
        for _, value in properties.UserProperty:
                if value == CMD:
                    msg_recv= message.payload.decode()
                    msg_recv_dict = ujson.loads(msg_recv)
                    msg_recv = msg_recv_dict["drone3"]
                    print(f"Received `{msg_recv}`")
                    if droneConnected == DRONE_NUMBER:
                        print("[DEBUG] Send data to pymavlink")
                elif value == MASTERLSTWIL:
                    print("[DEBUG] Master online",end="\r")
                    msgInit= message.payload.decode()
                    masterSts = int(msgInit)
                    if masterSts == MASTER_OFFLINE:
                        droneConnected = 0
                        print("[DEBUG] Master disconnect...")
                        sendInit = NOT_SEND_INIT
                elif value == LSTWILLMSG:
                    print("[Execute] Handle last will")
                    msgLstWil= message.payload.decode()
                    if masterSts == MASTER_ONLINE:
                        droneConnected += int(msgLstWil) 
                        print("[DEBUG] Drone was connected = ", droneConnected)
                elif value == INITMSG:            
                    msgInit= message.payload.decode()
                    if masterSts == MASTER_ONLINE:
                        droneConnected += int(msgInit) 
                        print("[DEBUG] Drone was connected = ", droneConnected)
                    
if __name__ == '__main__':
    droneConnected = 0
    print("[DEBUG]Drone3 start receive message from Master")
    clientRecvMsg = droneMQTT(client_id="Drone3")
    thdRevDataFromMaster = threading.Thread(target=receive_data, args=(clientRecvMsg,DRONE_COM,)) 
    thdRevDataFromMaster.start()

    clientInit    = droneMQTT(client_id="DroneInit3")
    clientInit.connectBroker(typeClient= TYPE_CLIENT_INIT)
    time.sleep(WAIT_TO_CONNECT)
    
    # thdHandleData = threading.Thread(target=handleData, args=(clientRecvMsg,))
    # thdHandleData.start()
    while True:
        if masterSts == MASTER_ONLINE and sendInit == NOT_SEND_INIT:
            sendInit = SEND_INIT_SUCCESS
            clientInit.droneInit(DRONE_COM)
            clientInit.logger()
        handleData(clientRecvMsg)

 