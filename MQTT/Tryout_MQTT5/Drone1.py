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
    global masterSts,sendInit
    if not Drone.queueData.empty():
        print("[DEBUG] Drone received a message")
        message = Drone.queueData.get()
        properties = message.properties
        for _, value in properties.UserProperty:
                if value == CMD:
                    msg_recv= message.payload.decode()
                    msg_recv_dict = ujson.loads(msg_recv)
                    msg_recv = msg_recv_dict["drone1"]
                    print(f"Received `{msg_recv}`")
                elif value == MASTERLSTWIL:
                    print("[Execute] Handle master init message")
                    msgInit= message.payload.decode()
                    masterSts = int(msgInit)
                    if masterSts == MASTER_OFFLINE:
                        print("[DEBUG] Master disconnect...")
                        sendInit = NOT_SEND_INIT
                    
if __name__ == '__main__':
    print("[DEBUG]Drone1 start receive message from Master")
    clientRecvMsg = droneMQTT(client_id="Drone1")
    thdRevDataFromMaster = threading.Thread(target=receive_data, args=(clientRecvMsg,DRONE_COM,)) 
    thdRevDataFromMaster.start()

    clientInit    = droneMQTT(client_id="DroneInit1")
    clientInit.connectBroker(typeClient= TYPE_CLIENT_INIT)
    time.sleep(WAIT_TO_CONNECT)
    
    # thdHandleData = threading.Thread(target=handleData, args=(clientRecvMsg,))
    # thdHandleData.start()
    while True:
        print("[DEBUG] Value sendInit= {}".format(sendInit),end="\r")
        if masterSts == MASTER_ONLINE and sendInit == NOT_SEND_INIT:
            sendInit = SEND_INIT_SUCCESS
            clientInit.droneInit(DRONE_COM)
            print("CHECKKKKKKKK")
            clientInit.logger()
        handleData(clientRecvMsg)

 