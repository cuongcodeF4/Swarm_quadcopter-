from droneMQTT import droneMQTT
from queue import Queue
import time

WAIT_TO_CONNECT = 1
ALL_DRONE_ACTIVED = 1
SYS_INVALID = 0


SYS_COM = "topicSystemCom"
DRONE_COM = "topicDroneCom"
CONTROL_COM = "topicControlCom"

TOPIC_LAST_WILL = "Drone/LastWill"

lastWillMsg = SYS_INVALID
dataDrone1 = Queue()

if __name__ == '__main__':
    # Initial the Client to receive command 
    Drone1 = droneMQTT(client_id="Drone1")
    Drone1.connectBroker()
    time.sleep(WAIT_TO_CONNECT)
    MsgDrone1 = Drone1.subscribe(topic=DRONE_COM)
    Drone1.Client.loop_forever()
    # #Receive command
    # while dataDrone1.empty() or None in dataDrone1.queue:
    #     MsgDrone1 = Drone1.subscribe(topic=DRONE_COM)
    #     if MsgDrone1 != None:
    #         dataDrone1.put(MsgDrone1)
    # Drone1.Client.loop_stop()
    # print("[INFO] Handle message :",dataDrone1.get())
