from droneMQTT import droneMQTT
from queue import Queue
import time
import threading

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
    DroneA = droneMQTT(client_id="Drone1")
    DroneB = droneMQTT(client_id="Drone2")

    def receive_data(Drone1:droneMQTT,topic):
        # Initial the Client to receive command 
        Drone1.connectBroker()
        time.sleep(WAIT_TO_CONNECT)
        MsgDrone1 = Drone1.subscribe(topic=topic)
        Drone1.Client.loop_forever()
    
    def handleData(Drone1:droneMQTT):
        while True:    
            if not Drone1.queueData.empty():
                message = Drone1.queueData.get()
                print("[Execute] Get message from Broker:", message.payload.decode())
                properties = message.properties
                for key, value in properties.UserProperty:
                        if key == 'Test':
                            print("Giá trị của thuộc tính 'Test' là:", value)
            
    
    thread1 = threading.Thread(target=receive_data, args=(DroneA,DRONE_COM,)) 
    thread2 = threading.Thread(target=handleData, args=(DroneA,)) 
    # Start threads
    thread1.start()
    thread2.start()

    print("Main thread exiting")
